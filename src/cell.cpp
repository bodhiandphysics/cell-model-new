
#include "cell.hpp"

void GTensor::rotate(float theta) {

	glm::mat2 rotation;
	rotation[0][0] = cos(theta);
	rotation[0][1] = -sin(theta);
	rotation[1][0] = -rotation[0][1];
	rotation[1][1] = rotation[0][0];

	tensor = rotation * tensor * inverse(rotation);  // tensor is a tensor, so transforms like this
		
}


glm::vec2 Cell::COM() {

	glm::vec2 com{0};
	float mass = 0;

	for (auto &position: positions) {

		com += position->mass * position->position;
		mass += position->mass;
	}

	return com / mass; 
}

glm::vec2 Cell::oldCOM() {

	glm::vec2 com{0};
	float mass = 0;

	for (auto &position: positions) {

		com += position->mass * position->old_position;
		mass += position->mass;
	}

	return com / mass; 
}

Cell::Cell(PositionList::iterator *wall_positions, glm::mat2 gtensor, int num_positions, float angle, float wall_alpha, float ang_alpha) {

	growth_tensor.tensor = gtensor;

	for (int i = 0; i < num_positions; i++)
		positions.push_back(wall_positions[i]);

	for (size_t i = 0; i < num_positions; i ++)
		walls.emplace_back(wall_positions[i], wall_positions[(i+1) % num_positions], wall_alpha);

	for (size_t i = 0; i < num_positions; i++)
		angconstraints.emplace_back(wall_positions[(i+1) % num_positions], 
									wall_positions[i],
									wall_positions[(i-1) % num_positions],
									angle, ang_alpha);

}


void Cell::grow(float dt) {

	for (auto &wall: walls) {

		glm::vec2 wall_vec =  wall.pos2->position - wall.pos1->position;
		glm::vec2 wall_vec_t = (mat2{1.f} + dt*growth_tensor.tensor) * wall_vec;

		wall.restlength = length(wall_vec_t);
	}
}

void Cell::update_gtensor() {


	glm::vec2 new_com = COM();
	glm::vec2 old_com = oldCOM();

	// this is a little tricky.  We need to find the minimum angle to rotate by to effectively transorm 
	// the cell last update into its current position.  What we do is minimize the funtion (R(theta)*pold - pnew)^2
	// where pold and pnew are the positions translated into the COM frames of the old and new cell respecively.
	// If we take the derivative by theta, and do some arithmatic we get that the minimum theta is give by 
	//atan(a/b), where a and b are defined as in the code. 

	float a,b; //  accumulators to find the right angle

	a = 0.f;
	b = 0.f;


	for (auto &position: positions ) {

		glm::vec2 pnew = position->position - new_com;
		glm::vec2 pold = position->position - old_com;

		a += (pnew.y * pold.x - pnew.x * pold.y);
		b += (pnew.x * pold.x + pnew.y * pold.y);
	}

	float theta = atan2(a, b);

	growth_tensor.rotate(theta);
}

void Cell::sim_iteration_reset() {

	for (auto &wall: walls)
		wall.lambda = 0.f;

	for (auto &angconstraint: angconstraints)
		angconstraint.lambda = 0.f;

	for (auto &position: positions) {

		position->old_position = position->position; 
	}
}

void Cell::run_sim_iteration_walls(float dt, float &conv_distance, float &conv_max) {

	float lin_const_weight = 1.f; 

	for (int i = 0; i < walls.size(); i++) {

		std::cout << "wall " << i << std::endl;

		auto &linconstraint = walls[i];

		float m1 = linconstraint.pos1->mass;
		float m2 = linconstraint.pos2->mass;
		vec2 pred1 = linconstraint.pos1->predict;
		vec2 pred2 = linconstraint.pos2->predict;
		vec2 diff = pred1 - pred2;

		float modalph = linconstraint.alpha/(dt*dt);

		// C(p)

		float distance =  length(diff);

		if (distance < 1e-10) 
			distance = 1e-10;

		auto constraintval = distance - linconstraint.restlength;
							
		// delC * 1/m * delCT					 

		float cinnerval = 1.f/m1 + 1.f/m2;

		//if (isnan(cinnerval)) cinnerval = 0.f;

		float deltal = (-constraintval - modalph*linconstraint.lambda) / (cinnerval + modalph);

		vec2 delta1 = (1.f/m1) * normalize(diff) * deltal;
		vec2 delta2 = -m1*delta1/m2;

		linconstraint.lambda += deltal;

		if (linconstraint.pos1->pinned || isnan(delta1.x+delta1.y)) {
			delta1 *= 0;
		}
		if (linconstraint.pos2->pinned || isnan(delta2.x+delta2.y)) {
			delta2 *= 0;
		}

		linconstraint.pos1->predict += delta1*lin_const_weight;
		linconstraint.pos2->predict += delta2*lin_const_weight;

		conv_distance += length(delta1*deltal)+length(delta2*deltal);//deltax1*deltax1 + deltay1*deltay1 + deltax2*deltax2 + deltay2*deltay2;
		//conv_max = std::max({conv_max, delta1.x, delta1.y, delta2.x, delta2.y});

		std::cout << linconstraint.pos1->predict.x << " " <<
				  linconstraint.pos1->predict.y << " " <<
				  linconstraint.pos1->predict.x << " " <<
				  linconstraint.pos2->predict.x << " " <<
				  linconstraint.pos2->predict.y << " " <<
				  std::endl;

	}

}

void Cell::run_sim_iteration_angles(float dt, float &conv_distance, float &conv_max) {

	float ang_const_weight = 1;

	for (auto &angconstraint: angconstraints) {

		float m1 = angconstraint.pos1->mass;
		float m2 = angconstraint.pos2->mass;
		float mc = angconstraint.posc->mass;
		vec2 pos1 = angconstraint.pos1->predict;
		vec2 pos2 = angconstraint.pos2->predict;
		vec2 posC = angconstraint.posc->predict;
		vec2 s1 = pos1 - posC;
		vec2 s2 = pos2 - posC;
		float modalph = angconstraint.alpha/(dt*dt);

		static int x = 0;

		x++;

		float s1ds2 = dot(s1,s2);//s1x*s2x + s1y*s2y;
		float s1xs2 = crossZ(s1,s2);//s1x*s2y - s1y*s2x;

		float constraintval = atan2(s1xs2,s1ds2) - angconstraint.theta0;

		std::cout << angconstraint.theta0 << " " << s1xs2 << " " << s1ds2 << std::endl; 

		std::cout << "constraintval " << constraintval << std::endl;
		vec2 dot_cross(s1ds2,s1xs2);

		float datan2 = 1.f / (dot(dot_cross,dot_cross));//1.f / (s1ds2*s1ds2 + s1xs2*s1xs2); 

		float delcx1 = datan2 * ((s1ds2*s2.y) - (s1xs2*s2.x));
		float delcx2 = datan2 * (-(s1ds2*s1.y) - (s1xs2*s1.x));

		float delcy1 = datan2 * (-(s1ds2*s2.x) - (s1xs2*s2.y));
		float delcy2 = datan2 * ((s1ds2*s1.x) - (s1xs2*s1.y));
		
		float delcxc = (delcx1 - delcx2);
		float delcyc = (delcy1 - delcy2);
	//			vec2 delc1(crossZ(dot_cross,s2),-dot(dot_cross,s2));
	//			vec2 delc2(-dot(dot_cross,s1),crossZ(dot_cross,s1));
	//			vec2 delcc(delc1.x)
	//			delc1*=datan2;
	//			delc2*=datan2;

		float cinnerval = (1.f/m1)*(delcx1*delcx1 + delcy1*delcy1) +
						  (1.f/m2)*(delcx2*delcx2 + delcy2*delcy2) +
						  (1.f/mc)*(delcxc*delcxc + delcyc*delcyc);


		float deltal = (-constraintval - modalph*angconstraint.lambda) / (cinnerval + modalph);



		float deltax1 = (1.f/m1)*delcx1*deltal;
		float deltax2 = (1.f/m2)*delcx2*deltal;
		float deltay1 = (1.f/m1)*delcy1*deltal;
		float deltay2 = (1.f/m2)*delcy2*deltal;
		float deltaxc = (1.f/mc)*delcxc*deltal;
		float deltayc = (1.f/mc)*delcyc*deltal;



		angconstraint.lambda += deltal;
		angconstraint.d1 = vec2(deltax1,deltay1);
		angconstraint.d2 = vec2(deltaxc,deltayc);
		angconstraint.d3 = vec2(deltax2,deltay2);

		if (angconstraint.pos1->pinned) {
			deltax1 = 0.f;
			deltay1 = 0.f;
		}
		if (angconstraint.pos2->pinned) {
			deltax2 = 0.f;
			deltay2 = 0.f;
		}
		
		if (angconstraint.posc->pinned) {
			deltaxc = 0.f;
			deltayc = 0.f;
		}


			if(isnan(deltax1))
				deltax1 = 0;
			if(isnan(deltay1))
				deltay1 = 0;
			if(isnan(deltax2))
				deltax2 = 0;
			if(isnan(deltay2))
				deltay2 = 0;
			if(isnan(deltaxc))
				deltaxc = 0;
			if(isnan(deltayc))
				deltayc = 0;


		// angconstraint.pos1->predict.x += deltax1*ang_const_weight;
		// angconstraint.pos1->predict.y += deltay1*ang_const_weight;
		// angconstraint.pos2->predict.x += deltax2*ang_const_weight;
		// angconstraint.pos2->predict.y += deltay2*ang_const_weight;
		// angconstraint.posc->predict.x += deltaxc*ang_const_weight;
		// angconstraint.posc->predict.y += deltayc*ang_const_weight;


		conv_distance += sqrt(deltax1*deltax1 + deltay1*deltay1) + sqrt(deltax2*deltax2 + deltay2*deltay2) + sqrt(deltaxc*deltaxc + deltayc*deltayc);
		//conv_max = std::max({conv_max, deltax1, deltay1, deltax2, deltay2, deltaxc, deltayc});

	}


}


