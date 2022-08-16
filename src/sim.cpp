#include "sim.hpp"
#include <math.h>


Sim::Sim(glm::vec2 startpos, float alength, int numcellsx, int numcellsy, float wall_alpha, float ang_alpha) { // I should fix this so it doesn't just do a grid
	
	glm::vec2 currentpos = startpos;

	for(int i = 0; i < numcellsy + 1; i++)
		for (int j = 0; j < numcellsx + 1; j++) {
			glm::vec2 currentpos = startpos + vec2{i*alength, j*alength};
			positions.emplace_back(currentpos);
	}

	PositionList::iterator cell_pos[4];

	for (int i = 0; i < numcellsy; i++)
		for (int j = 0; j < numcellsx; j++) {

			cell_pos[0] = positions.begin() + i*(numcellsx+1) + j;
			cell_pos[1] = positions.begin() + i*(numcellsx +1) + j+1;
			cell_pos[2] = positions.begin() + (i+1)*(numcellsx+1) + j+1;
			cell_pos[3] = positions.begin() + (i+1)*(numcellsx+1) + j;

			cells.emplace_back(cell_pos, glm::mat2{grate}, 4, M_PI/2, wall_alpha, ang_alpha);
			std::cout << "made a wall";
	}

}

void Sim::sim_iteration(float dt) {

	//update prodicted positions

	for (auto &position: positions) {
		position.velocity*=0.95; //Damping
		position.predict = position.position + position.velocity * dt;
	}

	for (auto &cell:cells)
		cell.sim_iteration_reset();

	float convergence_distance = 0;
	float convergence_distance_last = 0;
	float convergence_max = 0;
	int num_its = 0;

	while (num_its++ < 100) {

		convergence_distance = 0;
		convergence_max = 0;

		for (auto &cell: cells)
			cell.run_sim_iteration_walls(dt, convergence_distance, convergence_max);
		for (auto &cell: cells)
			cell.run_sim_iteration_angles(dt, convergence_distance, convergence_max);
		if (num_its % 25 ==0) {
			std::cout<<"Current convergence: "<<convergence_distance<<" and delta "<< convergence_distance - convergence_distance_last<<std::endl;
			convergence_distance_last = convergence_distance;
		}
	}

	for (auto& position: positions) {
		if (!position.pinned) {
			position.velocity.x = (position.predict.x - position.position.x) / dt;
			position.velocity.y = (position.predict.y - position.position.y) / dt;
			position.position = position.predict;
		}
	}

	for (auto& cell: cells) {

		cell.update_gtensor();
		cell.grow(dt);
	}

	std::cout<<"Current error: "<< convergence_distance <<std::endl;
	
}