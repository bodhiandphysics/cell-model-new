#ifndef CELL_HPP
#define CELL_HPP
#include "types.hpp"


struct GTensor {

	glm::mat2 tensor{0};

	void rotate(float theta); 
};

struct Cell {

	std::vector<PositionList::iterator> positions;
	LinConstraintList walls;
	AngConstraintList angconstraints;
	GTensor growth_tensor;

	Cell(PositionList::iterator *wall_positions, glm::mat2 gtensor, int num_positions, float angle, float wall_alpha, float ang_alpha);

	glm::vec2 COM();
	glm::vec2 oldCOM();

	void grow(float dt);

	void update_gtensor();

	void run_sim_iteration_walls(float dt, float &conv_distance, float &conv_max);

	void run_sim_iteration_angles(float dt, float &conv_distance, float &conv_max);

	void sim_iteration_reset(); 

};
#endif