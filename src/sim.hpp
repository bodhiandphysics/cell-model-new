#ifndef SIM_HPP
#define SIM_HPP
#include "types.hpp"
#include "cell.hpp"
struct Sim {

	PositionList positions;
	std::vector<Cell> cells;
	float wall_alpha, ang_alpha;
	float grate = 0;

	Sim(glm::vec2 startpos, float alength, int numcellsx, int numcellsy, float wall_alpha, float ang_alpha);
	
	void sim_iteration(float dt);

};
#endif
