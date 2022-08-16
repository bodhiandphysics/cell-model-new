//#pragma once
#ifndef TYPE_HPP
#define TYPE_HPP
#include <glm/glm.hpp>
#include <vector>
#include <optional>
#include <iostream>
#include <set>

using namespace glm;

static const int NUMITERS = 300; //guessing?

inline float crossZ(vec2 a,vec2 b){
	return a.x*b.y - a.y*b.x;
}


	static float originallength = 20.f;
	struct Position {

		glm::vec2 position;
		glm::vec2 old_position;
		glm::vec2 predict;
		glm::vec2 velocity = vec2(0,0);
		bool pinned = false;



		float mass = .001f;

		Position(glm::vec2 pos): position(pos), old_position(pos), predict(pos) {}
		Position(glm::vec2 pos, bool ispinned): position(pos), old_position(pos), predict(pos), pinned(ispinned) {}
		Position(glm::vec2 pos, glm::vec2 avelocity, bool ispinned): position(pos), old_position(pos), predict(pos), velocity(avelocity), pinned(ispinned) {}
		
	};

	using PositionList = std::vector<Position>;

	struct AngConstraint {

		PositionList::iterator pos1, pos2, posc;
		glm::vec2 d1, d2, d3;

		float alpha;
		float lambda = 0;
		float theta0;

		AngConstraint(PositionList::iterator apos1, 
					  PositionList::iterator apos2, 
					  PositionList::iterator aposc, 
					  float analpha): pos1(apos1), pos2(apos2), posc(aposc), alpha(analpha) {

			vec2 s1 = apos1->position - aposc->position;
			vec2 s2 = apos2->position - aposc->position;
			float s1xs2 = crossZ(s1,s2);//s1.x*s2.y - s1.y*s2.x;
			float s1ds2 = dot(s1, s2);
			theta0 = atan2(s1xs2, s1ds2);
			std::cout<<atan2(s1xs2, s1ds2)<<std::endl;
		};

		AngConstraint(PositionList::iterator apos1, 
					  PositionList::iterator apos2, 
					  PositionList::iterator aposc, 
					  float atheta0, float analpha): pos1(apos1), pos2(apos2), posc(aposc), theta0(atheta0), alpha(analpha) {}

	};

	using AngConstraintList = std::vector<AngConstraint>;


	struct LinConstraint {

		PositionList::iterator pos1, pos2;

		float alpha;
		float lambda = 0;
		float restlength;
		float original_length;


		LinConstraint(PositionList::iterator apos1, PositionList::iterator apos2, float analpha): pos1(apos1), pos2(apos2), alpha(analpha) {

			restlength = glm::length(pos1->position - pos2->position);
			original_length = length(apos2->position - apos1->position);
		}

		LinConstraint(PositionList::iterator apos1, PositionList::iterator apos2, float analpha, float arestlength): pos1(apos1), pos2(apos2), alpha(analpha), restlength(arestlength) {
			original_length = length(apos2->position - apos1->position);
		}




		float currentlength() {return length(pos1->position - pos2->position);}
		

	};

	using LinConstraintList = std::vector<LinConstraint>;

#endif