#ifndef GRID_H_
#define GRID_H_

#include "gloo/SceneNode.hpp"
#include "ParticleSystemBase.hpp"
#include "ParticleState.hpp"

#include <map>
#include <array>

namespace GLOO {

class Grid {
 public:
 	// p1 is a bottom corner of the box, and p2 is the diagonally
	// opposite corner on the top. This defines the whole boundary
	// of the box.
	Grid() {}
	Grid(glm::vec3 p1, glm::vec3 p2);	
	void CalculateBlobs(std::vector<glm::vec3> positions,
											std::vector<glm::vec3>& vertices,
											std::vector<unsigned int>& indices);

 private:
 	// This calculates a single primitive of a given grid cell
	void CalculatePrimitive(int x, int y, int z, 
													std::vector<glm::vec3>& vertices,
													std::vector<unsigned int>& indices);

 	// All of the values of each corner
 	std::vector<std::vector<std::vector<float>>> values_;
	
	// This will be the origin of the grid box, the bottom corner
	glm::vec3 origin_;

	// The number of grid cells in the box in each direction
	int grid_x_res_ = 100;
	int grid_y_res_ = 100;
	int grid_z_res_ = 100;
	
	// The size of each cell. This will be calculated based on the
	// size of the box.
	float cell_size_x_;
	float cell_size_y_;
	float cell_size_z_;
	
	// The radius of each point
	float radius_ = 0.1f;
	
	// The range (number of radiuses) at which to check the influence of
	// each point as radius * range
	int range_ = 2;

	// The vector of possible offsets for faces
	std::vector<std::array<int,3>> face_vector_;

	// A vector of all of face vertices
	std::vector<std::array<glm::vec3, 4>> face_vertices;

};

}
#endif
