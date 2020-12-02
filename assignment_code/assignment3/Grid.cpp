#include "Grid.hpp"
#include <iostream>
#include <math.h>
#include <sstream>
#include <set>
#include <unordered_set>

namespace GLOO {

Grid::Grid(glm::vec3 p1, glm::vec3 p2) {
	// Set the origin to whichever point is the bottom
	if (p1.y < p2.y)
		origin_ = p1;
	else
		origin_ = p2;
	
	// Calculate each of the respective cell sizes
	cell_size_x_ = std::abs(p1.x - p2.x)/float(grid_x_res_);
	cell_size_y_ = std::abs(p1.y - p2.y)/float(grid_y_res_);
	cell_size_z_ = std::abs(p1.z - p2.z)/float(grid_z_res_);

	// Initialize the value vector to all 0s
	for (int x = 0; x < grid_x_res_; x++) {
		values_.push_back(std::vector<std::vector<float>>());
		for (int y = 0; y < grid_y_res_; y++) {
			values_[x].push_back(std::vector<float>());
			for (int z = 0; z < grid_z_res_; z++) {
				values_[x][y].push_back(0.f);
			}
		}
	}

	// Populate the face set
	face_vector_.push_back({1,0,0});
	face_vector_.push_back({-1,0,0});
	face_vector_.push_back({0,1,0});
	face_vector_.push_back({0,-1,0});
	face_vector_.push_back({0,0,1});
	face_vector_.push_back({0,0,-1});

	// Pre-compute all of the faces for each of these
	face_vertices.push_back({glm::vec3(1.f, 1.f, 1.f),
													 glm::vec3(1.f, -1.f, 1.f),
													 glm::vec3(1.f, -1.f, -1.f),
													 glm::vec3(1.f, 1.f, -1.f)});
	face_vertices.push_back({glm::vec3(-1.f, 1.f, 1.f),
													 glm::vec3(-1.f, -1.f, 1.f),
													 glm::vec3(-1.f, -1.f, -1.f),
													 glm::vec3(-1.f, 1.f, -1.f)});
	face_vertices.push_back({glm::vec3(1.f, 1.f, 1.f),
													 glm::vec3(-1.f, 1.f, 1.f),
													 glm::vec3(-1.f, 1.f, -1.f),
													 glm::vec3(1.f, 1.f, -1.f)});
	face_vertices.push_back({glm::vec3(1.f, -1.f, 1.f),
													 glm::vec3(-1.f, -1.f, 1.f),
													 glm::vec3(-1.f, -1.f, -1.f),
													 glm::vec3(1.f, -1.f, -1.f)});
	face_vertices.push_back({glm::vec3(1.f, 1.f, 1.f),
													 glm::vec3(1.f, -1.f, 1.f),
													 glm::vec3(-1.f, -1.f, 1.f),
													 glm::vec3(-1.f, 1.f, 1.f)});
	face_vertices.push_back({glm::vec3(1.f, 1.f, -1.f),
													 glm::vec3(1.f, -1.f, -1.f),
													 glm::vec3(-1.f, -1.f, -1.f),
													 glm::vec3(-1.f, 1.f, -1.f)});
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 4; j++) {
			face_vertices[i][j].x *= cell_size_x_/2.f;
			face_vertices[i][j].y *= cell_size_y_/2.f;
			face_vertices[i][j].z *= cell_size_z_/2.f;
		}
	}
}

void Grid::CalculateBlobs(std::vector<glm::vec3> positions,
													std::vector<glm::vec3>& vertices,
													std::vector<unsigned int>& indices) {
	
	for (int x = 0; x < grid_x_res_; x++) {
		for (int y = 0; y < grid_y_res_; y++) {
			for (int z = 0; z < grid_z_res_; z++) {
				values_[x][y][z] = 0.f;
			}
		}
	}

	float i_radius = range_*radius_;
	for (int i = 0; i < positions.size(); i++) {
		auto p = positions[i] - origin_;
		
		// Calculate the min and max grid corners in this particles influence
		int x_l = (int) std::max(0.f, ceil((p.x - i_radius)/cell_size_x_));
		int x_h = (int) std::min((float)grid_x_res_, floor((p.x + i_radius)/cell_size_x_));
		int y_l = (int) std::max(0.f, ceil((p.y - i_radius)/cell_size_y_));
		int y_h = (int) std::min((float)grid_y_res_, floor((p.y + i_radius)/cell_size_y_));
		int z_l = (int) std::max(0.f, ceil((p.z - i_radius)/cell_size_z_));
		int z_h = (int) std::min((float)grid_z_res_, floor((p.z + i_radius)/cell_size_z_));

		for (int x = x_l; x < x_h; x++) {
			for (int y = y_l; y < y_h; y++) {
				for (int z = z_l; z < z_h; z++) {
					glm::vec3 point = glm::vec3(x*cell_size_x_, y*cell_size_y_, z*cell_size_z_);
					values_[x][y][z] += pow(radius_,2.f)/pow(glm::distance(point, p),2.f);
				}
			}
		}
	}
	// Now calculate all of the vertices and indices
	for (int x = 0; x < grid_x_res_; x++) {
		for (int y = 0; y < grid_y_res_; y++) {
			for (int z = 0; z < grid_z_res_; z++) {
				if (values_[x][y][z] >= 1.f)
					CalculatePrimitive(x, y, z, vertices, indices);
			}
		}
	}
}

void Grid::CalculatePrimitive(int x, int y, int z, 
															std::vector<glm::vec3>& vertices,
															std::vector<unsigned int>& indices) {

	// Finds the faces that aren't surrounded by other blocks
	for (int i = 0; i < face_vector_.size(); i++) {
		auto vals = face_vector_[i];
		int o_x = vals[0]; int o_y = vals[1]; int o_z = vals[2];

		float value;
		if (x + o_x < 0 || y + o_y < 0 || z + o_z < 0 ||
				x + o_x >= grid_x_res_ || y + o_y >= grid_y_res_ || z + o_z >= grid_z_res_) {
			value = 0.f;		
		} else {
			value = values_[x+o_x][y+o_y][z+o_z];
		}
		
		// Draw a square
		//TODO: Add linear interpolation between faces
		if (value < 1.f) {
			for (int j = 0; j < 4; j++) {
				glm::vec3 offset_point = glm::vec3((x) * cell_size_x_, (y) * cell_size_y_, (z) * cell_size_z_);
				vertices.push_back(origin_ + face_vertices[i][j] + offset_point);
			}
			int vert_size = vertices.size();
			indices.push_back(vert_size-4);
			indices.push_back(vert_size-3);
			indices.push_back(vert_size-2);
			indices.push_back(vert_size-4);
			indices.push_back(vert_size-2);
			indices.push_back(vert_size-1);
		}
	}
	

}

}
