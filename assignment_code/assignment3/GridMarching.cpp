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

	// Initialize the point to edge map
	point_to_edge_["000"] = {0, 1, 10};
	point_to_edge_["001"] = {7, 8, 10};
	point_to_edge_["010"] = {0, 3, 4};
	point_to_edge_["011"] = {4, 6, 8};
	point_to_edge_["100"] = {1, 2, 11};
	point_to_edge_["101"] = {7, 9, 11};
	point_to_edge_["110"] = {2, 3, 5};
	point_to_edge_["111"] = {5, 6, 9};
	
	// Initialize the edges
	edges_[0] = std::make_pair(glm::vec3(0.f, 0.f, 0.f), 
														glm::vec3(0.f, cell_size_y_, 0.f));
	edges_[1] = std::make_pair(glm::vec3(0.f, 0.f, 0.f), 
														glm::vec3(cell_size_x_, 0.f, 0.f));
	edges_[2] = std::make_pair(glm::vec3(cell_size_x_, 0.f, 0.f), 
														glm::vec3(cell_size_x_, cell_size_y_, 0.f));
	edges_[3] = std::make_pair(glm::vec3(0.f, cell_size_y_, 0.f), 
														glm::vec3(cell_size_x_, cell_size_y_, 0.f));
	edges_[4] = std::make_pair(glm::vec3(0.f, cell_size_y_, 0.f), 
														glm::vec3(0.f, cell_size_y_, cell_size_z_));
	edges_[5] = std::make_pair(glm::vec3(cell_size_x_, cell_size_y_, 0.f), 
														glm::vec3(cell_size_x_, cell_size_y_, cell_size_z_));
	edges_[6] = std::make_pair(glm::vec3(0.f, cell_size_y_, cell_size_z_), 
														glm::vec3(cell_size_x_, cell_size_y_, cell_size_z_));
	edges_[7] = std::make_pair(glm::vec3(0.f, 0.f, cell_size_z_), 
														glm::vec3(cell_size_x_, 0.f, cell_size_z_));
	edges_[8] = std::make_pair(glm::vec3(0.f, cell_size_y_, cell_size_z_), 
														glm::vec3(cell_size_x_, cell_size_y_, cell_size_z_));
	edges_[9] = std::make_pair(glm::vec3(cell_size_x_, 0.f, cell_size_z_), 
														glm::vec3(cell_size_x_, cell_size_y_, cell_size_z_));
	edges_[10] = std::make_pair(glm::vec3(0.f, 0.f, 0.f), 
														glm::vec3(0.f, 0.f, cell_size_z_));
	edges_[11] = std::make_pair(glm::vec3(cell_size_x_, 0.f, 0.f), 
														glm::vec3(cell_size_x_, 0.f, cell_size_z_));

}

void Grid::CalculateBlobs(std::vector<glm::vec3> positions,
													std::vector<glm::vec3>& vertices,
													std::vector<unsigned int>& indices) {
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
	for (int x = 0; x < grid_x_res_-1; x++) {
		for (int y = 0; y < grid_y_res_-1; y++) {
			for (int z = 0; z < grid_z_res_-1; z++) {
				CalculatePrimitive(x, y, z, vertices, indices);
			}
		}
	}
}

void Grid::CalculatePrimitive(int x, int y, int z, 
															std::vector<glm::vec3>& vertices,
															std::vector<unsigned int>& indices) {
	// Finds the edges that the primitive passes through
	std::set<int> edge_set;
	for (int o_x = 0; o_x < 2; o_x++) {
		for (int o_y = 0; o_y < 2; o_y++) {
			for (int o_z = 0; o_z < 2; o_z++) {
				// If the value is less than 1 it should not be considered
				if (values_[x+o_x][y+o_y][z+o_z] < 1.f)
					continue;

				std::string corner = std::to_string(o_x) +
														 std::to_string(o_y) +
														 std::to_string(o_z);
				for (int i = 0; i < 3; i++) {
					int edge = point_to_edge_[corner][i];

					// If the edge is already in the set you know that
					// both points have a value great than 1 so the
					// primitive doesn't pass through that edge
					if (edge_set.find(edge) == edge_set.end()) {
						edge_set.insert(edge);
					} else {
						edge_set.erase(edge);
					}
				}
			}																	
		}
	}
	
	//TODO: Add linear interpolation between corners

	// Draw a primitive between those edges
	for (int edge : edge_set) {
		glm::vec3 e_1 = edges_[edge].first;	
		glm::vec3 e_2 = edges_[edge].second;	
		glm::vec3 edge_offset = (e_1 + e_2) * 0.5f;
		glm::vec3 point = origin_ + glm::vec3(x * cell_size_x_, 
																					y * cell_size_y_, 
																					z * cell_size_z_);
		vertices.push_back(edge_offset + point);
	}
	
	if (edge_set.size() == 3) {
		// Simply form a triangle from the last 3 vertices
		indices.push_back(indices.size());		
		indices.push_back(indices.size());		
		indices.push_back(indices.size());		
	} else if (edge_set.size() == 4) { // This means the size must be 4
		int size = vertices.size();
		glm::vec3 p1 = vertices[size-4];

		// Find the diagonal point to the first point
		float max_distance = -1.f;
		int diagonal_index = -1;
		for (int i = 1; i < 4; i++) {
			glm::vec3 p2 = vertices[size-i];
			if (glm::distance(p1, p2) > max_distance)
				diagonal_index = i;
		}
		std::unordered_set<int> s {1,2,3};
		s.erase(diagonal_index);
		
		// Now construct the square
		for (int i : s) {
			indices.push_back(size - i);
			indices.push_back(size - 4);
			indices.push_back(size - diagonal_index);
		}
	}
}

}
