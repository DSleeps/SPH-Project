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
		gradients_.push_back(std::vector<std::vector<glm::vec3>>());
		for (int y = 0; y < grid_y_res_; y++) {
			values_[x].push_back(std::vector<float>());
			gradients_[x].push_back(std::vector<glm::vec3>());
			for (int z = 0; z < grid_z_res_; z++) {
				values_[x][y].push_back(0.f);
				gradients_[x][y].push_back(glm::vec3(0.f));
			}
		}
	}
	
	// Populate the corner set
	corners_.push_back({0,0,1});
	corners_.push_back({1,0,1});
	corners_.push_back({1,0,0});
	corners_.push_back({0,0,0});
	corners_.push_back({0,1,1});
	corners_.push_back({1,1,1});
	corners_.push_back({1,1,0});
	corners_.push_back({0,1,0});

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
													std::vector<unsigned int>& indices,
													std::vector<glm::vec3>& normals) {
	
	for (int x = 0; x < grid_x_res_; x++) {
		for (int y = 0; y < grid_y_res_; y++) {
			for (int z = 0; z < grid_z_res_; z++) {
				values_[x][y][z] = 0.f;
				gradients_[x][y][z] = glm::vec3(0.f);
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
					gradients_[x][y][z] += (-2.f * (p - point) * pow(radius_,2.f))/(pow(glm::distance(point, p),4.f));
				}
			}
		}
	}
	// Now calculate all of the vertices and indices
	for (int x = 0; x < grid_x_res_; x++) {
		for (int y = 0; y < grid_y_res_; y++) {
			for (int z = 0; z < grid_z_res_; z++) {
				if (smooth_) {
					CalculateSmooth(x, y, z, vertices, indices, normals);
				} else {
					if (values_[x][y][z] >= 1.f)
						CalculatePrimitive(x, y, z, vertices, indices);
				}
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

typedef struct {
	glm::vec3 p[8];
	double val[8];
} GRIDCELL;

void Grid::CalculateSmooth(int x, int y, int z,
													 std::vector<glm::vec3>& vertices,
													 std::vector<unsigned int>& indices,
													 std::vector<glm::vec3>& normals) {
	int cubeindex;
	glm::vec3 vertlist[12];
	float isolevel = 1.f;
	
	GRIDCELL grid;
	int index = 0;
	for (auto c : corners_) {
		int o_x = c[0]; int o_y = c[1]; int o_z = c[2];

		float value;
		if (x + o_x == 0 || y + o_y == 0 || z + o_z == 0 ||
				x + o_x >= grid_x_res_ || y + o_y >= grid_y_res_ || z + o_z >= grid_z_res_) {
			value = 0.f;	
		} else {
			value = values_[x+o_x][y+o_y][z+o_z];
		}
		grid.val[index] = (double) value;
		grid.p[index] = glm::vec3((x+o_x) * cell_size_x_, 
												  		(y+o_y) * cell_size_y_, 
												  		(z+o_z) * cell_size_z_);
		index++;
	}
	
	cubeindex = 0;
	if (grid.val[0] < isolevel) cubeindex |= 1;
	if (grid.val[1] < isolevel) cubeindex |= 2;
	if (grid.val[2] < isolevel) cubeindex |= 4;
	if (grid.val[3] < isolevel) cubeindex |= 8;
	if (grid.val[4] < isolevel) cubeindex |= 16;
	if (grid.val[5] < isolevel) cubeindex |= 32;
	if (grid.val[6] < isolevel) cubeindex |= 64;
	if (grid.val[7] < isolevel) cubeindex |= 128;
	
	// This means that none of the corners are in
	if (edgeTable[cubeindex] == 0)
		return;

	if (edgeTable[cubeindex] & 1)
		vertlist[0] =
		VertexInterp(isolevel,grid.p[0],grid.p[1],grid.val[0],grid.val[1]);
	if (edgeTable[cubeindex] & 2)
		vertlist[1] =
		VertexInterp(isolevel,grid.p[1],grid.p[2],grid.val[1],grid.val[2]);
	if (edgeTable[cubeindex] & 4)
		vertlist[2] =
		VertexInterp(isolevel,grid.p[2],grid.p[3],grid.val[2],grid.val[3]);
	if (edgeTable[cubeindex] & 8)
		vertlist[3] =
		VertexInterp(isolevel,grid.p[3],grid.p[0],grid.val[3],grid.val[0]);
	if (edgeTable[cubeindex] & 16)
		vertlist[4] =
		VertexInterp(isolevel,grid.p[4],grid.p[5],grid.val[4],grid.val[5]);
	if (edgeTable[cubeindex] & 32)
		vertlist[5] =
		VertexInterp(isolevel,grid.p[5],grid.p[6],grid.val[5],grid.val[6]);
	if (edgeTable[cubeindex] & 64)
		vertlist[6] =
		VertexInterp(isolevel,grid.p[6],grid.p[7],grid.val[6],grid.val[7]);
	if (edgeTable[cubeindex] & 128)
		vertlist[7] =
		VertexInterp(isolevel,grid.p[7],grid.p[4],grid.val[7],grid.val[4]);
	if (edgeTable[cubeindex] & 256)
		vertlist[8] =
		VertexInterp(isolevel,grid.p[0],grid.p[4],grid.val[0],grid.val[4]);
	if (edgeTable[cubeindex] & 512)
		vertlist[9] =
		VertexInterp(isolevel,grid.p[1],grid.p[5],grid.val[1],grid.val[5]);
	if (edgeTable[cubeindex] & 1024)
		vertlist[10] =
		VertexInterp(isolevel,grid.p[2],grid.p[6],grid.val[2],grid.val[6]);
	if (edgeTable[cubeindex] & 2048)
		vertlist[11] =
		VertexInterp(isolevel,grid.p[3],grid.p[7],grid.val[3],grid.val[7]);

	// Create the triangle
	for (int i = 0; triTable[cubeindex][i] != -1; i += 3) {
		int vertex_size = vertices.size();
		glm::vec3 p1 = vertlist[triTable[cubeindex][i]];
		glm::vec3 p2 = vertlist[triTable[cubeindex][i+1]];
		glm::vec3 p3 = vertlist[triTable[cubeindex][i+2]];
		
		vertices.push_back(p1 + origin_);
		vertices.push_back(p2 + origin_);
		vertices.push_back(p3 + origin_);
		indices.push_back(vertex_size);
		indices.push_back(vertex_size+1);
		indices.push_back(vertex_size+2);
		if (smooth_normals_) {
			normals.push_back(glm::normalize(glm::vec3(gradients_[floor(p1.x/cell_size_x_)][floor(p1.y/cell_size_y_)][floor(p1.z/cell_size_z_)])));
			normals.push_back(glm::normalize(glm::vec3(gradients_[floor(p2.x/cell_size_x_)][floor(p2.y/cell_size_y_)][floor(p2.z/cell_size_z_)])));
			normals.push_back(glm::normalize(glm::vec3(gradients_[floor(p3.x/cell_size_x_)][floor(p3.y/cell_size_y_)][floor(p3.z/cell_size_z_)])));
	}
	}
}

glm::vec3 Grid::VertexInterp(double isolevel, glm::vec3 p1, glm::vec3 p2, 
								 			 			 double valp1, double valp2) {
	double mu;
	glm::vec3 p;

	if (abs(isolevel-valp1) < 0.00001)
		return(p1);
	if (abs(isolevel-valp2) < 0.00001)
		return(p2);
	if (abs(valp1-valp2) < 0.00001)
		return(p1);
	
	mu = (isolevel - valp1) / (valp2 - valp1);
	p.x = p1.x + mu * (p2.x - p1.x);
	p.y = p1.y + mu * (p2.y - p1.y);
	p.z = p1.z + mu * (p2.z - p1.z);

	return p;
}

}
