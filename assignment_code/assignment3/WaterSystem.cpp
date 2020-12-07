#include "WaterSystem.hpp"
#include <iostream>
#include <math.h>
#include <algorithm>
#include <chrono>

namespace GLOO {

WaterSystem::WaterSystem() {
	// Initialize the grid data structure
	int grid_width = (int) (box_width_/grid_cell_width_);
	int grid_height = (int) (box_height_/grid_cell_height_);
	for (int x = 0; x < grid_width; x++) {
		grid_.push_back(std::vector<std::vector<std::set<int>>>());
		for (int y = 0; y < grid_height; y++) {
			grid_[x].push_back(std::vector<std::set<int>>());
			for (int z = 0; z < grid_width; z++) {
				grid_[x][y].push_back(std::set<int>());
			}
		}
	}
	
	grid_width_ = grid_width;
	grid_height_ = grid_height;
}

ParticleState WaterSystem::ComputeTimeDerivative(const ParticleState& state,
                                                    float time) {
	// Reset the grid
	for (int x = 0; x < grid_.size(); x++) {
		for (int y = 0; y < grid_[0].size(); y++) {
			for (int z = 0; z < grid_[0][0].size(); z++) {
				grid_[x][y][z].clear();
			}
		}
	}

	// Assign each particle to it's respective grid cell
	for (int i = 0; i < state.positions.size(); i++) {
		glm::vec3 cell = GetGridCell(state.positions[i]);
		grid_[(int)cell.x][(int)cell.y][(int)cell.z].insert(i);
	}
	std::vector<float> pressures;
	std::vector<float> rhos;
	std::vector<glm::vec3> forces;
	CalculatePressure(state, pressures, rhos);
	CalculateForces(state, pressures, rhos, forces);

	ParticleState gradient;
	for (int i = 0; i < state.positions.size(); i++) {
		gradient.positions.push_back(state.velocities[i]);
		gradient.velocities.push_back(forces[i]/rhos[i]);
	}
	return gradient;
}

void WaterSystem::AddParticle(ParticleState& state, glm::vec3 position, glm::vec3 velocity) {
  state.positions.push_back(position);
  state.velocities.push_back(velocity);
}

void WaterSystem::CalculatePressure(const ParticleState& state, std::vector<float>& pressures, std::vector<float>& rhos) const {
	for (int i = 0; i < state.positions.size(); i++) {
		// Get the nearby particles from the grid
		float rho = 0.f;
		glm::vec3 cell = GetGridCell(state.positions[i]);
		for (int x = std::max(0,(int)cell.x-1); x < std::min(grid_width_,(int)cell.x+2); x++) {
			for (int y = std::max(0,(int)cell.y-1); y < std::min(grid_height_,(int)cell.y+2); y++) {
				for (int z = std::max(0,(int)cell.z-1); z < std::min(grid_width_,(int)cell.z+2); z++) {
					for (int j : grid_[x][y][z]) {
						glm::vec3 d = state.positions[j] - state.positions[i];
						float d2 = pow(glm::length(d), 2.f);
						if (d2 < HSQ) {
							rho += MASS*POLY6*pow(HSQ-d2, 3.f);
						}
					}
				}
			}
		}
		pressures.push_back(GAS_CONST*(rho - REST_DENS));
		rhos.push_back(rho);
	}
}

void WaterSystem::CalculateForces(const ParticleState& state,
																	std::vector<float> pressures, 
																	std::vector<float> rhos, 
																	std::vector<glm::vec3>& forces) const {
	for (int i = 0; i < state.positions.size(); i++) {
		
		glm::vec3 pressure(0.f);
		glm::vec3 visc(0.f);
		glm::vec3 cell = GetGridCell(state.positions[i]);
		for (int x = std::max(0,(int)cell.x-1); x < std::min(grid_width_,(int)cell.x+2); x++) {
			for (int y = std::max(0,(int)cell.y-1); y < std::min(grid_height_,(int)cell.y+2); y++) {
				for (int z = std::max(0,(int)cell.z-1); z < std::min(grid_width_,(int)cell.z+2); z++) {
					for (int j : grid_[x][y][z]) {
						if (i == j) {
							continue;
						}
						glm::vec3 d_vec = state.positions[i] - state.positions[j];
						float d = pow(glm::length(d_vec), 1.f);
						
						if (d < H) {
							pressure += glm::normalize(-d_vec)*MASS*(pressures[i] + pressures[j])/(2.f * rhos[j]) * SPIKY_GRAD*pow(H-d,2.f);
							visc += VISC*MASS*(state.velocities[j] - state.velocities[i])/rhos[j] * VISC_LAP*(H-d);
						}
					}
				}
			}
		}
		forces.push_back(pressure + visc + GRAVITY * rhos[i]);
	}

}

glm::vec3 WaterSystem::GetGridCell(glm::vec3 p) const {
	int x_cell = std::min(grid_width_-1, std::max(0, (int)floor((p.x + box_width_/2.f)/grid_cell_width_)));
	int y_cell = std::min(grid_height_-1, std::max(0, (int)floor((p.y + box_height_/2.f)/grid_cell_height_)));
	int z_cell = std::min(grid_width_-1, std::max(0, (int)floor((p.z + box_width_/2.f)/grid_cell_width_)));
	return glm::vec3(x_cell, y_cell, z_cell);
}

}
