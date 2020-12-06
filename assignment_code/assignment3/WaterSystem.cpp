#include "WaterSystem.hpp"
#include <iostream>

namespace GLOO {

ParticleState WaterSystem::ComputeTimeDerivative(const ParticleState& state,
                                                    float time) const {
	std::vector<glm::vec3> accelerations;
	
	/*
	for (int i = 0; i < state.positions.size(); i++) {
		glm::vec3 a = glm::vec3(0.);
		a += g_;
		a += -(drag_constant_ * state.velocities[i])/MASS;
		accelerations.push_back(a);
	}
	*/

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
		float rho = 0.f;
		for (int j = 0; j < state.positions.size(); j++) {
			glm::vec3 d = state.positions[j] - state.positions[i];
			float d2 = pow(glm::length(d), 2.f);
			if (d2 < HSQ) {
				rho += MASS*POLY6*pow(HSQ-d2, 3.f);
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
		for (int j = 0; j < state.positions.size(); j++) {
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
		forces.push_back(pressure + visc + GRAVITY * rhos[i]);
	}

}

}
