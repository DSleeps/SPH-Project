#include "PendulumSystem.hpp"
#include <iostream>

namespace GLOO {

ParticleState PendulumSystem::ComputeTimeDerivative(const ParticleState& state,
                                                    float time) const {
  // Calculate gravity and the drag force
  std::vector<glm::vec3> accelerations;
  for (int i = 0; i < state.positions.size(); i++) {
    if (!is_rigid_[i]) {
      glm::vec3 v = state.velocities[i];
      glm::vec3 a = glm::vec3(0., 0., 0.);
      float m = masses_[i];

      a += g_; // Gravity
      a += -(drag_constant_ * v)/m; // Drag force
      accelerations.push_back(a);
    } else {
      accelerations.push_back(glm::vec3(0.f, 0.f, 0.f));
    }
  }
  
  // Now calculate the spring force
  for (int i = 0; i < springs_.size(); i++) {
    Spring s = springs_[i];
    glm::vec3 d = state.positions[s.p1] - state.positions[s.p2];
    
    glm::vec3 spring_force = -s.k * (glm::length(d) - s.rest_length)/(glm::length(d)) * d;
    if (!is_rigid_[s.p1]) {
      accelerations[s.p1] += spring_force/masses_[s.p1];
    }
    if (!is_rigid_[s.p2]) {
      accelerations[s.p2] += -spring_force/masses_[s.p2];
    }
  }
  
  // Now put everything in the correct particle state
  ParticleState gradient;
  for (int i = 0; i < state.positions.size(); i++) {
    gradient.positions.push_back(state.velocities[i]);
    gradient.velocities.push_back(accelerations[i]);
  }
  return gradient;
}

void PendulumSystem::AddParticle(ParticleState& state, float mass, bool is_rigid, glm::vec3 position) {
  state.positions.push_back(position);
  state.velocities.push_back(glm::vec3());
  masses_.push_back(mass);
  is_rigid_.push_back(is_rigid);
}

void PendulumSystem::SetRigid(int particle) {
  is_rigid_[particle] = true;
}

void PendulumSystem::AddSpring(int p1, int p2, float rest_length, float k) {
  Spring s;
  s.p1 = p1;
  s.p2 = p2;
  s.rest_length = rest_length;
  s.k = k;
  springs_.push_back(s);
}

} // namespace GLOO
