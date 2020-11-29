#include "SimpleCircleSystem.hpp"

namespace GLOO {


ParticleState SimpleCircleSystem::ComputeTimeDerivative(const ParticleState& state,
                                                        float time) const {
  ParticleState gradient;
  for (int i = 0; i < state.positions.size(); i++) {
    auto p = state.positions[i];
    gradient.positions.push_back(glm::vec3(-p.y, p.x, 0.));
    gradient.velocities.push_back(glm::vec3(0., 0., 0.));
  }
  
  return gradient;
}

} // namespace GLOO

