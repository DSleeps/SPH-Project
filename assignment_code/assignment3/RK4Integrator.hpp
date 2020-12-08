#ifndef RK4_INTEGRATOR_H_
#define RK4_INTEGRATOR_H_

#include "IntegratorBase.hpp"

namespace GLOO {
template <class TSystem, class TState>
class RK4Integrator : public IntegratorBase<TSystem, TState> {
	float box_width_ = 2.f; //TODO: This is also hardcoded into ParticleSystemNode
	float box_height_ = 2.f;
	float bound_damping_ = -0.3f;
	float eps = 0.0f;

  TState Integrate(TSystem& system,
                   TState& state,
                   float start_time,
                   float dt) override {
    TState k_1 = system.ComputeTimeDerivative(state, start_time);
    TState k_2 = system.ComputeTimeDerivative(state + dt/2.*k_1, start_time + dt/2.);
    TState k_3 = system.ComputeTimeDerivative(state + dt/2.*k_2, start_time + dt/2.);
    TState k_4 = system.ComputeTimeDerivative(state + dt*k_3, start_time + dt);

		// Check the boundary conditions (ie where the box is)
		TState new_state = state + (dt/6.) * (k_1 + 2*k_2 + 2*k_3 + k_4);
		for (int i = 0; i < new_state.positions.size(); i++) {
			glm::vec3 p = new_state.positions[i];
			if (p.x > box_width_/2. || p.x < -box_width_/2.) {
				new_state.velocities[i].x *= bound_damping_;
				new_state.positions[i].x = glm::clamp(new_state.positions[i].x, -box_width_/2.f+eps, box_width_/2.f-eps);
			}
			if (p.z > box_width_/2. || p.z < -box_width_/2.) {
				new_state.velocities[i].z *= bound_damping_;
				new_state.positions[i].z = glm::clamp(new_state.positions[i].z, -box_width_/2.f+eps, box_width_/2.f-eps);
			}
			if (p.y > box_height_/2. || p.y < -box_height_/2.) {
				new_state.velocities[i].y *= bound_damping_;
				new_state.positions[i].y = glm::clamp(new_state.positions[i].y, -box_height_/2.f+eps, box_height_/2.f-eps);
			}
		}
    return new_state;
  }
};
}  // namespace GLOO

#endif
