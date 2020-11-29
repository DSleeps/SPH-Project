#ifndef RK4_INTEGRATOR_H_
#define RK4_INTEGRATOR_H_

#include "IntegratorBase.hpp"

namespace GLOO {
template <class TSystem, class TState>
class RK4Integrator : public IntegratorBase<TSystem, TState> {
  TState Integrate(const TSystem& system,
                   const TState& state,
                   float start_time,
                   float dt) const override {
    TState k_1 = system.ComputeTimeDerivative(state, start_time);
    TState k_2 = system.ComputeTimeDerivative(state + dt/2.*k_1, start_time + dt/2.);
    TState k_3 = system.ComputeTimeDerivative(state + dt/2.*k_2, start_time + dt/2.);
    TState k_4 = system.ComputeTimeDerivative(state + dt*k_3, start_time + dt);
    return state + (dt/6.) * (k_1 + 2*k_2 + 2*k_3 + k_4);
  }
};
}  // namespace GLOO

#endif
