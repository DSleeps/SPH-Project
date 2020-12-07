#ifndef PARTICLE_SYSTEM_BASE_H_
#define PARTICLE_SYSTEM_BASE_H_

#include "ParticleState.hpp"

namespace GLOO {
class ParticleSystemBase {
 public:
  virtual ~ParticleSystemBase() {
  }

  virtual ParticleState ComputeTimeDerivative(const ParticleState& state,
                                              float time) = 0;
};
}  // namespace GLOO

#endif
