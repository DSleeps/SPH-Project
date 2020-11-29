#ifndef SIMPLE_CIRCLE_SYSTEM_H_
#define SIMPLE_CIRCLE_SYSTEM_H_

#include "ParticleSystemBase.hpp"

namespace GLOO {
class SimpleCircleSystem : ParticleSystemBase {
 public:
  ~SimpleCircleSystem() {}

  ParticleState ComputeTimeDerivative(const ParticleState& state,
                                               float time) const override;

};

} // namespace GLOO

#endif
