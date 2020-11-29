#ifndef PENDULUM_SYSTEM_H_
#define PENDULUM_SYSTEM_H_

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"

namespace GLOO {

struct Spring {
  int p1;
  int p2;
  float rest_length;
  float k;
};

class PendulumSystem : public ParticleSystemBase {
 public:
  virtual ~PendulumSystem() {
  }

  ParticleState ComputeTimeDerivative(const ParticleState& state,
                                      float time) const override;

  void AddParticle(ParticleState& state, float mass, bool is_rigid, glm::vec3 position);
  void SetRigid(int particle);
  void AddSpring(int p1, int p2, float rest_length, float k);

 private:
  std::vector<float> masses_;
  std::vector<bool> is_rigid_;
  std::vector<Spring> springs_;

  glm::vec3 g_ = glm::vec3(0., -2., 0);
  float drag_constant_ = 0.3;
};
}  // namespace GLOO

#endif
