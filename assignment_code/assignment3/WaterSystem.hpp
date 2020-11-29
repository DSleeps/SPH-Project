#ifndef WATER_SYSTEM_H_
#define WATER_SYSTEM_H_

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"

namespace GLOO {

 // solver parameters
const static float REST_DENS = 10.f; // rest density
const static float GAS_CONST = 20.f; // const for equation of state
const static float H = .16f; // kernel radius
const static float HSQ = H*H; // radius^2 for optimization
const static float MASS = .65f; // assume all particles have the same mass
const static float VISC = 2.50f; // viscosity constant

// smoothing kernels defined in Müller and their gradients
const static float POLY6 = 3.15f/(65.f*3.1415*pow(H, 9.f));
const static float SPIKY_GRAD = -.45f/(3.1415*pow(H, 6.f));
const static float VISC_LAP = .45f/(3.1415*pow(H, 6.f));

class WaterSystem : public ParticleSystemBase {
 public:

	virtual ~WaterSystem() {
  }

  ParticleState ComputeTimeDerivative(const ParticleState& state,
                                      float time) const override;

  void AddParticle(ParticleState& state, glm::vec3 position, glm::vec3 velocity);
 private:
	void CalculatePressure(const ParticleState& state, std::vector<float>& pressures, std::vector<float>& rhos) const;
	void CalculateForces(const ParticleState& state,
											 std::vector<float> pressures, 
											 std::vector<float> rhos, 
											 std::vector<glm::vec3>& forces) const;
  glm::vec3 g_ = glm::vec3(0., -2., 0);
  float drag_constant_ = 0.3;
};
}  // namespace GLOO

#endif
