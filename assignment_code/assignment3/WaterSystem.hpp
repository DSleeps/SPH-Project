#ifndef WATER_SYSTEM_H_
#define WATER_SYSTEM_H_

#include "ParticleState.hpp"
#include "ParticleSystemBase.hpp"
#include <set>

namespace GLOO {

 // solver parameters
const static float REST_DENS = 12.2f;//10.f; // rest density
const static float GAS_CONST = 20.f; // const for equation of state
const static float H = .16f; // kernel radius
const static float HSQ = H*H; // radius^2 for optimization
const static float MASS = 1.9f; // assume all particles have the same mass
const static float VISC = .5f;//2.50f; // viscosity constant

// smoothing kernels defined in Müller and their gradients
const static float POLY6 = 3.15f/(65.f*3.1415*pow(H, 9.f));
const static float SPIKY_GRAD = -.45f/(3.1415*pow(H, 6.f));
const static float VISC_LAP = .45f/(3.1415*pow(H, 6.f));
const static glm::vec3 GRAVITY = glm::vec3(0.f, -20.f, 0.f);

// const static float POLY6 = 315.f/(65.f*3.1415*pow(H, 9.f));
// const static float SPIKY_GRAD = -45.f/(3.1415*pow(H, 6.f));
// const static float VISC_LAP = 45.f/(3.1415*pow(H, 6.f));
class WaterSystem : public ParticleSystemBase {
 public:

	WaterSystem();
	virtual ~WaterSystem() {
  }

  ParticleState ComputeTimeDerivative(const ParticleState& state,
                                      float time) override;

  void AddParticle(ParticleState& state, glm::vec3 position, glm::vec3 velocity);
 private:
	void CalculatePressure(const ParticleState& state, std::vector<float>& pressures, std::vector<float>& rhos) const;
	void CalculateForces(const ParticleState& state,
											 std::vector<float> pressures,
											 std::vector<float> rhos,
											 std::vector<glm::vec3>& forces) const;

	// Gets the grid cell a point is in
	glm::vec3 GetGridCell(glm::vec3 p) const;

	// The width and height of the box
	float box_width_ = 2.f; //TODO: Hardcoded in RK4Integrator and ParticleSystemNode
	float box_height_ = 2.f;

	// A grid based data structure to improve the speed of pressure/density calculations
	std::vector<std::vector<std::vector<std::set<int>>>> grid_;

  // Must divide evenly into box_width_ and box_height_
	float grid_cell_width_ = 0.2f;
	float grid_cell_height_ = 0.2f;

	int grid_width_;
	int grid_height_;
};
}  // namespace GLOO

#endif
