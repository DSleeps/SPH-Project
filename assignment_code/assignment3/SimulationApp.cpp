#include "SimulationApp.hpp"

#include "ForwardEulerIntegrator.hpp"
#include "TrapezoidIntegrator.hpp"
#include "RK4Integrator.hpp"
#include "ParticleSystemNode.hpp"
#include "SimpleCircleSystem.hpp"
#include "PendulumSystem.hpp"
#include "WaterSystem.hpp"
#include "IntegratorFactory.hpp"

#include "glm/gtx/string_cast.hpp"

#include "gloo/shaders/PhongShader.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/CameraComponent.hpp"
#include "gloo/components/LightComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/MeshLoader.hpp"
#include "gloo/lights/PointLight.hpp"
#include "gloo/lights/AmbientLight.hpp"
#include "gloo/cameras/ArcBallCameraNode.hpp"
#include "gloo/debug/AxisNode.hpp"


namespace GLOO {
SimulationApp::SimulationApp(const std::string& app_name,
                             glm::ivec2 window_size,
                             IntegratorType integrator_type,
                             float integration_step)
    : Application(app_name, window_size),
      integrator_type_(integrator_type),
      integration_step_(integration_step) {
  // TODO: remove the following two lines and use integrator type and step to
  // create integrators; the lines below exist only to suppress compiler
  // warnings.
}

int IndexOf(int x, int y, int particle_x) {
  return x + y * particle_x;
}

void ConstructCloth(PendulumSystem& base, ParticleState& state, float width, float height, int particle_x, int particle_y) {
  float mass = 1.f;

  // Start by initializing all of the particles in the right place
  for (int x = 0; x < particle_x; x++) {
    for (int y = 0; y < particle_y; y++) {
      glm::vec3 position = glm::vec3(width*x/(particle_x-1), -height*y/(particle_y-1), (height*y/(particle_y-1))/4);
      if (y == 0 && (x == 0 || x == (particle_x-1))) {
        base.AddParticle(state, mass, true, position);
      } else {
        base.AddParticle(state, mass, false, position);
      }
    }
  }
  
  // Add structural springs
  float struct_width_rest = width/(particle_x-1);
  float struct_height_rest = height/(particle_y-1);
  float struct_k = 1000.f;
  for (int x = 0; x < particle_x; x++) {
    for (int y = 0; y < particle_y; y++) {
      int p1 = IndexOf(x, y, particle_x);
      int p2 = IndexOf(x+1, y, particle_x);
      int p3 = IndexOf(x, y+1, particle_x);
      if (x != (particle_x-1)) {
        base.AddSpring(p1, p2, struct_width_rest, struct_k);
      }
      if (y != (particle_y-1)) {
        base.AddSpring(p1, p3, struct_height_rest, struct_k);
      }
    }
  }
  
  // Add shear springs
  float shear_rest = pow(pow(width/(particle_x-1),2) + pow(height/(particle_y-1),2), 0.5f);
  float shear_k = 1000.f;
  for (int x = 0; x < particle_x; x++) {
    for (int y = 0; y < particle_y-1; y++) {
      int p1 = IndexOf(x, y, particle_x);
      int p2 = IndexOf(x+1, y+1, particle_x);
      int p3 = IndexOf(x-1, y+1, particle_x);
      if (x != (particle_x - 1)) {
        base.AddSpring(p1, p2, shear_rest, shear_k);
      }
      if (x != 0) {
        base.AddSpring(p1, p3, shear_rest, shear_k);
      }
    }
  }

  // Add flex springs
  float flex_width_rest = 2 * width/(particle_x-1);
  float flex_height_rest = 2 * height/(particle_y-1);
  float flex_k = 1000.f;
  for (int x = 0; x < particle_x; x++) {
    for (int y = 0; y < particle_y; y++) {
      int p1 = IndexOf(x, y, particle_x);
      int p2 = IndexOf(x+2, y, particle_x);
      int p3 = IndexOf(x, y+2, particle_x);
      if (x < (particle_x - 2)) {
        base.AddSpring(p1, p2, flex_width_rest, flex_k);
      }
      if (y < (particle_y - 2)) {
        base.AddSpring(p1, p3, flex_height_rest, flex_k);
      }
    }
  }
}

void SimulationApp::SetupScene() {
  SceneNode& root = scene_->GetRootNode();

  auto camera_node = make_unique<ArcBallCameraNode>(45.f, 0.75f, 5.0f);
  scene_->ActivateCamera(camera_node->GetComponentPtr<CameraComponent>());
  root.AddChild(std::move(camera_node));

  root.AddChild(make_unique<AxisNode>('A'));

  auto ambient_light = std::make_shared<AmbientLight>();
  ambient_light->SetAmbientColor(glm::vec3(0.2f));
  root.CreateComponent<LightComponent>(ambient_light);

  auto point_light = std::make_shared<PointLight>();
  point_light->SetDiffuseColor(glm::vec3(0.8f, 0.8f, 0.8f));
  point_light->SetSpecularColor(glm::vec3(1.0f, 1.0f, 1.0f));
  point_light->SetAttenuation(glm::vec3(1.0f, 0.09f, 0.032f));
  auto point_light_node = make_unique<SceneNode>();
  point_light_node->CreateComponent<LightComponent>(point_light);
  point_light_node->GetTransform().SetPosition(glm::vec3(0.0f, 2.0f, 4.f));
  root.AddChild(std::move(point_light_node));
 	
	auto integrator = 
   IntegratorFactory::CreateIntegrator<WaterSystem, ParticleState>(integrator_type_);
  WaterSystem base;
  ParticleState state;
  
  int particle_number = 100;
	for (int i = 0; i < particle_number; i++) {
		float r_x = (((float) rand()/RAND_MAX) - 0.5f) * 2.f;
		float r_y = (((float) rand()/RAND_MAX) - 0.5f) * 2.f;
		float r_z = (((float) rand()/RAND_MAX) - 0.5f) * 2.f;
		
		base.AddParticle(state, glm::vec3(r_x, r_y, r_z), glm::vec3(r_x, r_y, r_z));
	}

  auto particle_node = 
             make_unique<ParticleSystemNode<WaterSystem>>
                        (std::move(integrator), base, state, integration_step_);
  particle_node->GetTransform().SetPosition(glm::vec3(0.f, 0.f, 0.f));
  root.AddChild(std::move(particle_node));
}

}  // namespace GLOO
