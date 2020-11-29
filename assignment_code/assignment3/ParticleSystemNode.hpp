#ifndef PARTICLE_SYSTEM_NODE_H_
#define PARTICLE_SYSTEM_NODE_H_

#include "gloo/SceneNode.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/MeshLoader.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/InputManager.hpp"

#include "IntegratorBase.hpp"
#include "ParticleSystemBase.hpp"
#include "ParticleState.hpp"
#include "SimpleCircleSystem.hpp"

namespace GLOO {
template <class TSystem>
class ParticleSystemNode : public SceneNode {
 public:
  ParticleSystemNode() {}
  ParticleSystemNode(std::unique_ptr<IntegratorBase<TSystem, ParticleState>> integrator,
                     TSystem base,
                     ParticleState state,
                     float dt);
  ParticleSystemNode(std::unique_ptr<IntegratorBase<TSystem, ParticleState>> integrator,
                     TSystem base,
                     ParticleState state,
                     float dt, int particle_width, int particle_height);
  void Update(double delta_time) override;
  
 private:
  void InitializeParticles();
  NormalArray CalculateNormals();
  void DrawCloth();

  std::unique_ptr<IntegratorBase<TSystem, ParticleState>> integrator_;
  TSystem base_;
  ParticleState state_;
  std::vector<glm::vec3> original_pos_;
  std::vector<glm::vec3> original_vel_;

  std::shared_ptr<VertexObject> sphere_mesh_;
  std::shared_ptr<PhongShader> shader_;
  std::shared_ptr<VertexObject> vertex_obj_;

  std::vector<SceneNode*> particles;
  
  int particle_width_;
  int particle_height_;

  float dt_;
  float cur_time_;
  bool cloth_;
};

template<class TSystem>
ParticleSystemNode<TSystem>::ParticleSystemNode(
    std::unique_ptr<IntegratorBase<TSystem, ParticleState>> integrator,
    TSystem base,
    ParticleState state,
    float dt) : base_(base), state_(state) {
  integrator_ = std::move(integrator);
  
  sphere_mesh_ = PrimitiveFactory::CreateSphere(0.055f, 25, 25);
  shader_ = std::make_shared<PhongShader>();
  vertex_obj_ = std::make_shared<VertexObject>();
  
  original_pos_ = state.positions;
  original_vel_ = state.velocities;

  cur_time_ = 0.;
  dt_ = dt;
  cloth_ = false;

  InitializeParticles();
}

template<class TSystem>
ParticleSystemNode<TSystem>::ParticleSystemNode(
    std::unique_ptr<IntegratorBase<TSystem, ParticleState>> integrator,
    TSystem base,
    ParticleState state,
    float dt, int particle_width, int particle_height) : base_(base), state_(state) {
  integrator_ = std::move(integrator);
  
  sphere_mesh_ = PrimitiveFactory::CreateSphere(0.055f, 25, 25);
  shader_ = std::make_shared<PhongShader>();
  vertex_obj_ = std::make_shared<VertexObject>();

  original_pos_ = state.positions;
  original_vel_ = state.velocities;
  
  particle_width_ = particle_width;
  particle_height_ = particle_height;

  cur_time_ = 0.;
  dt_ = dt;
  cloth_ = true;

  InitializeParticles();
}

template<class TSystem>
void ParticleSystemNode<TSystem>::Update(double delta_time) {
  int iter_times = ceil(delta_time/dt_);
  for (int i = 0; i < iter_times; i++) {
    state_ = integrator_->Integrate(base_, state_, cur_time_, dt_);
  }
  for (int i = 0; i < state_.positions.size(); i++) {
    SceneNode* particle = particles[i];
    particle->GetTransform().SetPosition(state_.positions[i]);
  }

  if (cloth_) {
    DrawCloth();
    static bool prev_released = true;
    if (InputManager::GetInstance().IsKeyPressed('R')) {
      if (prev_released) {
        state_.positions = original_pos_;
        state_.velocities = original_vel_;
      }
      prev_released = false;
    } else if (InputManager::GetInstance().IsKeyReleased('R')) {
      prev_released = true;
    }
  }
  
}

template<class TSystem>
void ParticleSystemNode<TSystem>::InitializeParticles() {
  for (int i = 0; i < state_.positions.size(); i++) {
    auto particle_node = make_unique<SceneNode>();
    if (!cloth_) {
      particle_node->CreateComponent<ShadingComponent>(shader_);
      particle_node->CreateComponent<RenderingComponent>(sphere_mesh_);
    }
    particles.push_back(particle_node.get());
    AddChild(std::move(particle_node));
  }
  if (cloth_) {
    DrawCloth();
    CreateComponent<ShadingComponent>(shader_);
    CreateComponent<RenderingComponent>(vertex_obj_);
  }
}

template<class TSystem>
NormalArray ParticleSystemNode<TSystem>::CalculateNormals() {
  PositionArray positions = vertex_obj_->GetPositions();
  IndexArray indices = vertex_obj_->GetIndices();

  // Populate the array with empty vectors
  NormalArray normals;
  for (int i = 0; i < positions.size(); i++) {
    normals.push_back(glm::vec3());
  }
  
  for (int i = 0; i < indices.size(); i += 3) {
    auto p1 = positions[indices[i]];
    auto p2 = positions[indices[i+1]];
    auto p3 = positions[indices[i+2]];

    glm::vec3 v1 = p1 - p2;
    glm::vec3 v2 = p1 - p3;
    glm::vec3 normal = glm::normalize(glm::cross(v1, v2));

    normals[indices[i]] = glm::normalize(normal + normals[indices[i]]);
    normals[indices[i+1]] = glm::normalize(normal + normals[indices[i+1]]);
    normals[indices[i+2]] = glm::normalize(normal + normals[indices[i+2]]);
  }
  return normals;
}

template<class TSystem>
void ParticleSystemNode<TSystem>::DrawCloth() {
  auto positions = make_unique<PositionArray>();
  auto indices = make_unique<IndexArray>();

  for (int x = 0; x < particle_width_; x++) {
    for (int y = 0; y < particle_height_; y++) {
      // Update the position and the normals
      SceneNode* particle = particles[x + particle_width_*y];
      positions->push_back(particle->GetTransform().GetPosition());
      
      //Update the indices
      if (x != particle_width_-1 && y != particle_height_-1) {
        indices->push_back(y*(particle_width_) + x);
        indices->push_back((y+1)*(particle_width_) + x);
        indices->push_back(y*(particle_width_) + x+1);
        indices->push_back(y*(particle_width_) + x+1);
        indices->push_back((y+1)*(particle_width_) + x);
        indices->push_back((y+1)*(particle_width_) + x+1);
      }
    }
  }
  vertex_obj_->UpdatePositions(std::move(positions));
  vertex_obj_->UpdateIndices(std::move(indices));
  vertex_obj_->UpdateNormals(std::move(make_unique<NormalArray>(CalculateNormals())));
}

} // namespace GLOO

#endif
