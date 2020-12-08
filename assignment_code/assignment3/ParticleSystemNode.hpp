#ifndef PARTICLE_SYSTEM_NODE_H_
#define PARTICLE_SYSTEM_NODE_H_

#include "gloo/SceneNode.hpp"
#include "gloo/debug/PrimitiveFactory.hpp"
#include "gloo/components/RenderingComponent.hpp"
#include "gloo/components/ShadingComponent.hpp"
#include "gloo/components/MaterialComponent.hpp"
#include "gloo/MeshLoader.hpp"
#include "gloo/shaders/PhongShader.hpp"
#include "gloo/InputManager.hpp"

#include "IntegratorBase.hpp"
#include "ParticleSystemBase.hpp"
#include "ParticleState.hpp"
#include "Grid.hpp"
#include "stb_image.h"
#include "stb_image_write.h"

namespace GLOO {

typedef std::bitset<8> BYTE;

template <class TSystem>
class ParticleSystemNode : public SceneNode {
 public:
  ParticleSystemNode() {}
  ParticleSystemNode(std::unique_ptr<IntegratorBase<TSystem, ParticleState>> integrator,
                     TSystem base,
                     ParticleState state,
                     float dt);
  void Update(double delta_time) override;

 private:
  void InitializeParticles();
  NormalArray CalculateNormals();
  void DrawWater();

  std::unique_ptr<IntegratorBase<TSystem, ParticleState>> integrator_;
  TSystem base_;
  ParticleState state_;
  std::vector<glm::vec3> original_pos_;
  std::vector<glm::vec3> original_vel_;

  std::shared_ptr<VertexObject> sphere_mesh_;
  std::shared_ptr<PhongShader> shader_;
  std::shared_ptr<VertexObject> vertex_obj_;
  std::shared_ptr<Material> material_comp_;

  std::vector<SceneNode*> particles;

	Grid grid_;

  float dt_;
	float fps_ = 1.f/60.f;
	float fps_pos_ = 0.f;

  float cur_time_;
  int count = 0;

	float box_width_ = 2.f; //TODO: This is also hardcoded in RK4 integrator and WaterSystem
	float box_height_ = 2.f;

	int frame_ = 0;

	int width_;
	int height_;
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

  Material default_material(glm::vec3(0.0f, 0.0f, 1.f),
                                     glm::vec3(0.0f, 0.0f, 1.0f),
                                     glm::vec3(0.4f, 0.4f, 0.4f), 20.0f);
  material_comp_ = std::make_shared<Material>(default_material);
  original_pos_ = state.positions;
  original_vel_ = state.velocities;

	grid_ = Grid(glm::vec3(-box_width_/2.f, -box_height_/2.f, -box_width_/2.f),
							 glm::vec3(box_width_/2.f, box_height_/2.f, box_width_/2.f));

  cur_time_ = 0.;
  dt_ = dt;

	GLint dims[4] = {0};
	glGetIntegerv(GL_VIEWPORT, dims);
	width_ = (int)dims[2];
	height_ = (int)dims[3];

  InitializeParticles();
	DrawWater();
  CreateComponent<ShadingComponent>(shader_);
  CreateComponent<RenderingComponent>(vertex_obj_);
  CreateComponent<MaterialComponent>(material_comp_);
}

template<class TSystem>
void ParticleSystemNode<TSystem>::Update(double delta_time) {
  // Now just take one step everytime
	state_ = integrator_->Integrate(base_, state_, cur_time_, dt_);
 	DrawWater();

	if (fps_pos_ > fps_) {
		// Take a screenshot
		std::string filename = "SimScreenshots/Sim" + std::to_string(frame_) + ".bmp";
		frame_ += 1;

		BYTE* pixels = new BYTE[3 * width_ * height_];
		glReadPixels(0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, pixels);
		stbi_write_bmp(filename.c_str(), width_, height_, 3, pixels);

		delete [] pixels;
		fps_pos_ = fps_pos_ - fps_;
	} else {
		fps_pos_ += dt_;
	}

  if (count == 10){
    auto particle_node = make_unique<SceneNode>();
    particle_node->CreateComponent<ShadingComponent>(shader_);
    // particle_node->CreateComponent<RenderingComponent>(sphere_mesh_);
    particle_node->CreateComponent<MaterialComponent>(material_comp_);
    particles.push_back(particle_node.get());
    AddChild(std::move(particle_node));
    float r_x = (((float) rand()/RAND_MAX) - 0.5f) * 0.25f;
		float r_z = (((float) rand()/RAND_MAX) - 0.5f) * 0.25f;
    state_.positions.push_back(glm::vec3(r_x,.9,r_z));
    state_.velocities.push_back(glm::vec3(0,-2.5,0));

  	for (int i = 0; i < state_.positions.size(); i++) {
      SceneNode* particle = particles[i];
      particle->GetTransform().SetPosition(state_.positions[i]);
    }
    count = 0;

    if (state_.positions.size() % 250 == 0){
      std::cout << state_.velocities.size() << '\n';
    }

  }
  count ++;



}

template<class TSystem>
void ParticleSystemNode<TSystem>::InitializeParticles() {
  for (int i = 0; i < state_.positions.size(); i++) {
    auto particle_node = make_unique<SceneNode>();
    // particle_node->CreateComponent<ShadingComponent>(shader_);
    // particle_node->CreateComponent<RenderingComponent>(sphere_mesh_);
    // particle_node->CreateComponent<MaterialComponent>(material_comp_);
    particles.push_back(particle_node.get());
    AddChild(std::move(particle_node));
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
void ParticleSystemNode<TSystem>::DrawWater() {
  PositionArray p_array;
	p_array.clear();
	IndexArray i_array;
	i_array.clear();
	NormalArray n_array;
	n_array.clear();

	grid_.CalculateBlobs(state_.positions, p_array, i_array, n_array);

	auto positions = make_unique<PositionArray>(p_array);
  auto indices = make_unique<IndexArray>(i_array);

	vertex_obj_->UpdatePositions(std::move(positions));
  vertex_obj_->UpdateIndices(std::move(indices));
	if (n_array.size() > 0) {
		auto normals = make_unique<NormalArray>(n_array);
  	vertex_obj_->UpdateNormals(std::move(normals));
	} else {
  	vertex_obj_->UpdateNormals(std::move(make_unique<NormalArray>(CalculateNormals())));
	}
}

} // namespace GLOO

#endif
