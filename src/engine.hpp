#pragma once

#include <PxPhysicsAPI.h>
#include <btBulletCollisionCommon.h>

#include <memory>
#include <thread>
#include <vector>

#include "material.hpp"

#define USE_PHYSX_GPU 0

#define COORDS(v) \
  { v[0], v[1], v[2] }

namespace raytracer {

  class Engine {
  public:
    virtual ~Engine() = default;
    virtual void create_sphere(const cv::Vec3f& position, const float radius, Material* material) = 0;
    virtual cv::Vec3f raytrace(const cv::Vec3f& origin, const cv::Vec3f& direction, int depth) const = 0;

  protected:
    const float _max_ray_dist = 1e3f;
    const cv::Vec3f _background = {1, 1, 0.5};  // background color
  };

  class PhysxEngine : public Engine {
  private:
    struct PxDeleter {
      template <typename T>
      void operator()(T* t) const {
        if (t) t->release();
      }
    };

    template <typename T>
    using PxUniquePtr = std::unique_ptr<T, PxDeleter>;

    template <typename T>
    PxUniquePtr<T> _make_unique(T* t) {
      return PxUniquePtr<T>(t);
    }

    template <typename T>
    std::shared_ptr<T> _make_shared(T* t) {
      return std::shared_ptr<T>(t, PxDeleter());
    }

  public:
    PhysxEngine(const std::size_t threads = std::thread::hardware_concurrency()) {
      _foundation = _make_unique(PxCreateFoundation(PX_PHYSICS_VERSION, _allocator, _error_callback));
      _dispatcher = _make_unique(physx::PxDefaultCpuDispatcherCreate(threads));
      _physics = _make_unique(PxCreatePhysics(PX_PHYSICS_VERSION, *_foundation, physx::PxTolerancesScale()));
      _material = _make_unique(_physics->createMaterial(0.5f, 0.5f, 0.6f));
      _world_desc = std::make_unique<physx::PxSceneDesc>(_physics->getTolerancesScale());

      _world_desc->gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
      _world_desc->cpuDispatcher = _dispatcher.get();
      _world_desc->filterShader = physx::PxDefaultSimulationFilterShader;

#if USE_PHYSX_GPU
      _cuda_manager = _make_unique(PxCreateCudaContextManager(*_foundation, _world_cuda_desc, PxGetProfilerCallback()));
      _world_desc->cudaContextManager = _cuda_manager.get();
      _world_desc->filterShader = physx::PxDefaultSimulationFilterShader;
      _world_desc->flags |= physx::PxSceneFlag::eENABLE_GPU_DYNAMICS;
      _world_desc->broadPhaseType = physx::PxBroadPhaseType::eGPU;
#endif
      _world = _make_unique(_physics->createScene(*_world_desc));
    }

    void create_sphere(const cv::Vec3f& pos, const float radius, Material* material) override {
      auto shape = _make_shared(_physics->createShape(physx::PxSphereGeometry(radius), *_material));
      auto body = _make_shared(_physics->createRigidStatic(physx::PxTransform(physx::PxVec3(COORDS(pos)))));

      body->attachShape(*shape);
      body->userData = material;
      _world->addActor(*body);

      _shapes.push_back(shape);
      _bodies.push_back(body);
    }

    cv::Vec3f raytrace(const cv::Vec3f& origin, const cv::Vec3f& direction, int depth) const override {
      const auto color = _raytrace(COORDS(origin), COORDS(direction), depth);
      return COORDS(color);
    }

  private:
    physx::PxVec3 _raytrace(const physx::PxVec3& origin, const physx::PxVec3& dir, int depth) const {
      physx::PxRaycastBuffer hit;

      // no light source hitted
      if (depth <= 0) return {0, 0, 0};

      if (_world->raycast(origin, dir, _max_ray_dist, hit, _hit_flags)) {
        const auto material = static_cast<Material*>(hit.block.actor->userData);

        // if (hit.block.hadInitialOverlap()) std::cout << "hadInitialOverlap" << std::endl;
        auto reflected_dir = material->get_reflect_dir(COORDS(dir), COORDS(hit.block.normal));

        if (reflected_dir.has_value()) {
          auto color = _raytrace(hit.block.position + hit.block.normal * 0.01f,  // avoid self-hit
                                 COORDS(reflected_dir.value()), depth - 1);
          return {material->color[0] * color[0], material->color[1] * color[1], material->color[2] * color[2]};
        }
        return {0, 0, 0};
      }

      // sky hitted
      return COORDS(_background);
    }

    physx::PxHitFlags _hit_flags = physx::PxHitFlag::ePOSITION | physx::PxHitFlag::eNORMAL;
    physx::PxDefaultAllocator _allocator;
    physx::PxDefaultErrorCallback _error_callback;
    physx::PxCudaContextManagerDesc _world_cuda_desc;
    std::unique_ptr<physx::PxSceneDesc> _world_desc;
    PxUniquePtr<physx::PxFoundation> _foundation;
    PxUniquePtr<physx::PxPhysics> _physics;
    PxUniquePtr<physx::PxDefaultCpuDispatcher> _dispatcher;
    PxUniquePtr<physx::PxScene> _world;
    PxUniquePtr<physx::PxMaterial> _material;
    PxUniquePtr<physx::PxCudaContextManager> _cuda_manager;

    std::vector<std::shared_ptr<physx::PxShape>> _shapes;
    std::vector<std::shared_ptr<physx::PxRigidStatic>> _bodies;
  };

  class BulletEngine : public Engine {
  public:
    BulletEngine() {
      _config = std::make_unique<btDefaultCollisionConfiguration>();
      _dispatcher = std::make_unique<btCollisionDispatcher>(_config.get());
      _broadphase = std::make_unique<btDbvtBroadphase>();
      _world = std::make_unique<btCollisionWorld>(_dispatcher.get(), _broadphase.get(), _config.get());
    }

    virtual ~BulletEngine() {
      for (const auto& body : _bodies) _world->removeCollisionObject(body.get());
    }

    void create_sphere(const cv::Vec3f& pos, const float radius, Material* material) override {
      auto shape = std::make_shared<btSphereShape>(radius);
      auto body = std::make_shared<btCollisionObject>();

      btTransform pose;
      pose.setIdentity();
      pose.setOrigin(COORDS(pos));

      body->setCollisionShape(shape.get());
      body->setWorldTransform(pose);
      body->setUserPointer(material);
      _world->addCollisionObject(body.get());

      _shapes.push_back(shape);
      _bodies.push_back(body);
    }

    cv::Vec3f raytrace(const cv::Vec3f& origin, const cv::Vec3f& direction, int depth) const override {
      const auto color = _raytrace(COORDS(origin), COORDS(direction), depth);
      return COORDS(color);
    }

  private:
    btVector3 _raytrace(const btVector3& origin, const btVector3& dir, int depth) const {
      btVector3 target = origin + dir * _max_ray_dist;
      btCollisionWorld::ClosestRayResultCallback hit(origin, target);

      // no light source hitted
      if (depth <= 0) return {0, 0, 0};
      _world->rayTest(origin, target, hit);

      if (hit.hasHit()) {
        const auto material = static_cast<Material*>(hit.m_collisionObject->getUserPointer());
        auto reflected_dir = material->get_reflect_dir(COORDS(dir), COORDS(hit.m_hitNormalWorld));

        if (reflected_dir.has_value()) {
          auto color = _raytrace(hit.m_hitPointWorld + hit.m_hitNormalWorld * 0.01f,  // avoid self-hit
                                 COORDS(reflected_dir.value()), depth - 1);
          return {material->color[0] * color[0], material->color[1] * color[1], material->color[2] * color[2]};
        }
        return {0, 0, 0};
      }

      // sky hitted
      return COORDS(_background);
    }

    std::unique_ptr<btDefaultCollisionConfiguration> _config;
    std::unique_ptr<btCollisionDispatcher> _dispatcher;
    std::unique_ptr<btDbvtBroadphase> _broadphase;
    std::unique_ptr<btCollisionWorld> _world;

    std::vector<std::shared_ptr<btCollisionShape>> _shapes;
    std::vector<std::shared_ptr<btCollisionObject>> _bodies;
  };

}  // namespace raytracer
