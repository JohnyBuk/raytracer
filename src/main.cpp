#include <chrono>
#include <iostream>
#include <map>

#include "engine.hpp"
#include "raytracer.hpp"

int main() {
  raytracer::Raytracer raytracer(1000, float(16) / 9, 10);

  std::unordered_map<std::string, std::unique_ptr<raytracer::Engine>> engines;
  engines["bullet"] = std::make_unique<raytracer::BulletEngine>();
  engines["physx"] = std::make_unique<raytracer::PhysxEngine>();

  raytracer::DiffuseMaterial diff_yellow({0.8, 0.8, 0.0});
  raytracer::DiffuseMaterial diff_red({0.8, 0.0, 0.0});
  raytracer::DiffuseMaterial diff_purple({0.8, 0.0, 0.8});
  raytracer::DiffuseMaterial diff_floor({0.0, 0.0, 1.0});
  raytracer::MetalMaterial mirror({0.8, 0.8, 0.8});
  raytracer::MetalMaterial metal({0.8, 0.8, 0.8}, 0.15);

  const int samples_per_pixel = 100;
  const int recursion_depth = 50;

  for (auto& [name, engine] : engines) {
    engine->create_sphere({100, -5, 4}, 5, &mirror);
    engine->create_sphere({98, 5, 4}, 4, &diff_red);
    engine->create_sphere({85, 7, 1}, 2, &mirror);
    engine->create_sphere({80, 3, 1}, 1.5f, &diff_purple);
    engine->create_sphere({90, 16, 4}, 6, &metal);
    engine->create_sphere({200, 0, -997}, 1000, &diff_floor);

    std::cout << name << " engine is running" << std::endl;
    std::string save_path = "image_" + name + ".png";

    const auto begin = std::chrono::steady_clock::now();
    const cv::Mat image = raytracer.run(*engine, samples_per_pixel, recursion_depth);
    const auto end = std::chrono::steady_clock::now();

    cv::imwrite(save_path, image);
    std::cout << std::endl << save_path << " saved\n";
    std::cout << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " seconds elapsed\n\n";
  }
  return 0;
}