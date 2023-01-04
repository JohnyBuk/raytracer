#pragma once

#include <opencv2/opencv.hpp>
#include <optional>

namespace raytracer {

  class Material {
  public:
    Material(const cv::Vec3f &col) : color(col) {}
    virtual std::optional<cv::Vec3f> get_reflect_dir(const cv::Vec3f &normal, const cv::Vec3f &incomming) const = 0;
    const cv::Vec3f color;
  };

  class DiffuseMaterial : public Material {
  public:
    DiffuseMaterial(cv::Vec3f color) : Material::Material(color) {}

    std::optional<cv::Vec3f> get_reflect_dir(const cv::Vec3f &normal,
                                             [[maybe_unused]] const cv::Vec3f &incomming) const override {
      cv::Vec3f reflected;
      cv::randu(reflected, -1, 1);
      reflected /= cv::norm(reflected);
      if (normal.dot(reflected) > 0) return reflected;
      return -reflected;
    }
  };

  class MetalMaterial : public Material {
  public:
    MetalMaterial(cv::Vec3f color, const float roughness = 0) : Material::Material(color), _roughness(roughness) {}

    std::optional<cv::Vec3f> get_reflect_dir(const cv::Vec3f &incomming, const cv::Vec3f &normal) const override {
      cv::Vec3f reflected = incomming - 2 * incomming.dot(normal) * normal;
      if (_roughness > 0) {
        cv::Vec3f fuzzy;
        cv::randu(fuzzy, -_roughness, _roughness);
        reflected += fuzzy;
      }
      reflected /= cv::norm(reflected);
      if (normal.dot(reflected) > 0) return reflected;
      return {};
    }

  private:
    const float _roughness;
  };

}  // namespace raytracer