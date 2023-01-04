#pragma once

#include <opencv2/opencv.hpp>

#include "engine.hpp"

namespace raytracer {

  class Raytracer {
  public:
    Raytracer(const int width, const float aspect_ratio, const float focal_length) :
        _image_width(width),
        _image_height(width / aspect_ratio),
        _aspect_ratio(aspect_ratio),
        _focal_length(focal_length),
        _origin{0, 5, 0} {}

    cv::Mat run(const Engine& scene, const int samples_per_pixel, const int recursion_depth) {
      CV_Assert(samples_per_pixel > 0);
      CV_Assert(recursion_depth > 0);
      CV_Assert(_image_height > 0);
      CV_Assert(_image_width > 0);

      cv::Mat image = cv::Mat::zeros(_image_height, _image_width, CV_32FC3);

      for (int i = 0; i < _image_height; i++) {
        std::cerr << "\r" << int(float(i) / _image_height * 100) + 1 << " %" << std::flush;
        for (int j = 0; j < _image_width; j++) {
          for (int k = 0; k < samples_per_pixel; k++) {
            // generate two random floats in (-1,1)
            cv::Vec2f sample;
            cv::randu(sample, -1, 1);

            const float u = 2 * ((j + sample[0]) / _image_width - 0.5f) * _aspect_ratio;
            const float v = 2 * ((i + sample[1]) / _image_height - 0.5f);

            cv::Vec3f ray_direction = _get_pixel_direction(u, v);
            image.at<cv::Vec3f>(i, j) += scene.raytrace(_origin, ray_direction, recursion_depth);
          }
        }
      }

      cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
      // gamma-correction for gamma=2
      cv::sqrt(image / samples_per_pixel, image);
      return image * 255;
    }

  private:
    cv::Vec3f _get_pixel_direction(const float u, const float v) {
      cv::Vec3f dir = cv::Vec3f(10, u + 5, -v) - _origin;
      return dir / cv::norm(dir);
    }

    const int _image_width, _image_height;
    const float _aspect_ratio, _focal_length;
    const cv::Vec3f _origin;
  };

}  // namespace raytracer