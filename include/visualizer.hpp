#ifndef VISUALIZER_HPP_
#define VISUALIZER_HPP_

#include <limits>
#include <opencv2/opencv.hpp>
#include <vector>

#include "generate_color.hpp"
#include "lidar_types/lidar_data.hpp"

constexpr int IMG_SIZE = 600;

cv::Mat visualize(LiDARData data, std::string window_name, uint32_t num = 4,
                  double max_dist = 1000.0) {
  float angle_step = 360.0f / num;

  std::vector<double> near;

  for (auto i = 0; i < num; ++i) {
    near.push_back(IMG_SIZE / 2.0f);
  }

  cv::Mat img = cv::Mat::zeros(IMG_SIZE, IMG_SIZE, CV_8UC3);

  for (auto &[degree, distance] : data) {
    if (distance == 0.0f || distance > max_dist) {
      continue;
    }

    int index = static_cast<int>(degree / angle_step);

    near[index] =
        std::min(near[index], distance * (IMG_SIZE / 2.0f) / max_dist);
  }

  for (auto i = 0; i < num; ++i) {
    cv::ellipse(img, cv::Point2d(IMG_SIZE / 2, IMG_SIZE / 2),
                cv::Size(near[i], near[i]), angle_step, angle_step * (i - 1),
                angle_step * i, generate_color(i, num), 2);
  }

  // cv::imshow(window_name, img);
  return img;
}

#endif  // VISUALIZER_HPP_