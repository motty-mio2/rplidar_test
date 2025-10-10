#ifndef VISUALIZER_HPP_
#define VISUALIZER_HPP_

#include <gflags/gflags.h>

#include <limits>
#include <opencv2/opencv.hpp>
#include <vector>

#include "generate_color.hpp"
#include "lidar_obj.hpp"

DECLARE_string(d);
DECLARE_uint32(num);
DECLARE_double(max_dist);

constexpr int IMG_SIZE = 600;

void visualize(LiDARData data) {
  float angle_step = 360.0f / FLAGS_num;

  std::vector<double> near;

  for (auto i = 0; i < FLAGS_num; ++i) {
    near.push_back(IMG_SIZE / 2.0f);
  }

  cv::Mat img = cv::Mat::zeros(IMG_SIZE, IMG_SIZE, CV_8UC3);

  for (auto &[degree, distance] : data) {
    if (distance == 0.0f || distance > FLAGS_max_dist) {
      continue;
    }

    int index = static_cast<int>(degree / angle_step);

    near[index] =
        std::min(near[index], distance * (IMG_SIZE / 2.0f) / FLAGS_max_dist);
  }

  for (auto i = 0; i < FLAGS_num; ++i) {
    cv::ellipse(img, cv::Point2d(IMG_SIZE / 2, IMG_SIZE / 2),
                cv::Size(near[i], near[i]), angle_step, angle_step * (i - 1),
                angle_step * i, generate_color(i, FLAGS_num), 2);
  }

  cv::imshow("Test Window", img);

  if (cv::waitKey(1) == 'q') {
    std::exit(0);
  }
}

#endif  // VISUALIZER_HPP_