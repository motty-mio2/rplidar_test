#ifndef RANDOM_LIDAR_HPP
#define RANDOM_LIDAR_HPP

#include <memory>
#include <random>

#include "mock_lidar.hpp"

class RandomLiDAR : MockLiDAR {
 private:
  std::unique_ptr<std::mt19937> random_engine;
  std::unique_ptr<std::uniform_int_distribution<int>> random_distance;
  std::unique_ptr<std::uniform_real_distribution<float>> random_rate;
  std::unique_ptr<std::uniform_real_distribution<float>> random_skip;

 public:
  RandomLiDAR() {
    random_engine = std::make_unique<std::mt19937>(std::random_device{}());
    random_distance =
        std::make_unique<std::uniform_int_distribution<int>>(10, 1000);
    random_skip =
        std::make_unique<std::uniform_real_distribution<float>>(0.0, 1.0);
    random_rate =
        std::make_unique<std::uniform_real_distribution<float>>(0.8, 1.2);
  };

  ~RandomLiDAR();
};

#endif  // RANDOM_LIDAR_HPP