#ifndef RANDOM_LIDAR_HPP
#define RANDOM_LIDAR_HPP

#include <memory>
#include <random>
#include <thread>

#include "mock_lidar.hpp"

class RandomLiDAR : MockLiDAR {
 private:
  std::unique_ptr<std::mt19937> random_engine;
  std::unique_ptr<std::uniform_int_distribution<int>> random_distance;
  std::unique_ptr<std::uniform_real_distribution<float>> random_rate;
  std::unique_ptr<std::uniform_real_distribution<float>> random_skip;

 public:
  inline RandomLiDAR() {
    random_engine = std::make_unique<std::mt19937>(std::random_device{}());
    random_distance =
        std::make_unique<std::uniform_int_distribution<int>>(10, 1000);
    random_skip =
        std::make_unique<std::uniform_real_distribution<float>>(0.0, 1.0);
    random_rate =
        std::make_unique<std::uniform_real_distribution<float>>(0.8, 1.2);
  };

  inline bool get(LiDARDataWrapper &data) override {
    float rate = (*random_rate)(*random_engine);
    for (int degree = 0; degree < 360; degree++) {
      if ((*random_skip)(*random_engine) > rate) {
        continue;
      }
      int dist = (*random_distance)(*random_engine);
      data.insert(degree, dist);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
  }

  inline ~RandomLiDAR(){};
};

#endif  // RANDOM_LIDAR_HPP