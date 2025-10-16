#ifndef MOCK_LIDAR_HPP
#define MOCK_LIDAR_HPP

#include "lidar_types/lidar_data.hpp"

class MockLiDAR {
 public:
  virtual bool get(LiDARDataWrapper &data) = 0;
  virtual ~MockLiDAR() = default;
};

#endif  // MOCK_LIDAR_HPP