#ifndef MOCK_LIDAR_HPP
#define MOCK_LIDAR_HPP

#include "lidar_types/lidar_data.hpp"

class MockLiDAR {
 private:
  int dummy_value;

 public:
  virtual bool get(LiDARDataWrapper &data) = 0;
};

#endif  // MOCK_LIDAR_HPP