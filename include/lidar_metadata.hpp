#ifndef LIDAR_METADATA_HPP_
#define LIDAR_METADATA_HPP_

#include <cstdint>
#include <cstring>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

class LidarMetadata {
 public:
  int x;
  int y;
  int min_rad;
  int max_rad;
  int max_dist;

  LidarMetadata(const std::vector<uint8_t> bin_data);
  std::vector<uint8_t> dump();
};

#endif  // LIDAR_METADATA_HPP_