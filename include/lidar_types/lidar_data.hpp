#ifndef LIDAR_DATA_HPP_
#define LIDAR_DATA_HPP_

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

using LiDARData = std::map<float, float>;

class LiDARDataWrapper {
 private:
 public:
  LiDARData data;
  LiDARDataWrapper(){};
  LiDARDataWrapper(const LiDARData &new_data) : data(new_data){};
  LiDARDataWrapper(const std::vector<uint8_t> bin_data);

  void insert(float degree, float value);

  void clear();
  const LiDARData get();
  void get(LiDARData &s);
  void set(LiDARData new_data) { data = new_data; };
  std::vector<uint8_t> dump();
};

#endif  // LIDAR_DATA_HPP_