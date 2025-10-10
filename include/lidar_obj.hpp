#ifndef LIDAR_TYPE_HPP_
#define LIDAR_TYPE_HPP_

#include <cstdint>
#include <cstring>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

struct metadata_t {
  int x;
  int y;
  int min_rad;
  int max_rad;
  int max_dist;
};

using sensor_data_t = std::map<float, float>;

struct lidar_data {
  sensor_data_t data;
};

class LiDAR_DATA_WRAPPER {
 private:
  sensor_data_t data;

 public:
  LiDAR_DATA_WRAPPER() {};
  LiDAR_DATA_WRAPPER(const sensor_data_t &new_data) : data(new_data) {};
  LiDAR_DATA_WRAPPER(const std::vector<uint8_t> bin_data);

  void insert(float degree, float value);

  void clear();
  const sensor_data_t get();
  void get(sensor_data_t &s);
  void set(sensor_data_t new_data) { data = new_data; };
  std::vector<uint8_t> dump();
};

#endif  // LIDAR_TYPE_HPP_