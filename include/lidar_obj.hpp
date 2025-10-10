#ifndef LIDAR_TYPE_HPP_
#define LIDAR_TYPE_HPP_

#include <map>
#include <string>

using lidar_data_t = std::map<float, float>;

struct metadata_t {
  int x;
  int y;
  int min_rad;
  int max_rad;
  int max_dist;
};

class LiDAR_DATA {
private:
  lidar_data_t data;

public:
  void insert(float degree, float value);
  void to_string(std::string &out);
  void clear();
  lidar_data_t get();
};

#endif // LIDAR_TYPE_HPP_