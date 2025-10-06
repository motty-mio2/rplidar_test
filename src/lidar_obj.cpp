
#include <lidar_obj.hpp>

void LiDAR_DATA::insert(float degree, float value) { //
  data[degree] = value;
}

void LiDAR_DATA::to_string(std::string &out) {
  out = "{";
  for (auto &d : data) {
    out += std::to_string(d.first) + ": " + std::to_string(d.second) + ", ";
  }
  out += "}";
}

void LiDAR_DATA::clear() { //
  data.clear();
}

lidar_data_t LiDAR_DATA::get() { //
  return data;
}