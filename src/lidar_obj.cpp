
#include <lidar_obj.hpp>

void LiDAR_DATA::insert(int index, float value) { data[index] = value; }

void LiDAR_DATA::to_string(std::string &out) {
  out = "{";
  for (auto &d : data) {
    out += std::to_string(d.first) + ": " + std::to_string(d.second) + ", ";
  }
  out += "}";
}

void LiDAR_DATA::clear() { data.clear(); }