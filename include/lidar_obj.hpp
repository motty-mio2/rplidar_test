#ifndef LIDAR_TYPE_HPP_
#define LIDAR_TYPE_HPP_

#include <map>
#include <string>

class LiDAR_DATA {
private:
  std::map<int, float> data;

public:
  LiDAR_DATA();

  void insert(int index, float value);
  void to_string(std::string &out);
  void clear();
};

#endif // LIDAR_TYPE_HPP_