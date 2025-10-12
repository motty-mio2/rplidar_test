#ifndef RPLIDAR_WRAPPER_HPP
#define RPLIDAR_WRAPPER_HPP

#include <rplidar.h>

#include <limits>
#include <memory>
#include <random>

#include "lidar_data.hpp"
#include "mock_lidar.hpp"

class RplidarWrapper : MockLiDAR {
 private:
  std::unique_ptr<sl::IChannel> communication_channel;
  std::unique_ptr<sl::Result<sl::IChannel *>> channel;
  //    =    createSerialPortChannel(FLAGS_d, 115200);
  std::unique_ptr<sl::ILidarDriver *> lidar;
  float max_dist;

 public:
  RplidarWrapper(std::string device, float max_dist);
  bool get(LiDARDataWrapper &data);
  ~RplidarWrapper();
};

#endif  // RPLIDAR_WRAPPER_HPP