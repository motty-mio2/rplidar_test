
#include "rplidar_wrapper.hpp"

RplidarWrapper::RplidarWrapper(std::string device, float max_dist)
    : max_dist(max_dist) {
  channel = std::make_unique<sl::Result<sl::IChannel *>>(
      sl::createSerialPortChannel(device, 115200));

  lidar = std::make_unique<sl::ILidarDriver *>(sl::createLidarDriver());
  auto op_result = (*lidar)->connect(channel.get()->value);
  if (sl::SL_IS_OK(op_result)) {
    (*lidar)->setMotorSpeed();
    (*lidar)->startScan(0, 1);
  } else {
    fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", op_result);
  }
}

bool RplidarWrapper::get(LiDARDataWrapper &data) {
  sl_lidar_response_measurement_node_hq_t nodes[8192];
  size_t count = std::size(nodes);

  auto op_result = (*lidar)->grabScanDataHq(nodes, count);

  (*lidar)->ascendScanData(nodes, count);

  for (int pos = 0; pos < (int)count; ++pos) {
    if (nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT != 47) {
      continue;
    }

    float degree = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
    float dist = std::min(max_dist, nodes[pos].dist_mm_q2 / 4.0f);

    data.insert(degree, dist);
  }

  return true;
}

RplidarWrapper::~RplidarWrapper() {
  (*lidar)->stop();
  (*lidar)->setMotorSpeed(0);
}
