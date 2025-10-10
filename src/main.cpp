#include <rplidar.h>
#include <signal.h>

#include <iostream>
#include <limits>

#include "arc_intersection.hpp"
#include "degree2position.hpp"
#include "flags.hpp"
#include "lidar_obj.hpp"
#include "visualizer.hpp"
#include "zenoh_wrapper.hpp"
using namespace sl;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

volatile sig_atomic_t ctrl_c_pressed = 0;

void ctrlc_handler(int) { ctrl_c_pressed = 1; }

int main(int argc, char *argv[]) {
  gflags::SetUsageMessage("How To Use");
  gflags::SetVersionString("1.0.0");

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << "Hello, RPLIDAR!" << std::endl;

  signal(SIGINT, ctrlc_handler);

  ///  Create a communication channel instance
  IChannel *_channel;
  Result<IChannel *> channel = createSerialPortChannel(FLAGS_d, 115200);

  ///  Create a LIDAR driver instance
  ILidarDriver *lidar = *createLidarDriver();
  auto res = (*lidar).connect(*channel);
  if (SL_IS_OK(res)) {
    sl_lidar_response_device_info_t deviceInfo;
    res = (*lidar).getDeviceInfo(deviceInfo);
    if (SL_IS_OK(res)) {
      printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
             deviceInfo.model, deviceInfo.firmware_version >> 8,
             deviceInfo.firmware_version & 0xffu, deviceInfo.hardware_version);
    } else {
      fprintf(stderr, "Failed to get device information from LIDAR %08x\r\n",
              res);
    }
  } else {
    fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
  }

  lidar->setMotorSpeed();
  // start scan...
  lidar->startScan(0, 1);

  // fetech result and print it out...

  while (!ctrl_c_pressed) {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);

    sl_result op_result = lidar->grabScanDataHq(nodes, count);

    auto data = LiDARDataWrapper();

    if (SL_IS_OK(op_result)) {
      lidar->ascendScanData(nodes, count);

      for (int pos = 0; pos < (int)count; ++pos) {
        if (nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT !=
            47) {
          continue;
        }

        float degree = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
        float dist = std::min(FLAGS_max_dist,  //
                              (double)nodes[pos].dist_mm_q2 / 4.0f);

        data.insert(degree, dist);
      }
    }

    visualize(data.get());
    data.clear();
  }

  lidar->stop();
  lidar->setMotorSpeed(0);

  return 0;
}
