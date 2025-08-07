#include <rplidar.h>
#include <signal.h>

#include <iostream>
#include <vector>

using namespace sl;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

volatile sig_atomic_t ctrl_c_pressed = 0;

void ctrlc_handler(int) { ctrl_c_pressed = 1; }

int main() {
  std::cout << "Hello, RPLIDAR!" << std::endl;

  signal(SIGINT, ctrlc_handler);

  ///  Create a communication channel instance
  IChannel* _channel;
  Result<IChannel*> channel = createSerialPortChannel("/dev/ttyUSB0", 115200);

  ///  Create a LIDAR driver instance
  ILidarDriver* lidar = *createLidarDriver();
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

  int div = 4;
  float angle_step = 360.0f / div;
  std::vector<float> near;

  for (auto i = 0; i < div; ++i) {
    near.push_back(std::numeric_limits<float>::infinity());
  }

  while (!ctrl_c_pressed) {
    int loop_count = 0;

    for (auto& v : near) {
      v = std::numeric_limits<float>::infinity();
    }

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);

    sl_result op_result = lidar->grabScanDataHq(nodes, count);

    if (SL_IS_OK(op_result)) {
      lidar->ascendScanData(nodes, count);

      for (int pos = 0; pos < (int)count; ++pos) {
        if (nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT !=
            47) {
          continue;
        }
        float angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
        float dist = nodes[pos].dist_mm_q2 / 4.0f;

        loop_count += (angle > (loop_count + 1) * angle_step);

        near[loop_count] = std::min(near[loop_count], dist);

        //   printf("%i ", pos);
        //   printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
        //          (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "
        //          ", (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
        //          nodes[pos].dist_mm_q2 / 4.0f,
        //          nodes[pos].quality >>
        //          SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
      }
      std::cout << near[0] << " " << near[1] << " " << near[2] << " " << near[3]
                << std::endl;
    }
  }

  lidar->stop();
  lidar->setMotorSpeed(0);

  // cv::imshow("Test Window", cv::Mat::zeros(300, 300, CV_8UC3));
  // cv::waitKey(0);
  return 0;
}