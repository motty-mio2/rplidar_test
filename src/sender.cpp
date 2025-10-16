#include <gflags/gflags.h>
#include <rplidar.h>
#include <signal.h>

#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include "lidar_device/random_lidar.hpp"
#include "lidar_device/rplidar_wrapper.hpp"
#include "lidar_types/lidar_data.hpp"
#include "zenoh.hxx"

DEFINE_string(d, "/dev/ttyUSB0", "path/to/device");
DEFINE_double(m, 1000, "maximum distance in mm");
DEFINE_string(b, "random", "lidar backend (rplidar or random)");

volatile sig_atomic_t ctrl_c_pressed = 0;

void ctrlc_handler(int) { ctrl_c_pressed = 1; }

int main(int argc, char *argv[]) {
  // Flag Setup
  gflags::SetUsageMessage("How To Use");
  gflags::SetVersionString("1.0.0");

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  signal(SIGINT, ctrlc_handler);

  // Zenoh Setup

  auto config = zenoh::Config::create_default();
  config.insert_json5(Z_CONFIG_ADD_TIMESTAMP_KEY, "true");

  auto session = zenoh::Session(std::move(config));
  auto publisher = session.declare_publisher(zenoh::KeyExpr("lidar/data"));
  auto metadata_publisher =
      session.declare_publisher(zenoh::KeyExpr("lidar/metadata"));

  std::unique_ptr<MockLiDAR> lidar;

  if (FLAGS_b == "rplidar") {
    lidar = std::make_unique<RplidarWrapper>(FLAGS_d, FLAGS_m);
  } else if (FLAGS_b == "random") {
    lidar = std::make_unique<RandomLiDAR>(FLAGS_m);
  } else {
    std::cerr << "Invalid backend: " << FLAGS_b << std::endl;
    return 1;
  }

  auto data = LiDARDataWrapper();

  while (!ctrl_c_pressed) {
    data.clear();

    if (lidar && lidar->get(data)) {
      publisher.put(data.dump());
    }
  }
  return 0;
}