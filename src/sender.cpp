#include <gflags/gflags.h>
#include <rplidar.h>
#include <signal.h>

#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include "flags.hpp"
#include "lidar_device/random_lidar.hpp"
#include "lidar_device/rplidar_wrapper.hpp"
#include "lidar_types/lidar_data.hpp"
#include "zenoh.hxx"

DECLARE_string(d);
DECLARE_double(max_dist);

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

  // Lidar Setup
  // RplidarWrapper lidar(FLAGS_d, FLAGS_max_dist);
  RandomLiDAR lidar;

  auto data = LiDARDataWrapper();

  while (!ctrl_c_pressed) {
    data.clear();

    if (lidar.get(data)) {
      publisher.put(data.dump());
    }
  }
  return 0;
}