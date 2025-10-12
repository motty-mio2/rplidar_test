#include <gflags/gflags.h>
#include <rplidar.h>
#include <signal.h>

#include <iostream>
#include <limits>

#include "arc_intersection.hpp"
#include "degree2position.hpp"
#include "flags.hpp"
#include "lidar_data.hpp"
#include "lidar_device/rplidar_wrapper.hpp"
#include "visualizer.hpp"
#include "zenoh_wrapper.hpp"

DECLARE_string(d);
DECLARE_double(max_dist);

volatile sig_atomic_t ctrl_c_pressed = 0;

void ctrlc_handler(int) { ctrl_c_pressed = 1; }

int main(int argc, char *argv[]) {
  gflags::SetUsageMessage("How To Use");
  gflags::SetVersionString("1.0.0");

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  signal(SIGINT, ctrlc_handler);

  RplidarWrapper lidar(FLAGS_d, FLAGS_max_dist);

  auto data = LiDARDataWrapper();

  while (!ctrl_c_pressed) {
    data.clear();

    if (lidar.get(data)) {
      visualize(data.get());
    }

    return 0;
  }
}