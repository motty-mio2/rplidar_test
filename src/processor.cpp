#include <chrono>
#include <cstdint>
#include <iostream>
#include <lidar_types/lidar_data.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "flags.hpp"
#include "visualizer.hpp"
#include "zenoh.hxx"

std::chrono::system_clock::time_point ntp64_to_timepoint(uint64_t ntp64) {
  uint32_t seconds = (ntp64 >> 32);  // NTPエポックからの秒数
  uint32_t fraction = ntp64 & 0xFFFFFFFF;

  // 小数部をナノ秒に変換
  uint64_t nanos = (static_cast<uint64_t>(fraction) * 1000000000ULL) >> 32;

  return std::chrono::system_clock::time_point{std::chrono::seconds(seconds) +
                                               std::chrono::nanoseconds(nanos)};
}

int main() {
  std::map<std::string,
           std::pair<std::chrono::system_clock::time_point, cv::Mat>>
      timestamps;

  bool updated = false;

  // Zenoh Setup
  auto config = zenoh::Config::create_default();
  config.insert_json5(Z_CONFIG_ADD_TIMESTAMP_KEY, "true");

  auto session = zenoh::Session(std::move(config));
  session.declare_background_subscriber(  //
      zenoh::KeyExpr("lidar/data"),       //
      [&timestamps, &updated](const zenoh::Sample &sample) {
        auto timestamp = ntp64_to_timepoint(sample.get_timestamp()->get_time());

        auto id = sample.get_timestamp()->get_id().to_string();
        auto data = sample.get_payload().as_vector();
        auto z = LiDARDataWrapper(data);

        timestamps[id] = {timestamp, visualize(z.get(), id)};

        updated = true;
      },
      zenoh::closures::none);

  while (true) {
    if (updated) {
      for (auto &[id, pair] : timestamps) {
        auto &[timestamp, img] = pair;
        cv::imshow(id, img);
      }

      cv::waitKey(1);

      updated = false;
      continue;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    auto now = std::chrono::system_clock::now();

    for (auto it = timestamps.begin(); it != timestamps.end();) {
      if (now - it->second.first > std::chrono::seconds(5)) {
        cv::destroyWindow(it->first);
        it = timestamps.erase(it);
      } else {
        ++it;
      }
    }
  }

  return 0;
}