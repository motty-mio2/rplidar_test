#include <lidar_data.hpp>

void to_json(nlohmann::json j, const LidarMetadata &t) {
  j = nlohmann::json{{"x", t.x},
                     {"y", t.y},
                     {"min_rad", t.min_rad},
                     {"max_rad", t.max_rad},
                     {"max_dist", t.max_dist}};
}

void from_json(const nlohmann::json &j, LidarMetadata &t) {
  j.at("x").get_to(t.x);
  j.at("y").get_to(t.y);
  j.at("min_rad").get_to(t.min_rad);
  j.at("max_rad").get_to(t.max_rad);
  j.at("max_dist").get_to(t.max_dist);
}

std::vector<uint8_t> LidarMetadata::dump() {
  return nlohmann::json::to_msgpack(*this);
}

LidarMetadata::LidarMetadata(const std::vector<uint8_t> bin_data) {
  auto j = nlohmann::json::from_msgpack(bin_data);
  j.get_to(*this);
}
