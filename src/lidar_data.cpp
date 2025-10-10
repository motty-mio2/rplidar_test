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

void to_json(nlohmann::json &j, const LiDARDataWrapper &p) {
  j = nlohmann::json{{"data", p.data}};
}

void from_json(const nlohmann::json &j, LiDARDataWrapper &p) {
  j.at("data").get_to(p.data);
}

LiDARDataWrapper::LiDARDataWrapper(const std::vector<uint8_t> bin_data) {
  nlohmann::json::from_msgpack(bin_data).at("data").get_to(data);
}

void LiDARDataWrapper::insert(float degree, float value) {  //
  data[degree] = value;
}

void LiDARDataWrapper::clear() {  //
  data.clear();
}

const LiDARData LiDARDataWrapper::get() {  //
  return data;
}

void LiDARDataWrapper::get(LiDARData &s) { s = data; }

std::vector<uint8_t> LiDARDataWrapper::dump() {
  return nlohmann::json::to_msgpack(data);
}