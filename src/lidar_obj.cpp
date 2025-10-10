#include <lidar_obj.hpp>

void to_json(nlohmann::json j, const metadata_t &t) {
  j = nlohmann::json{{"x", t.x},
                     {"y", t.y},
                     {"min_rad", t.min_rad},
                     {"max_rad", t.max_rad},
                     {"max_dist", t.max_dist}};
}

void from_json(const nlohmann::json &j, metadata_t &t) {
  j.at("x").get_to(t.x);
  j.at("y").get_to(t.y);
  j.at("min_rad").get_to(t.min_rad);
  j.at("max_rad").get_to(t.max_rad);
  j.at("max_dist").get_to(t.max_dist);
}

void to_json(nlohmann::json &j, const lidar_data &p) {
  j = nlohmann::json{{"data", p.data}};
}

void from_json(const nlohmann::json &j, lidar_data &p) {
  j.at("data").get_to(p.data);
}

LiDAR_DATA_WRAPPER::LiDAR_DATA_WRAPPER(const std::vector<uint8_t> bin_data) {
  nlohmann::json::from_msgpack(bin_data).at("data").get_to(data);
}

void LiDAR_DATA_WRAPPER::insert(float degree, float value) {  //
  data[degree] = value;
}

void LiDAR_DATA_WRAPPER::clear() {  //
  data.clear();
}

const sensor_data_t LiDAR_DATA_WRAPPER::get() {  //
  return data;
}

void LiDAR_DATA_WRAPPER::get(sensor_data_t &s) { s = data; }

std::vector<uint8_t> LiDAR_DATA_WRAPPER::dump() {
  return nlohmann::json::to_msgpack(data);
}