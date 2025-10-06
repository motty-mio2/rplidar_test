#include "zenoh_wrapper.hpp"

ZenohWrapper::ZenohWrapper() {
  auto config = zenoh::Config::create_default();
  config.insert_json5(Z_CONFIG_ADD_TIMESTAMP_KEY, "true");

  session = std::make_unique<zenoh::Session>(std::move(config));
  publisher = std::make_unique<zenoh::Publisher>(
      session->declare_publisher(zenoh::KeyExpr("cam/jpg")));
}
void ZenohWrapper::publish(std::string data) {
  publisher->put(zenoh::Bytes(data));
}