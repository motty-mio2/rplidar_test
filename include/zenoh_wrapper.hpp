#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

#include <memory>
#include <string>
#include <thread>

#include "zenoh.hxx"

class ZenohWrapper {
private:
  std::unique_ptr<zenoh::Session> session;
  std::unique_ptr<zenoh::Publisher> publisher;

public:
  ZenohWrapper();
  void publish(std::string data);
};

#endif // PUBLISHER_HPP_
