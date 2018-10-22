#include "copernicus_control/pid_core.h"

CopernicusPID::CopernicusPID() {}

CopernicusPID::~CopernicusPID() {}

void CopernicusPID::publishMessage(ros::Publisher *pub) {
  copernicus_msgs::PID msg;
  msg.p = p_;
  msg.d = d_;
  msg.i = i_;
  pub->publish(msg);
}

void CopernicusPID::configCallback(copernicus_control::copernicusPIDConfig &config, double level) {
  p_ = config.p;
  d_ = config.d;
  i_ = config.i;
}