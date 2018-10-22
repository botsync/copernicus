#include "ros/ros.h"
#include "ros/time.h"

#include <copernicus_msgs/PID.h>

#include <dynamic_reconfigure/server.h>

#include <copernicus_control/copernicusPIDConfig.h>

class CopernicusPID
{
public:
  CopernicusPID();
  ~CopernicusPID();
  void configCallback(copernicus_control::copernicusPIDConfig &config, double level);
  void publishMessage(ros::Publisher *pub_message);

  double p_;
  double d_;
  double i_;
};