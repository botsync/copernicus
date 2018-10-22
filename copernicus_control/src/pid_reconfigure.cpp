#include "copernicus_control/pid_core.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pid_reconfigure");
  ros::NodeHandle nh;

  double p;
  double d;
  double i;
  int rate;

  CopernicusPID *copernicusPID = new CopernicusPID();

  dynamic_reconfigure::Server<copernicus_control::copernicusPIDConfig> dr_server;
  dynamic_reconfigure::Server<copernicus_control::copernicusPIDConfig>::CallbackType callback;
  callback = boost::bind(&CopernicusPID::configCallback, copernicusPID, _1, _2);
  dr_server.setCallback(callback);

  ros::NodeHandle snh("~");
  snh.param("p", p, 0.05);
  snh.param("d", d, 0.10);
  snh.param("i", i, 0.00);
  snh.param("rate", rate, 1);

  ros::Publisher pub_message = nh.advertise<copernicus_msgs::PID>("pid", 10);

  ros::Rate r(rate);

  while (nh.ok()) {
    copernicusPID->publishMessage(&pub_message);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}