#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <copernicus_msgs/PBFlags.h>
#include <copernicus_msgs/SubsPBFlags.h>
#include <actionlib_msgs/GoalID.h>
#include "std_msgs/Bool.h"

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
void pb_flags_callback(const copernicus_msgs::PBFlags::ConstPtr& pb_flags);

ros::Publisher cmd_vel_pub;
ros::Publisher e_stop_pub;
ros::Publisher stop_pub;

bool software_e_stop_state = false;
bool hardware_e_stop_state = true;

int enable_button;
int stop_button;
int e_stop_button;
int linear_speed_axis;
int angular_speed_axis;

int enable_holonomic_movement;
int sideways_speed_axis;
int enable_e_stop;

double max_linear_speed;
double max_angular_speed;
