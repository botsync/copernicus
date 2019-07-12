/*
 Copyright (c) 2016, NTU-MBZIRC Team
 Credits to:
 Juan Jimeno https://github.com/grassjelly

 Source: http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of  nor the names of its contributors may be used to
 endorse or promote products derived from this software without specific
 prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <copernicus_msgs/Velocities.h>
#include <math.h>
#include <tf/tf.h>

double vel_x = 0.0;
double vel_y = 0.0;
double ang_z = 0.0;
double vel_dt = 0.0;
ros::Publisher odom_pub;
tf::TransformBroadcaster odom_broadcaster;

ros::Time last_loop_time(0.0);
ros::Time last_vel_time(0.0);

double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;

void velCallback( const copernicus_msgs::Velocities& vel) {
  //callback every time the robot's linear velocity is received
  ros::Time current_time = ros::Time::now();

  vel_x = vel.linear_x;
  vel_y = vel.linear_y;
  ang_z = vel.angular_z;
  vel_dt = (current_time - last_vel_time).toSec();
  last_vel_time = current_time;

  //linear velocity is the linear velocity published from the Teensy board in x axis
  double linear_velocity_x = vel_x;

  //linear velocity is the linear velocity published from the Teensy board in y axis
  double linear_velocity_y = vel_y;

  double angular_velocity_z = ang_z;

  //calculate angular displacement  Î¸ = Ï‰ * t
  double delta_theta = angular_velocity_z * vel_dt; //radians
  double delta_x = (linear_velocity_x * cos(theta) - linear_velocity_y * sin(theta)) * vel_dt; //m
  double delta_y = (linear_velocity_x * sin(theta) + linear_velocity_y * cos(theta)) * vel_dt; //m

  //calculate current position of the robot
  x_pos += delta_x;
  y_pos += delta_y;
  theta += delta_theta;

  //calculate robot's heading in quarternion angle
  //ROS has a function to calculate yaw in quaternion angle
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  //robot's position in x,y, and z
  odom_trans.transform.translation.x = x_pos;
  odom_trans.transform.translation.y = y_pos;
  odom_trans.transform.translation.z = 0.0;
  //robot's heading in quaternion
  odom_trans.transform.rotation = odom_quat;
  odom_trans.header.stamp = current_time;
  //publish robot's tf using odom_trans object
  odom_broadcaster.sendTransform(odom_trans);

  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  //robot's position in x,y, and z
  odom.pose.pose.position.x = x_pos;
  odom.pose.pose.position.y = y_pos;
  odom.pose.pose.position.z = 0.0;
  //robot's heading in quaternion
  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = "base_link";
  //linear speed from encoders
  odom.twist.twist.linear.x = linear_velocity_x;
  odom.twist.twist.linear.y = linear_velocity_y;
  odom.twist.twist.linear.z = 0.0;

  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = angular_velocity_z;

  odom_pub.publish(odom);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle n;
  ros::NodeHandle nh_private_("~");
  ros::Subscriber sub = n.subscribe("raw_vel", 1, velCallback);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);

  ros::spin();
}