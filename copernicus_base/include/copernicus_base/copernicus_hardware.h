#ifndef MAGELLAN_BASE_MAGELLAN_HARDWARE_H
#define MAGELLAN_BASE_MAGELLAN_HARDWARE_H

#include "ros/ros.h"

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include <string>
#include "sensor_msgs/JointState.h"

#include <copernicus_base/constants.h>

// ROS messages for ROSSerial Comms
#include <copernicus_msgs/RPM.h>

namespace copernicus_base {
class CopernicusHardware: public hardware_interface::RobotHW {
public:
    CopernicusHardware(ros::NodeHandle nh, double target_control_freq);
    void update_encoder_readings();
    void send_velocities();
    void register_controllers();
    void rpm_callback(const copernicus_msgs::RPM::ConstPtr& rpm) {
        this->encoder_left_rpm = rpm->left;
        this->encoder_right_rpm = rpm->right; 
    }
private:
    void publish_rpm(int left, int right);
    void subscribe_encoder();

    void limit_speeds(int16_t &left, int16_t &right);
    double convert_rpm_to_radians(int16_t rpm);
    int16_t convert_radians_to_rpm(double radians);

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    double target_control_freq;
    double wheel_diameter_, max_accleration_, max_speed_, max_rpm_;

    ros::Publisher rpm_publisher;
    ros::Subscriber rpm_subscriber;

    int16_t encoder_left_rpm, encoder_right_rpm;

    ros::NodeHandle nh_;

    struct Joint {
        double position;
        double position_offset;
        double velocity;
        double effort;
        double velocity_command;

        Joint() :
            position(0),velocity(0),effort(0),velocity_command(0)
            {
            }
    } joints_[4];
};
}
#endif