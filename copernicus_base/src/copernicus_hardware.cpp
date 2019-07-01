#include <copernicus_base/copernicus_hardware.h>
#include <boost/assign/list_of.hpp>

namespace copernicus_base {

CopernicusHardware::CopernicusHardware(ros::NodeHandle nh, double target_control_freq) {
    this->nh_ = nh;
    this->target_control_freq = target_control_freq;
    this->max_rpm_ = MAX_RPM;

    this->rpm_publisher = this->nh_.advertise<copernicus_msgs::RPM>("set_rpm", 1000);
    this->rpm_subscriber = this->nh_.subscribe("get_rpm", 1000, &CopernicusHardware::rpm_callback, this);
    
    this->register_controllers();
}

void CopernicusHardware::register_controllers() {
    ros::V_string joint_names = boost::assign::list_of("front_left_wheel_joint")("front_right_wheel_joint")("back_left_wheel_joint")("back_right_wheel_joint");
    for (unsigned int i = 0 ; i< joint_names.size(); i++) {
        hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);

        joint_state_interface_.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle(joint_state_handle,&joints_[i].velocity_command);
        velocity_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
}

void CopernicusHardware::update_encoder_readings() {
    double left, right;
    left = (double)this->convert_rpm_to_radians(this->encoder_left_rpm);
    right = (double)this->convert_rpm_to_radians(this->encoder_right_rpm);

    for (int i=0; i<4; i++) {
        if (i%2 == 0) {
            this->joints_[i].velocity = left;
        } else {
            this->joints_[i].velocity = right;
        }
    }
}

void CopernicusHardware::send_velocities() {
    double left = this->joints_[0].velocity_command;
    double right = this->joints_[1].velocity_command;

    int16_t rpm_left, rpm_right;

    rpm_left = (int16_t)this->convert_radians_to_rpm(left);
    rpm_right = (int16_t)this->convert_radians_to_rpm(right);

    this->limit_speeds(rpm_left, rpm_right);

    copernicus_msgs::RPM rpm;
    rpm.left = rpm_left;
    rpm.right = rpm_right;
    this->rpm_publisher.publish(rpm);
}

void CopernicusHardware::limit_speeds(int16_t &left, int16_t &right) {
    int16_t temp_max = std::max(std::abs(left), std::abs(right));

    if (temp_max > this->max_rpm_) {
        left *= this->max_rpm_/temp_max;
        right *= this->max_rpm_/temp_max;
    }
}

int16_t CopernicusHardware::convert_radians_to_rpm(double radians) {
    int16_t ret= (int16_t)(radians*60.0*MOTOR_REDUCTION)/(2.0*PI);
    return ret;
}

double CopernicusHardware::convert_rpm_to_radians(int16_t rpm) {
    double ret= (double)((rpm*2.0*PI)/(60.0*MOTOR_REDUCTION));
    return ret;
}

}