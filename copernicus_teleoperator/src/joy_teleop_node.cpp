#include <ros/ros.h>
#include <copernicus_teleoperator/joy_teleop.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_teleop_node");

    ros::NodeHandle nh;

    nh.param<int>("enable_button", enable_button, 1);
    nh.param<int>("stop_button", stop_button, 2);
    nh.param<int>("e_stop_button", e_stop_button, 3);
    nh.param<int>("linear_speed_axis", linear_speed_axis, 1);
    nh.param<int>("angular_speed_axis", angular_speed_axis, 0);

    nh.param<double>("max_linear_speed", max_linear_speed, 0.3);
    nh.param<double>("max_angular_speed", max_angular_speed, 0.3);

    nh.param<int>("enable_e_stop", enable_e_stop, 0);

    nh.param<int>("enable_holonomic_movement", enable_holonomic_movement, 0);
    nh.param<int>("sideways_speed_axis", sideways_speed_axis, -1);

    std::string cmd_vel_topic, e_stop_pub_topic, joy_topic, e_stop_sub_topic;

    nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
    nh.param<std::string>("e_stop_pub_topic", e_stop_pub_topic, "/subsPBFlags");

    nh.param<std::string>("e_stop_sub_topic", e_stop_pub_topic, "/PBFlags");
    nh.param<std::string>("joy_topic", joy_topic, "/joy");

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1); 
    stop_pub = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

    ros::Subscriber joy_subscriber = nh.subscribe(joy_topic, 1, joy_callback);

    if (enable_e_stop) {
        e_stop_pub = nh.advertise<copernicus_msgs::SubsPBFlags>(e_stop_pub_topic, 1);
        ros::Subscriber pb_flags_subscriber = nh.subscribe(e_stop_sub_topic, 1, pb_flags_callback);
    }

    ros::spin();    

    return 0;
}