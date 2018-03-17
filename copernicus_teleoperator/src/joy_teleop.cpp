#include <copernicus_teleoperator/joy_teleop.h>

void pb_flags_callback(const copernicus_msgs::PBStatusFlags::ConstPtr& pb_flags) {
    software_e_stop_state = pb_flags->E_STOP_FLAG_SW;
    hardware_e_stop_state = pb_flags->E_STOP_FLAG_HW;
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& joy) {
    geometry_msgs::Twist cmd;

    if (enable_e_stop && joy->buttons[e_stop_button] && !software_e_stop_state) {
        std_msgs::Bool e_stop_msg;

        e_stop_msg.data = true;

        // Publish 0 velocity
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;

        e_stop_pub.publish(e_stop_msg);
    } else if (enable_e_stop && joy->buttons[e_stop_button] && software_e_stop_state) {
        std_msgs::Bool e_stop_msg;

        e_stop_msg.data = true;

        // Publish 0 velocity
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;

        e_stop_pub.publish(e_stop_msg);
    } else if (joy->buttons[stop_button]) {
        // Publish 0 velocity
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;

        // Actionlib_msg to publish cancel message to move_base node. This cancels all navigation goals and stops the robot.
        actionlib_msgs::GoalID goal_id = actionlib_msgs::GoalID();
        stop_pub.publish(goal_id);
    } else if (joy->buttons[enable_button]) {
        cmd.linear.x = max_linear_speed * joy->axes[linear_speed_axis];
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = max_angular_speed * joy->axes[angular_speed_axis];

        if (enable_holonomic_movement && sideways_speed_axis >= 0.0) {
            cmd.linear.y = max_linear_speed * joy->axes[sideways_speed_axis];
        }
    } else {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;
    }

    cmd_vel_pub.publish(cmd);
}
