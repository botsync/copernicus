#include <copernicus_base/copernicus_hardware.h>
#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

void controlLoop(copernicus_base::CopernicusHardware & copernicus, controller_manager::ControllerManager &cm, time_source::time_point &last_time) {
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    copernicus.update_encoder_readings();
    cm.update(ros::Time::now(), elapsed);
    copernicus.send_velocities();
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "copernicus_base");
    ros::NodeHandle nh;
    double control_freq;

    nh.param<double>("control_frequency", control_freq, 10.0);

    copernicus_base::CopernicusHardware copernicus(nh, control_freq);
    controller_manager::ControllerManager cm(&copernicus);
    ros::CallbackQueue queue;
    ros::AsyncSpinner spinner(1, &queue);
    
    time_source::time_point last_time = time_source::now();
    ros::Time last_time_d = ros::Time::now();
    ros::TimerOptions control_timer(
    ros::Duration(1 / control_freq),
    boost::bind(controlLoop, boost::ref(copernicus), boost::ref(cm), boost::ref(last_time)),
    &queue);
    ros::Timer control_loop = nh.createTimer(control_timer);

    spinner.start();

    ros::spin();

    return 0;
}