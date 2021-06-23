# Copernicus
Version:1.1.0
ROS packages used to to run copernicus with hardware

* copernicus_base - Package contains files to connect to the robot and run the sensor driver.
* copernicus_control - Control configuration required for ROS control.
* copernicus_description - copernicus Robot description (URDF files).
* copernicus_localization - Sensor fusion of wheel odometry and IMU data using the robot localization package.
* copernicus_msgs - contains ROS messages specific to copernicus such as the power board messages,..etc
* copernicus_navigation - Contains files required for running gmapping, amcl and move_base ROS packages.
* copernicus_teleoperation - Control the robot using keyboard or Joystick.
* copernicus_rules - Contains rules that ensures the USB peripherals are binded correctly

## Steps to Launch copernicus :

### 1. Mapping an environment with copernicus
* Launch the move_base node by running:   
```
$ roslaunch copernicus_base bringup.launch   
```
* Launch the sensors node by running:    
```
$ roslaunch copernicus_base sensors.launch   
```
* To perform the mapping, launch the navigation package by running   
```
$ roslaunch copernicus_navigation navigation.launch gmapping:=true  
```
* Launch the rviz visualization tool by running:   
```
$ rosrun rviz rviz  
```
You can then open the copernicus configured rviz environment by opening the copernicus rviz config file, located under copernicus_navigation->rviz_config->navigation.rviz, from the rviz tool

* In order to control the robot, launch the teleoperation node by running:     
```
$ roslaunch copernicus_teleoperator teleoperator.launch keyboard:=true
```

* Once the mapping of the entire environment is completed, the map can be saved by running:     
```
$ rosrun map_server map_saver –f <filename>
```

You can then open the copernicus configured rviz environment by opening the copernicus rviz config file, located under copernicus_navigation->rviz_config->navigation.rviz, from the rviz tool

Use the 2D Nav Goal tool in the top toolbar to select a navigation goal in the visualizer. Ensure that the nav goal is given in a mapped section of the map
