# Copernicus

Botsync Copernicus is a 4 Wheel Differential Drive robot. This repo contains all ROS packages to autonomously navigate Copernicus.

# Running the Simulation
1. To run in Simulation, launch the gazebo node first using
```roslaunch copernicus_description gazebo.launch```

2. Followed by:
```roslaunch copernicus_description one_robot.launch```

3. Open Rviz and use the config file at copernicus_navigation/rviz_config/navigation.rviz to visualize.

# Running Online
1. ```roslaunch copernicus_base copernicus_bringup.launch```
2. ```roslaunch copernicus_base copernicus_base.launch```
3. ```roslaunch copernicus_navigation navigation.launch```

# Contributing Guidelines
1. Create a branch or fork this repo.
2. Submit a pull request once features have been added.
3. Add all necessary information to the pull request message.
