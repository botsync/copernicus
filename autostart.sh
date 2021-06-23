#!/bin/bash
rossetup=/opt/ros/melodic/setup.bash
catkin_ws_setup=/home/copernicus/copernicus_ws/devel/setup.bash
launcher="roslaunch copernicus_base bringup.launch"
pathfile=/home/copernicus
pathfile2=/lib/systemd/system
cat <<EOF >$pathfile/robotstart.sh
#!/bin/bash
bash -c "source $rossetup && source $catkin_ws_setup && $launcher"
EOF
sudo chmod u+x $pathfile/robotstart.sh
cat <<EOF >$pathfile2/robotstart.service
[Unit]
Description=robot Auto Start
After=multi-user.target
[Service]
Type=idle
ExecStart=$pathfile/robotstart.sh
[Install]
WantedBy=multi-user.target
EOF
sudo chmod 777 $pathfile2/robotstart.service
sudo systemctl daemon-reload
sudo systemctl enable robotstart.service
echo "line25"
sudo systemctl start robotstart.service
echo "line27"
