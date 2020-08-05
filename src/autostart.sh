#!/bin/bash
rossetup=/opt/ros/kinetic/setup.bash
rossetup2=/home/volta/volta_workspace/devel/setup.bash
launcher="roslaunch volta_base volta_bringup.launch"
launcher2="rosrun volta_base rpmSend"
pathfile=/home/volta
pathfile2=/lib/systemd/system
cat <<EOF >$pathfile/vehiclestart.sh
#!/bin/bash
bash -c "source $rossetup && source $rossetup2 && $launcher && launcher2"
EOF

sudo chmod u+x $pathfile/vehiclestart.sh

cat <<EOF >$pathfile2/vehiclestart.service
[Unit]
Description=Vehicle Auto Start
After=multi-user.target

[Service]
Type=idle
ExecStart=$pathfile/vehiclestart.sh

[Install]
WantedBy=multi-user.target
EOF
sudo chmod 644 $pathfile2/vehiclestart.service
sudo systemctl daemon-reload
sudo systemctl enable vehiclestart.service
sudo systemctl start vehiclestart.service
