# Volta Hardware

ROS2 package to interface with Volta Hardware.

## Porting to ROS2
1. Added volta_hardware_to_ROS file 
    - Contains server to respond with /rpm_sub values from encoder

2. volta_hardware file
    - Exported as a plugin to be loaded by the controller manager 

3. Update PID values on Volta
```
$ ros2 service call /volta_pid_service volta_msgs/srv/Pid "{kp1: 0.23, ki1: 0.18, kd1: 0.002, kp2: 0.23, ki2: 0.18, kd2: 0.002, write_contros: True, save_to_eeprom: True, reset_pid: False}"
```
   - Provide all the six PID values (kp1, ki1, kd1, kp2, ki2, kd), when calling the service, to ensure non zero value 

## Maintainer Info
1. Name: Nikhil Venkatesh, James Abraham, Sachin Devadiga
2. E-mail: [nikhil@botsync.co](mailto:nikhil@botsync.co)
