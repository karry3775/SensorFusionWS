# SensorFusionWS

### A ROS-based open-source software for fusing and visualizing SmartPhone Orientation

### Connecting the Phone through PhonePi App
- Connect your phone and computer to the same network (Hotspot can be used)
- Enter <ip address>:5000 to the URL field in PhonePi app
- ip addess could be find using ifconfig command on linux and ipconfig on windows. 

### Visualize Phone Orientation (Just from Accelerometer and Magnetometer data)
```
$ roscore
$ rosrun sensor_fusion_pkg sensor_streamer.py
$ rosrun sensor_fusion_pkg test_node_for_accel_mag.py
$ roslaunch phone_description phone_spawn.launch
$ roslaunch phone_description phone_rviz.launch

```

### Visualize Phone Orientation (Just from GyroScope data)
```
$ roscore
$ rosrun sensor_fusion_pkg sensor_streamer.py
$ rosrun sensor_fusion_pkg gyro_only_orientation.py
$ roslaunch phone_description phone_spawn.launch
$ roslaunch phone_description phone_rviz.launch

```

### Visualize Phone Orientation (Fused orientation from Gyro, Accelerometer and Magnetometer using Complimentary Filter)
```
$ roscore
$ rosrun sensor_fusion_pkg sensor_streamer.py
$ rosrun sensor_fusion_pkg complimentary_filter.py
$ roslaunch phone_description phone_spawn.launch
$ roslaunch phone_description phone_rviz.launch

```

### Demonstration of Complimentary Filter

[![Complimentary Filter](http://img.youtube.com/vi/bj4u9_aLW6o/0.jpg)](https://www.youtube.com/watch?v=bj4u9_aLW6o "Complimentary Filter")

### Visualize Phone Orientation (Fused orientation from Gyro, Accelerometer and Magnetometer using Kalman Filter)
```
$ roscore
$ rosrun sensor_fusion_pkg sensor_streamer.py
$ rosrun sensor_fusion_pkg kalman_filter.py
$ roslaunch phone_description phone_spawn.launch
$ roslaunch phone_description phone_rviz.launch

```
### Comparison between Kalman Filter and Complimentary Filter estimates
#### Body Yaw 
![Body Yaw](https://github.com/karry3775/SensorFusionWS/blob/master/src/SensorFusion/images/yaw_comparison.png)
#### Body Pitch
![Body Pitch](https://github.com/karry3775/SensorFusionWS/blob/master/src/SensorFusion/images/pitch_comparison.png)
#### Body Roll
![Body Roll](https://github.com/karry3775/SensorFusionWS/blob/master/src/SensorFusion/images/roll_comparison.png)
