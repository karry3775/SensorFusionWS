# SensorFusionWS

### A ROS-based open-source software for fusing mobile sensor data and visualizing SmartPhone Orientation

### Connecting the Phone through PhonePi App
- Connect your phone and computer to the same network (Hotspot can be used)
- Enter \<ip address\>:5000 to the URL field in PhonePi app
- ip addess could be find using ```ifconfig``` command on linux terminal and ```ipconfig``` on windows cmd prompt. 

### Visualize Phone Orientation (Just from Accelerometer and Magnetometer data)
```
$ roscore
$ rosrun sensor_fusion_pkg sensor_streamer.py
$ rosrun sensor_fusion_pkg test_node_for_accel_mag.py
$ roslaunch phone_description phone_spawn.launch
$ roslaunch phone_description phone_rviz.launch

```

### Visualize Phone Orientation (Just from Gyroscope data)
```
$ roscore
$ rosrun sensor_fusion_pkg sensor_streamer.py
$ rosrun sensor_fusion_pkg gyro_only_orientation.py
$ roslaunch phone_description phone_spawn.launch
$ roslaunch phone_description phone_rviz.launch

```

### Visualize Phone Orientation (Fused orientation from Gyro, Accelerometer and Magnetometer using Complementary Filter)
```
$ roscore
$ rosrun sensor_fusion_pkg sensor_streamer.py
$ rosrun sensor_fusion_pkg complimentary_filter.py
$ roslaunch phone_description phone_spawn.launch
$ roslaunch phone_description phone_rviz.launch

```
### Equations used for Complementary Filter
#### Accelerometer
![roll_accel](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/a_roll.gif)

![pitch_accel](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/pitch_a.gif)

#### Magnetometer
![M_x](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/Mx_mag.gif)

![M_y](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/My_mag.gif)

![m_yaw](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/yaw_mag.gif)
#### Gyroscope
![roll_g](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/roll_gyro.gif)

![pitch_g](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/pitch_gyro.gif)

![yaw_g](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/yaw_gyro.gif)

### Demonstration of Complementary Filter

[![Complimentary Filter](http://img.youtube.com/vi/bj4u9_aLW6o/0.jpg)](https://www.youtube.com/watch?v=bj4u9_aLW6o "Complimentary Filter")

### Visualize Phone Orientation (Fused orientation from Gyro, Accelerometer and Magnetometer using Kalman Filter)
```
$ roscore
$ rosrun sensor_fusion_pkg sensor_streamer.py
$ rosrun sensor_fusion_pkg kalman_filter.py
$ roslaunch phone_description phone_spawn.launch
$ roslaunch phone_description phone_rviz.launch

```
### Equations used for Kalman Filter
#### Prediction
![kf_prediction](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/kf_prediction.gif)

![kf_prediction_cov](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/kf_prediction_cov.gif)

#### Kalman Gain
![kgain_1](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/kgain_1.gif)

![kgain_2](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/kgain_2.gif)

#### Correction
![kf_correction_state](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/kf_correction_state.gif)

![kf_correction_cov](https://github.com/karry3775/SensorFusionWS/blob/master/src/images/kf_correction_cov.gif)

### Comparison between fused and gyro only estimates (RPYCF - complementary filter, RPYG - gyro only, RPYKF - kalman filter)
### Body Yaw
![Body Yaw](https://github.com/karry3775/SensorFusionWS/blob/master/src/SensorFusion/images/comparison_yaw2.png)

### Body Pitch
![Body Yaw](https://github.com/karry3775/SensorFusionWS/blob/master/src/SensorFusion/images/comparison_pitch.png)

### Body Roll
![Body Yaw](https://github.com/karry3775/SensorFusionWS/blob/master/src/SensorFusion/images/comparison_roll.png)
