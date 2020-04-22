# SensorFusionWS
### A ROS-based open-source software for fusing mobile sensor data and visualizing SmartPhone Orientation

### System and software used:
- Python
- ROS melodic
- Ubuntu 18.04 LTS
- Gazebo 9 simulator

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

### Visualize Phone Orientation (Just from Gyroscope data) [transform broadcaster needs to be enabled]
```
$ roscore
$ rosrun sensor_fusion_pkg sensor_streamer.py
$ rosrun sensor_fusion_pkg gyro_only_orientation.py
$ roslaunch phone_description phone_spawn.launch
$ roslaunch phone_description phone_rviz.launch

```

### Visualize Phone Orientation (Fused orientation from Gyro, Accelerometer and Magnetometer using Complementary Filter)
### [transform broadcaster needs to be enabled]
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

### Real Time Visualization 
[![Complimentary Filter](http://img.youtube.com/vi/bj4u9_aLW6o/0.jpg)](https://www.youtube.com/watch?v=bj4u9_aLW6o "Complimentary Filter")

### Kalman Filter Estimates
[![Kalman Filter](http://img.youtube.com/vi/mp79Vo_6c6o/0.jpg)](https://www.youtube.com/watch?v=mp79Vo_6c6o "Kalman Filter")

### Complementary Filter Estimates
[![Complementary Filter](http://img.youtube.com/vi/qFsYn6i03hA/0.jpg)](https://www.youtube.com/watch?v=qFsYn6i03hA "Complementary Filter")


### Visualize Phone Orientation (Fused orientation from Gyro, Accelerometer and Magnetometer using Kalman Filter)
### [transform broadcaster needs to be enabled]
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

### Comparison with accurate Angular Benchmark
```
$ roscore
$ rosrun sensor_fusion_pkg sensor_streamer.py
$ rosrun sensor_fusion_pkg ground_truth_comparison.py
$ rosrun sensor_fusion_pkg kalman_filter.py
$ rosrun sensor_fusion_pkg complimentary_filter.py
$ rosrun sensor_fusion_pkg gyro_only_orientation.py
```
##### Note : The phone needs to be aligned with the angular benchmark in such a way that 0 degree yaw corresponds with 0 degrees. Successive positioning of the phones heading on the benchmark is carried out.

![benchmark](https://github.com/karry3775/SensorFusionWS/blob/master/src/SensorFusion/images/GT_BENCHMARK_cropped-1.png)
![compare](https://github.com/karry3775/SensorFusionWS/blob/master/src/SensorFusion/images/gt_2.png)

### Benchmark using ground truth obtained using articulated robotic arm (ABB4600)
```
$ roscore
$ download bagfiles from google drive (square_traj_01.bag and rot_180_01.bag) and place in random folder
$ open new terminal and type "rqt_bag" and navigate to one of the bagfiles
$ right click on the topics "Accel_topic_stamped", "Gyro_topic_stamped", "Magno_topic_stamped" and "ee_rotation_stamped" and click "publish" to publish these topics
$ "rosrun sensor_fusion_pkg kalman_filter_robot.py" in separate terminal
$ Click the play button to publish topics.  You should see text being printed in executable terminal
$ Stop publishing at any time and press ctrl+c in terminal.  Plot will be generated
```

