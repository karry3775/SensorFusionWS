# SensorFusionWS

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
