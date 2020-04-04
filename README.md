# SensorFusionWS

### Visualize Phone Orientation (Just from Accelerometer and Magnetometer data)
```
$ roscore
$ rosrun sensor_fusion_pkg sensor_streamer.py
$ rosrun sensor_fusion_pkg test_node_for_accel_mag.py
$ roslaunch phone_description phone_spawn.launch
$ roslaunch phone_description phone_rviz.launch

```
