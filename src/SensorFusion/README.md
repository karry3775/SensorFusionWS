# SensorFusion

## Cloning the package
- create an empty workspace in a local directory
```
$ mkdir -p ~/sensor_fusion_ws/src 
$ cd sensor_fusion_ws
$ catkin_make
$ cd src
$ git clone https://github.com/karry3775/SensorFusion.git
```

## Using the package
- Connect your phone and computer to the same network (Hotspot can be used)
- Enter ip address:5000 to the URL field in PhonePi app
- ip addess could be find using ifconfig command on linux and ipconfig on windows. 
- run the following commands
```
$ roscore
$ rosrun sensor_fusion_pkg sensor_streamer.py
```
- if the above command gives error then move to the location of this file
```
$ cd SensorFusion/src/
$ chmod +x sensor_streamer.py
```
- The above line will make this file executable
- Activate the sensor readings you want on the PhonePi App
- Current implementation only supports Accelerometer, Gyroscope, Magnetometer and Orientation data
- You will see data printed on the terminal from which the following command was run
```
$ rosrun sensor_fusion_pkg sensor_streamer.py
```
- Open another terminal and type
```
rostopic echo \<topic_name>
```
- This will stream data for the specific topic you want

## Topics
- Gyro_topic - Publishes angular rate information from Gyro (TimeStamp, roll rate, pitch rate, yaw rate) (BODY FRAME)
- Accel_topic - Publishes linear acceleration information from Accelerometer (TimeStamp, a<sub>x</sub> , a<sub>y</sub>, a<sub>z</sub>) (BODY FRAME)
- Magneto_topic - Publishes magnetic field information from Magnetometer (TimeStamp, m<sub>x</sub> , m<sub>y</sub>, m<sub>z</sub>) (BODY FRAME)
- Orientation_topic - Publishes some fused orientation provided by the PhonePi App (TimeStamp, roll, pitch, yaw) (GLOBAL FRAME)

## Pushing and pulling from the Repo
- Move to your local directory
```
$ git status
```
- Above command will let you know the status of the repo and your local directory
```
$ git pull origin master
```
- Above command will make sure that your local directory will update with changes that others might have made
```
$ git add .
$ git commit -m "Descriptory message explaining the nature of the commit"
$ git push origin master
```
- Above command will update the github repository with the changes you made in the local directory in your computer
- Follow these steps sequentially to prevent improper syncing!
