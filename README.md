# tracking_controller
tune performance with difference velocity

## I. Installation Guide


## II. Run Controller DEMO
For tracking a circular trajectory, please run the following commands. Note that the tracking circle function is a part of [autonomous_flight](https://github.com/Zhefan-Xu/autonomous_flight) package and please install it before running. Also, for simulator, you can either use the original PX4 simulator or use our PX4-based [uav_simulator](https://github.com/Zhefan-Xu/uav_simulator) package:
```
# start PX4 simulator
roslaunch uav_simulator px4_start.launch 

# launch the tracking node function with the tracking controller
roslaunch autonomous_flight takeoff_and_track_circle.launch
```
The example results of running the above command can be visulized as below:

https://github.com/Zhefan-Xu/tracking_controller/assets/55560905/5a83ede2-a8a2-4c7a-a5f4-3dd3304e9ad0

