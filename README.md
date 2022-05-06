# rosproject1
## Team members
10627267  Veronica Cardigliano

10597564 Thomas Jean Bernard Bonenfant
## Files Description
The package is composed of 5 subdirectories: 
- `cfg` contains the `parameters.cfg` python script for dynamic reconfigure, to dynamically select the odometry integration method (Euler/RK) and `calibration.cfg` to manually change the robot's parameter during runtime
- `launch` contains the `.launch` file to launch all the nodes and set parameters values
- `msg` with `WheelsSpeeds.msg`: the custom message with which `vel_sub` node publishes the computed wheels' speeds
- `src` with the nodes: 
	- `wheels_sub` which subscribes to bag messages of type `sensor_msgs/JointState` as topic `wheel_states`, computes `v` and `⍵`, starting from encoder ticks and model parameters, and publishes `v` and `⍵` as topic `cmd_vel` of type `geometry_msgs/TwistStamped`
	- `vel_sub` subscribes to `cmd_vel` topic of `wheels_sub` node, computes wheel speeds starting from `v`, `⍵` received and publishes the computed speeds as topic `wheels_rpm` using custom messages of type `WheelsSpeeds`
	- `odom_node` subscribes to the `cmd_vel` topic, computes the odometry both with Euler and Runge-Kutta integration methods, and publishes it on topic `odom`, as messages of type `nav_msgs/Odometry`. It also broadcasts the transformation from frame `odom` to frame `base_link`.
- `srv` contains the file Reset.srv that represents the service used to reset the odometry to any given pose `(x, y, θ)`
## Name and meaning of ROS parameters

|name|description             |
|----|------------------------|
|`x,y,theta`| initial position in the odometry frame|
|
## Structure of TF tree
```
├── cfg
│ 	└── parameters.cfg
├── CMakeLists.txt
├── launch
│	└── pub_and_sub.launch
├── msg
│	└── WheelsSpeeds.msg
├── package.xml
├── README.md
├── src
│	├── odom_node.cpp
│	├── pose_sub.cpp
│	├── vel_sub.cpp
│	└── wheels_sub.cpp
└── srv
    └── Reset.srv
```
## Structure of custom message
The custom message WheelsSpeeds.msg has the structure shown in project presentation slides:
```
Header header
float64 rpm_fl
float64 rpm_fr
float64 rpm_rr
float64 rpm_rl
```
## How to start/use the nodes
The nodes can be easily started calling the launch file with the command `roslaunch rosproject1 pub_and_sub.launch` from the `launch` directory. This launch file will start 
## Other infos
### Calibration
We used meaningful time points in bag1 and bag2 to evaluate errors due to the robot's parameters:
- `radius`: radius of the wheels
- `N`: ticks count per revolution (encoder)
- `l`: wheel's position along the forward axis
- `w`: wheel's position along the lateral axis

Tools used during calibration:
- `rviz` to visualize in the 2D space the robot and the ground truth pose
- `plotjuggler` to plot each component of the computed pose and the ground truth
- `rqt_servicecaller` to call the reset service in order to avoid restarting each time ROS
- `rqt_reconfigure` to manually tune the parameters

We used bag1 to calibrate `radius`:
With the default parameter the robot was too slow compared to ground truth data therefore we tried to increase it getting better results.

We used bag2 to calibrate `l`:
Thanks to the first rotation we noticed the robot was not rotating enough compared to the ground truth therefore we lowered a bit the parameter and got better results.

| parameter | before calibration | after calibration|
|--|--|--|
|`radius`| 0.07 | 0.076|
|`l`|0.2|0.185|

The updated parameters have been written in the `calibration.cfg` file as default values.

The other parameters were left unchanged because no additional improvement was detected.


