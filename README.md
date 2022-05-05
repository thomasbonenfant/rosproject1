# rosproject1
## Team members
10627267 - Veronica Cardigliano
- Thomas Jean Bernard Bonenfant
## Description of files
The package is composed of 5 subdirectories: 
*cfg contains parameters.cfg file with python code for dynamic reconfigure, to dynamically select the odometry integration method (Euler/RK)
*launch contains the .launch file to launch all the nodes and set parameters values
*msg with WheelsSpeeds.msg: the custom message with which vel_sub node publishes the computed wheels' speeds
*src with the nodes: 
	*wheels_sub subscribes to bag messages of type sensor_msgs/JointState as topic wheel_states, computes v and ⍵, starting from encoder ticks and model parameters, and publishes v and ⍵ as topic cmd_vel of type geometry_msgs/TwistStamped
	*vel_sub subscribes to cmd_vel topic of wheels_sub node, computes wheel speeds starting from v, ⍵ received and publishes the computed speeds as topic wheels_rpm using custom messages of type WheelsSpeeds 
	*odom_node subscribes to cmd_vel topic, computes the odometry both for Euler and Runge-Kutta integration methods, and publishes it on topic odom, as messages of type nav_msgs/Odometry
	*pose_sub, 
*srv contains the file Reset.srv that represents the service used to reset the odometry to any given pose (x, y, θ)
## Name and meaning of ROS parameters

## Structure of TF tree
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
│ 	├── bag1.bag
│	├── bag2.bag
│	├── bag3.bag
│	├── odom_node.cpp
│	├── pose_sub.cpp
│	├── vel_sub.cpp
│	└── wheels_sub.cpp
└── srv
    └── Reset.srv
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
The node can be easily started calling the launch file with the command _roslaunch rosproject1 pub_and_sub.launch_ 
## Other infos