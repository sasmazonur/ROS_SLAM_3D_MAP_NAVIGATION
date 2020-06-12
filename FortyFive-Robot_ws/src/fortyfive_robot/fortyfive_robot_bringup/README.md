This folder contains the roslaunch scripts for starting the Robot.

* geometry_msgs/Twist -Control the translational and rotational speed of the robot unit in m/s, rad/s
* diagnostic_msgs/DiagnosticArray - Contains self diagnostic information

* sensor_msgs/Imu - attitude of the robot based on the acceleration and gyro sensor
* sensor_msgs/JointState - state of a set of torque controlled joints
* sensor_msgs/LaserScan - scan values of the LiDAR
* nav_msgs/Odometry - odometry information based on the encoder and IMU
* fortyfive_robot_msgs/SensorState -  contains the values of the sensors mounted on Robot.
* tf2_msgs/tfMessage - coordinate transformation such as base_footprint and odom


More Information can be found at: http://wiki.ros.org/turtlebot3_bringup
