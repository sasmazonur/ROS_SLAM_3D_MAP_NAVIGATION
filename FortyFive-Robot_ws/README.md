## Project Description
With the rise of online shopping, it is becoming more and more important to be able to organize warehouses. Lost packages have become a large problem as employees have misplaced many packages due to the mistakes of poor organization.

This project is aiming to solve this problem by being able to track the location of packages. In order to do this, our robot will:
* Be able to travel and navigate a warehouse
* Create a virtual map of its surroundings.
* Detect and avoid moving obstacles such as people.
* Detect and recognize barcodes on packages.
* Log and keep track of the movements of packages. 

## Hardware
* TurtleBot 3 - Waffle
* WidowX Robot Arm Kit
* Intel® RealSense™ D415 Depth Camera

## Software Requirements
* Ubuntu 16.04
* ROS 1 Kinetic


#### File Structure
```
FortyFive-Robot_ws/        --WORKSPACE
  src/                   --Space for the user source code
    CMakeLists.txt       --This is symlinked to catkin/cmake/toplevel.cmake
    package_n/
      src                --Source Code Files
      msg                --Message Files
      srv                --Service Files
      CATKIN_IGNORE      --Optionally place this marker file to exclude package_n from being processed. Its file type (e.g. regular file, directory) and contents don't matter. It may even be a dangling symlink.
      CMakeLists.txt     --Configuration File
      package.xml        --Package Configuration File (Containing information about package)
  build/                 --BUILD SPACE(this is where build system is invoked, not necessarily within workspace)
    CATKIN_IGNORE        --Marking the folder to be ignored when crawling for packages (necessary when source space is in the root of the workspace, the file is emtpy)
  devel/                 --DEVEL SPACE (targets go here, parameterizable, but defaults to peer of Build Space)
    bin/
    etc/
    include/            
    lib/
    share/
    .catkin              --Marking the folder as a development space (the file contains a semicolon separated list of Source space paths)
    env.bash
    setup.bash
    setup.sh
    ...
  install/               --INSTALL SPACE (this is where installed targets for test installations go, not necessarily within workspace)
    bin/
    etc/
    include/
    lib/
    share/
    .catkin              --Marking the folder as an install space (the file is empty)
    env.bash
    setup.bash
    setup.sh
    ...
```
#### NO-Robot Installation Guide:
To install just simulation follow these steps:
* [1.1](#11-install-ubuntu-on-laptop) Install Ubuntu 16.04
* [1.2](#12-install-ros-on-laptop) Install ROS 1 Kinetic
* [1.3](#13-install-dependent-ros-packages) Dependencies
* [1.4.1](#141-ros-network-configuration) - Network Configuration ROS
* [3.1.1](#311-slam-simulation) - 3.1.4 - Run SLAM
* [4.1.2](#41-navigate-robot) - 4.2 - Navigate Robot
* [7.1](#71-install-visp-dependencies) Install Barcode Dependencies (VISP)


**Installation may take long time. Do NOT close the terminal. I recommend increasing RAM on the virtual machine.**

Following Instructions adapted and modified from TurtleBot3 Manual.
For more detailed explanation:
http://emanual.robotis.com/docs/en/platform/turtlebot3/overview

## 1 Software Installation
### 1.1 Install Ubuntu on Laptop
Download and install the **Ubuntu 16.04** on the Robot PC and Laptop from the following link.

https://www.ubuntu.com/download/alternative-downloads

### 1.2 Install ROS on Laptop
Run the following command in a terminal window.
```
$ sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
```

### 1.3 Install Dependent ROS Packages
```
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs
```

Initialize a Workspace Folder:

```$ cd ~
$ git clone https://github.com/CS45-FortyFive/FortyFive-Robot_ws.git
$ cd  ~/FortyFive-Robot_ws
$ catkin_make
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ echo "source ~/FortyFive-Robot_ws/devel/setup.bash" >> ~/.bashrc # Adds workspace to search path
$ source ~/.bashrc
```

### 1.4 Network Configuration
To reduce the time difference betweeen remote pc and robot, run following:

```
$ sudo apt-get install -y chrony ntpdate
$ sudo ntpdate -q ntp.ubuntu.com
```

### 1.4.1 ROS Network Configuration
ROS requires IP addresses in order to communicate between Robot PC and the Laptop.
##### 1.4.3 [Simulation]:
Run following commands to configurate simulation network.
###### Laptop:
```
$ echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
$ echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
$ echo "export ROS_IP=localhost" >> ~/.bashrc
$ echo "export FORTYFIVE_ROBOT_MODEL=waffle" >> ~/.bashrc

```
##### 1.4.2 Laptop - [Connect to Robot]:
ROS requires IP addresses in order to communicate between Robot PC and the Laptop. The Laptop and Robot PC should be connected to the same wifi router.
Run the following command in a terminal window on Laptop to find out the IP address.
```
ifconfig
```

###### Laptop Set ROS Network
```
$ echo "export ROS_MASTER_URI=IP_OF_YOUR_LAPTOP" >> ~/.bashrc
$ echo "export ROS_HOSTNAME=IP_OF_THE_ROBOT" >> ~/.bashrc
$ echo "export FORTYFIVE_ROBOT_MODEL=waffle" >> ~/.bashrc
```
###### Robot Set ROS Network
```
$ echo "export ROS_MASTER_URI=IP_OF_YOUR_LAPTOP" >> ~/.bashrc
$ echo "export ROS_HOSTNAME=IP_OF_YOUR_LAPTOP" >> ~/.bashrc
$ echo "export FORTYFIVE_ROBOT_MODEL=waffle" >> ~/.bashrc
```


When you are done, with this step run following:
```
source ~/.bashrc.
```

Make sure that your ROS environment set correctly.
```
declare -x ROSLISP_PACKAGE_DIRECTORIES="/home/username/FortyFive-Robot_ws/devel/share/common-lisp:"
declare -x ROS_DISTRO="kinetic"
declare -x ROS_ETC_DIR="/opt/ros/kinetic/etc/ros"
declare -x ROS_HOSTNAME="localhost"
declare -x ROS_IP="localhost"
declare -x ROS_MASTER_URI="http://localhost:11311"
declare -x ROS_PACKAGE_PATH="/home/bro/FortyFive-Robot_ws/src:/opt/ros/kinetic/share"
declare -x ROS_PYTHON_VERSION="2"
declare -x ROS_ROOT="/opt/ros/kinetic/share/ros"
declare -x ROS_VERSION="1"
```



### 2.1 Wake Up The Robot [Laptop]

### 2.1.1 For Simulation on Local computer [Laptop]
On your Laptop run following command on your terminal to simulate robot on Rviz.
```
$ roslaunch fortyfive_robot_fake fortyfive_robot_fake.launch
```
![Image of BringUp](https://github.com/sasmazonur/Capstone_Pictures_Gifs/blob/master/images_videos/bringup_robot.png)
### 2.1.2 Wake Up The Robot
##### Run following command on your terminal [Laptop]

```
roscore
```
roscore is the command that runs the ROS master.

##### Run following command on your terminal [Robot]
```
$ roslaunch fortyfive_robot_bringup fortyfive_robot_robot.launch
```

You Should see the following output on your terminal
```
SUMMARY
========

PARAMETERS
 * /rosdistro: kinetic
 * /rosversion: 1.12.13
 * /fortyfive_robot_core/baud: 115200
 * /fortyfive_robot_core/port: /dev/ttyACM0
 * /fortyfive_robot_core/tf_prefix:
 * /fortyfive_robot_lds/frame_id: base_scan
 * /fortyfive_robot_lds/port: /dev/ttyUSB0

NODES
  /
    fortyfive_robot_core (rosserial_python/serial_node.py)
    fortyfive_robot_diagnostics (fortyfive_robot_bringup/fortyfive_robot_diagnostics)
    fortyfive_robot_lds (hls_lfcd_lds_driver/hlds_laser_publisher)

ROS_MASTER_URI=http://localhost:11311

process[fortyfive_robot_core-1]: started with pid [14198]
process[fortyfive_robot_lds-2]: started with pid [14199]
process[fortyfive_robot_diagnostics-3]: started with pid [14200]
[INFO] [1531306690.947198]: ROS Serial Python Node
[INFO] [1531306691.000143]: Connecting to /dev/ttyACM0 at 115200 baud
[INFO] [1531306693.522019]: Note: publish buffer size is 1024 bytes
[INFO] [1531306693.525615]: Setup publisher on sensor_state [fortyfive_robot_msgs/SensorState]
[INFO] [1531306693.544159]: Setup publisher on version_info [fortyfive_robot_msgs/VersionInfo]
[INFO] [1531306693.620722]: Setup publisher on imu [sensor_msgs/Imu]
[INFO] [1531306693.642319]: Setup publisher on cmd_vel_rc100 [geometry_msgs/Twist]
[INFO] [1531306693.687786]: Setup publisher on odom [nav_msgs/Odometry]
[INFO] [1531306693.706260]: Setup publisher on joint_states [sensor_msgs/JointState]
[INFO] [1531306693.722754]: Setup publisher on battery_state [sensor_msgs/BatteryState]
[INFO] [1531306693.759059]: Setup publisher on magnetic_field [sensor_msgs/MagneticField]
[INFO] [1531306695.979057]: Setup publisher on /tf [tf/tfMessage]
[INFO] [1531306696.007135]: Note: subscribe buffer size is 1024 bytes
[INFO] [1531306696.009083]: Setup subscriber on cmd_vel [geometry_msgs/Twist]
[INFO] [1531306696.040047]: Setup subscriber on sound [fortyfive_robot_msgs/Sound]
[INFO] [1531306696.069571]: Setup subscriber on motor_power [std_msgs/Bool]
[INFO] [1531306696.096364]: Setup subscriber on reset [std_msgs/Empty]
[INFO] [1531306696.390979]: Setup TF on Odometry [odom]
[INFO] [1531306696.394314]: Setup TF on IMU [imu_link]
[INFO] [1531306696.397498]: Setup TF on MagneticField [mag_link]
[INFO] [1531306696.400537]: Setup TF on JointState [base_link]
[INFO] [1531306696.407813]: --------------------------
[INFO] [1531306696.411412]: Connected to OpenCR board!
[INFO] [1531306696.415140]: This core(v1.2.1) is compatible with TB3 Burger
[INFO] [1531306696.418398]: --------------------------
[INFO] [1531306696.421749]: Start Calibration of Gyro
[INFO] [1531306698.953226]: Calibration End
```

## 3.1 SLAM
* 3.1 - ROBOT
* 3.2 - Simulation


## 3.1.1 SLAM [Simulation]
Follow instructions under 3.2.x to run SLAM on the virtual environment.

### 3.1.2 Bringup Gazebo Environment [Simulation]
This is to simulate a world in Gazebo.
You can launch the Gazebo World:
```
$ export FORTYFIVE_ROBOT_MODEL=waffle
$ roslaunch fortyfive_robot_gazebo fortyfive_robot_world.launch
```
![Image of Gazebo World](https://github.com/sasmazonur/Capstone_Pictures_Gifs/blob/master/images_videos/gazebo_world.png)


or You can Launch Gazebo House:
```
$ export FORTYFIVE_ROBOT_MODEL=waffle
$ roslaunch fortyfive_robot_gazebo fortyfive_robot_house.launch
```
![Image of Gazebo House](https://github.com/sasmazonur/Capstone_Pictures_Gifs/blob/master/images_videos/gazebo_house.png)

### 3.1.2 Run SLAM Nodes [Simulation]
On your Laptop open a new terminal and run following commands.
```
$ export FORTYFIVE_ROBOT_MODEL=waffle
$ roslaunch fortyfive_robot_slam fortyfive_robot_slam.launch slam_methods:=gmapping
```
![Image of SLAM](https://github.com/sasmazonur/Capstone_Pictures_Gifs/blob/master/images_videos/mapping.png)

If you receive this error:
```
[fortyfive_robot_slam.launch] is neither a launch file in package [fortyfive_robot_slam] nor is [fortyfive_robot_slam] a launch file name
The traceback for the exception was written to the log file
```

Run Following command and try again:
```
source ~/FortyFive-Robot_ws/devel/setup.bash
```


### 3.1.3 Control Robot over Terminal [Simulation]
On new terminal and run following commands.
```
$ export FORTYFIVE_ROBOT_MODEL=waffle
$ roslaunch fortyfive_robot_teleop fortyfive_robot_teleop_key.launch
```
![Gif of SLAM](https://github.com/sasmazonur/Capstone_Pictures_Gifs/blob/master/images_videos/slam2.gif)
### 3.1.4 Save SLAM Map [Simulation]
On your Laptop run following command to save the map created by gmapping.

```
$ rosrun map_server map_saver -f ~/map
```

### 3.2.1 Run SLAM Nodes [Laptop]


On your Laptop open a new terminal and run following commands.
```
$ export FORTYFIVE_ROBOT_MODEL=waffle
$ roslaunch fortyfive_robot_slam fortyfive_robot_slam.launch slam_methods:=gmapping
```

### 3.2.2 Control Robot over Terminal [Laptop]
On your Laptop open a new terminal and run following commands.
```
$ export FORTYFIVE_ROBOT_MODEL=waffle
$ roslaunch fortyfive_robot_teleop fortyfive_robot_teleop_key.launch
```

You should see following output:
```
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop

CTRL-C to quit
```
![Gif of teleop](https://github.com/sasmazonur/Capstone_Pictures_Gifs/blob/master/images_videos/teleop.gif)

Now you can control the robot with your keyboard. Press W to move Forward. X to go backwards. A to turn left and D to turn right. Pressing S will stop the movement of the Robot.



### 3.2.3 Save SLAM Map [Laptop]
On your Laptop run following command to save the map created by gmapping.

```
$ rosrun map_server map_saver -f ~/map
```

## 4.1 Navigate Robot

### 4.1.1 Navigate Robot on the Created map [Robot]
On your Laptop run following command on your terminal
```
roscore
```

On robot PC run following command on your terminal.
```
$ roslaunch fortyfive_robot_bringup fortyfive_robot_robot.launch
```

### 4.1.2  Navigate Robot on the Created map [Simulation]
Bring the Gazebo World:
```
$ export FORTYFIVE_ROBOT_MODEL=waffle
$ roslaunch fortyfive_robot_gazebo fortyfive_robot_world.launch
```

### 4.2 Set Navigation Goal
On your Laptop run following command on your terminal:
```
$ export FORTYFIVE_ROBOT_MODEL=waffle
$ roslaunch fortyfive_robot_navigation fortyfive_robot_navigation.launch map_file:=$HOME/map.yaml
```

Press 2D Pose Estimate in the menu of RViz, a very large green arrow appears. This green arrow is a marker that can specify the destination of the robot. The root of the arrow is the x and y position of the robot, and the orientation pointed by the arrow is the theta direction of the robot. Click this arrow at the position where the robot will move, and drag it to set the orientation like the instruction below.

![Gif of Navigation Position](https://github.com/sasmazonur/Capstone_Pictures_Gifs/blob/master/images_videos/pos_estimate.gif)

Click the 2D Nav Goal button.
Click on a specific point in the map to set a goal position and drag the cursor to the direction where TurtleBot should be facing at the end.
The robot will create a path to avoid obstacles to its destination based on the map. Then, the robot moves along the path. At this time, even if an obstacle is suddenly detected, the robot moves to the target point avoiding the obstacle.

![Gif of Navigation](https://github.com/sasmazonur/Capstone_Pictures_Gifs/blob/master/images_videos/navigation.gif)

## 5 3-D Mapping with Intel® RealSense™ D410
### 5.1 Install RealSense ROS Package:

Install Prerequisites:
```
$ wget -O enable_kernel_sources.sh http://bit.ly/en_krnl_src
$ bash ./enable_kernel_sources.sh
```

Install Sensor Packages:
```
$ sudo apt install ros-kinetic-librealsense ros-kinetic-realsense-camera
$ sudo reboot
```

Install Kernel 4.10 work-around:
```
$ sudo apt-get install libglfw3-dev
$ cd ~
$ git clone https://github.com/IntelRealSense/librealsense.git
$ cd librealsense
$ mkdir build && cd build
$ cmake ../
$ make && sudo make install
$ cd ..
$ sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
$ sudo udevadm control –reload-rules && udevadm trigger
$ ./scripts/patch-realsense-ubuntu-xenial.sh

```

### 5.2 Install Intel® RealSense™ ROS from Sources
```
$ cd ~/FortyFive-Robot_ws/src/
```
Clone the latest Intel® RealSense™ ROS into 'FortyFive-Robot_ws/src/'
```
$ git clone https://github.com/IntelRealSense/realsense-ros.git
$ cd realsense-ros/
$ git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
$ cd ..
```

```
$ catkin_init_workspace
$ cd ..
$ catkin_make clean
$ catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
$ catkin_make install
$ echo "source ~/FortyFive-Robot_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### 5.2 Start Camera
On the Robot run following command:
```
$ roslaunch realsense2_camera rs_camera.launch
```

### 5.3 SLAM with Intel® RealSense™
##### Run the camera Node [Robot]:
```
$ roslaunch realsense2_camera rs_rgbd.launch
```

##### Bring Up the Robot [Robot]:
```
$ roslaunch fortyfive_robot_bringup fortyfive_robot_robot.launch
```
##### Run SLAM Nodes [Laptop]:

```
$ roslaunch fortyfive_robot_slam fortyfive_robot_slam.launch
```

##### Configurate the Camera Settings [Laptop]:
* Click `Panels -> Views` to open the view window
* Click `TopDownOrtho` and change it into `XYOrbit`
* Click `add - By topic` and find the `PointCloud2` type `/points` topic in `/camera/depth`
* Click `PointCloud2` type topic on the left window, then change Color Transformer from `Intensity` to `AxisColor`.
This will show the depth of each points by color description.
* Click `add - By topic` and find the Image type `/image_color` topic in `/camera/rgb`, then click it. This will show the view of the normal camera image


## 6 WidowX MKII Robot Arm
### 6.1 Assemble WidowX MKII Robot Arm
Following Instructions adapted from TurtleBot3 Manual. For more detailed explanation please visit:
https://widowx-arm.readthedocs.io/en/latest/index.html

Before installing arm dependencies, carefully assemble the robotic arm by following the directions in this link.
```
http://www.trossenrobotics.com/productdocs/assemblyguides/widowx-robot-arm-mk2.html
```
### 6.2 Install WidowX Arm Dependancies
```
$ sudo apt install git htop
$ sudo apt install ros-kinetic-moveit ros-kinetic-pcl-ros
```

Set dialout permission for Arbotix:
yourUserAccount is the name of your pc
```
$ sudo usermod -a -G dialout yourUserAccount
```
Then Restart your computer:
```
sudo reboot
```
Clone Widowx Arm repository and build:
```
$ mkdir -p ~/widowx_arm/src
$ cd ~/widowx_arm/src
$ git clone https://github.com/Interbotix/widowx_arm.git .
$ git clone https://github.com/Interbotix/arbotix_ros.git -b parallel_gripper
$ cd ~/widowx_arm
$ catkin_make
```

Test execution without additional sensors:
```
$ cd ~/widowx_arm
$ source devel/setup.bash
$ roslaunch widowx_arm_bringup arm_moveit.launch sim:=false sr300:=false```
```
### 6.3 Install ROS Arm Dependancies
On your Laptop run following commands on your terminal to download and build the OpenMANIPULATOR-X package.

```
$ cd ~/FortyFive-Robot_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
$ cd ~/FortyFive-Robot_ws && catkin_make
```

### 6.4 Operate the Arm
On your Laptop run following command on your terminal:
```
$ roscore
```
On the Robot PC run following command on your terminal:
```
$ roslaunch fortyfive_robot_bringup fortyfive_robot_robot.launch
```
On the Laptop run following command on your terminal:
```
$ roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```

Now you can control the Robot Arm, using GUI
```
$ roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```
or You can use Rviz

```
$ roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
```

### 7 Installing Barcode Reader Dependencies
In this section we will install `visp` package that corresponds to the `Open Source Visual Servoing Library ` packaged for ROS.

The algorithm will allow us to detect automatically the barcode using one of the following detectors:

* QR-code detection
* flashcode detection

You can find more aboit visp here:
https://visp.inria.fr

#### 7.1 Install visp Dependencies

Install visp Pre-Build Packages:
```
sudo apt-get install ros-kinetic-vision-visp
```

Install dependencies in to workspace.
```
$ cd ~/FortyFive-Robot_ws/src
$ git clone https://github.com/lagadic/vision_visp.git
$ cd vision_visp
$ git checkout kinetic
$ cd ~/FortyFive-Robot_ws
$ sudo rosdep init
$ rosdep update
$ rosdep install --from-paths src --ignore-src --rosdistro kinetic

$ cd ~/FortyFive-Robot_ws
$ catkin_make
```

You can test your installation:
```
$ roslaunch visp_tracker tutorial.launch
```

You should see this screen:
![Image of Barcode Test1](https://github.com/sasmazonur/Capstone_Pictures_Gifs/blob/master/images_videos/barcode1.png)

And after validation you should see:

![Image of Barcode Test2](https://github.com/sasmazonur/Capstone_Pictures_Gifs/blob/master/images_videos/barcode2.png)


You can test your installation on Pre-Recorded Video:

You should see the object position from QR Code
![Image of Barcode Test3](https://github.com/sasmazonur/Capstone_Pictures_Gifs/blob/master/images_videos/barcode3.png)

#### 7.2 Calibrate the Camera
For calibrating camera we will use `visp_camera_calibration`.
We will use the example images that come with this package. They are located in `launch/images`.

Run Following command on terminal to start calibration:

```
roslaunch visp_camera_calibration lagadic_grid.launch
```

![Gif of calibration](https://github.com/sasmazonur/Capstone_Pictures_Gifs/blob/master/images_videos/barcode_calib.gif)

Start clicking the numbered circles liked showed on the gif.
when you done with selecting 1 to 4. The image processing interface will try to detect the rest of the points for you.

### 8 3D RTAB-Map
#### 8.1 Install RTAB-Map Dependancies
To install RTAB-Map libraries:
```
$ sudo apt-get install ros-kinetic-rtabmap-ros
$ sudo apt-get install ros-kinetic-rtabmap ros-kinetic-rtabmap-ros
```
Install RTAB-Map standalone libraries:
```
$ cd ~
$ git clone https://github.com/introlab/rtabmap.git rtabmap
$ cd rtabmap/build
$ cmake ..  [<---double dots included]
$ make
$ sudo make install
```

Clone `vision_opencv`, `image_transport_plugins` and `rtabmap_ros` packages in workspace:
```
$ cd ~/FortyFive-Robot_ws
$ git clone https://github.com/ros-perception/vision_opencv src/vision_opencv
$ git clone https://github.com/ros-perception/image_transport_plugins.git src/image_transport_plugins
$ git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
$ catkin_make
```

### 8.2 RGB-D Handheld Mapping

Run the following command in a terminal window to start the depth camera.
```
$ roslaunch realsense2_camera rs_camera.launch align_depth:=true unite_imu_method:="linear_interpolation"
$ rosrun imu_filter_madgwick imu_filter_node \
    _use_mag:=false \
    _publish_tf:=false \
    _world_frame:="enu" \
    /imu/data_raw:=/camera/imu \
    /imu/data:=/rtabmap/imu
```

```
$ roslaunch rtabmap_ros rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false\
    rviz:=true
  ```
 rtabmap_args `"--delete_db_on_start" ` used for starting mapping from a clean database.

 The map will be saved to `~/.ros/rtabmap.db`
