# Fetch Navigation for UMMA

## 1. Prerequisites

### 1.1 **Ubuntu**
Ubuntu 64-bit 16.04.

### 1.2 **ROS**
ROS Kinetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.3 **YOLO ROS**
YOLO ROS: Real-Time Object Dectection for ROS. [YOLO ROS installation](https://github.com/leggedrobotics/darknet_ros)

After installation, go to the [ros.yaml] file in [config] folder,
```
roscd darknet_ros/config
```
To make YOLO work with Fetch robot, we need to modify the camera topic under [Subscribers] [camera_reading] to [/head_camera/rgb/image_raw].


### 1.4 **PCL**
Point Cloud Library(PCL). [PCL installation](http://pointclouds.org/downloads/)

### 1.5 **ROS Packages**
Slam-karto
```
sudo apt-get install ros-kinetic-slam-karto
```

Navigation
```
sudo apt-get install ros-kinetic-navigation
```

Pointcloud to laserscan
```
sudo apt-get install ros-kinetic-pointcloud-to-laserscan
```

Laser filters
```
sudo apt-get install ros-kinetic-laser-filters
```

## 2. Build umma_navigation
Clone the repository and catkin_make:

```
cd [Target_Dir]
catkin_make
source /devel/setup.bash
```

## 3. Run the code
Launch the gazebo simulation with the umma building 

Run the launch file as:
```
roslaunch umma_navigation gazebo_launch.launch
```
Run YOLO first to recognize people with the RGB-D camera on Fetch. Extract the pointcloud and publish a marker for each person. Project the pointcloud on the ground as a laser_scan topic. Filter the scan topics to clear the map with each update.

```
roslaunch umma_navigation umma.launch
```

Run amcl to localize the robot. Generate the global and local costmaps with move_base. If the navigation for the map doesn't work proper, go to [/umma_navigation/params/amcl.yaml] file, and modify the [recovery_alpha_slow] to 0.05 and [recovery_alpha_fast] to 0.5. After the robot localize itself to the right place on the map, turn them back to 0. 

```
roslaunch umma_navigation costmap_test.launch
```

## 4. Visualize with Rviz
```
roslaunch umma_navigation umma_rviz.launch 
```

## 5. Set a Navigation Goal
Once the costmaps are generated and the robot localize itself, click the [2D Nav Goal] button in Rviz and click on the position on the mapto set the navigation goal position and goal orientation for robot.