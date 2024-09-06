## Prerequisites

Ensure you have the following installed on your system before proceeding:

- **ROS Noetic**: The Robot Operating System (ROS) Noetic Ninjemys is required for this setup.
- **Gazebo**: A robotics simulator that integrates with ROS for testing and simulation.

## Installation Steps

### 1. Install Gazebo Packages

Open a terminal and run the following commands to install the necessary Gazebo packages:

```jsx
sudo apt-get install ros-noetic-gazebo-ros-pkgs
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-gazebo-msgs
sudo apt-get install ros-noetic-gazebo-plugins
sudo apt-get install ros-noetic-gazebo-teleop-twist-keyboard
```

### 2. Create and Set Up Your ROS Workspace

Create a new workspace for your ROS project:

```jsx
mkdir -p ~/robot/src
cd ~/robot
catkin_make
source ~/robot/devel/setup.bash

```

Verify the ROS package path:

```jsx
echo $ROS PACKAGE PATH
```

### 3. Create a ROS Package

Create a new ROS package for your robot model:

```jsx
cd ~/robot/src
catkin_create_pkg robot_model_pkg gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_ gazebo_ros_control
```

## 4. Create Source Files

NOTE: You can find the necessary files in robot_model_pkg folder 

Navigate to your package directory and create the necessary folders:

```bash
cd ~/robot/src/robot_model_pkg
mkdir urdf
cd urdf
```

Create and edit the URDF files for your robot:

- **robot.xacro**: Add the URDF configuration here.
- **robot.gazebo**: Add Gazebo-specific configurations here.

Use a text editor (like `gedit`) to create these files:

```powershell
gedit robot.xacro
```

*(Add the content for robot.xacro, save, and close)*

```bash
gedit robot.gazbo
```

*(Add the content for robot.gazebo, save, and close)*

## 5. Create the Launch File

Create a directory for launch files and add the launch file:

```bash
cd ~/robot/src/robot_model_pkg
mkdir launch
cd launch
```

Create and edit the launch file:

```bash
gedit robot_xacro.launch
```

*(Add the content for robot_xacro.launch, save, and close)*

## 6. Build the Workspace

Build your workspace to include the new package:

```bash
cd ~/robot
catkin_make
```

## 7. Start the Simulation

Open a new terminal and start the ROS core:

```bash
roscore
```

In a separate terminal, launch the robot model in Gazebo:

```bash
roslaunch robot_model_pkg robot_xacro.launch
```

In another terminal, control the robot using the keyboard:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
