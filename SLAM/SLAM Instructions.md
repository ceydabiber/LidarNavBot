## Prerequisites

Ensure you have the following installed on your system before proceeding:

- **ROS Melodic**:  Follow the installation guide for ROS Melodic suitable for your operating system.
- **RViz**: Install RViz with ROS Melodic. It's usually included with the ROS installation, but ensure it's installed using:

```powershell
sudo apt-get install ros-melodic-rviz
```

---

# Jetson Nano Setup

Follow these steps to set up the Jetson Nano:

1. **Setup Instructions:**
    - Refer to these guides for detailed setup:
        - [Jetson Nano Developer Kit Setup](https://automaticaddison.com/how-to-set-up-the-nvidia-jetson-nano-developer-kit/)
        - [Install ROS Melodic on Jetson Nano](https://automaticaddison.com/how-to-install-ros-melodic-on-the-nvidia-jetson-nano/)
2. **YDLidar 4x User Manual:**
    - [YDLidar X4 User Manual](https://www.ydlidar.com/Public/upload/files/2024-02-01/YDLIDAR%20X4%20Lidar%20User%20Manual%20V1.3(240124).pdf)

---

## LiDAR Setup

### Install YDLidar Package

1. Open a terminal and run the following commands:

```powershell
$ roscd
~/catkin_ws/devel$ cd ..
~/catkin_ws$ cd src
~/catkin_ws/src$ git clone https://github.com/EAIBOT/ydlidar.git
~/catkin_ws$ catkin_make
~/catkin_ws/src$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
~/catkin_ws/src$ source ~/.bashrc
~/catkin_ws/src$ cd ~/catkin_ws/src/ydlidar/startup
~/catkin_ws/src/ydlidar/startup$ sudo chmod +x initenv.sh
~/catkin_ws/src/ydlidar/startup$ sudo sh initenv.sh
~/catkin_ws/src/ydlidar/launch$ roslaunch ydlidar lidar_view.launch
```

### Troubleshooting

If you encounter the following error:

```
CMake Error at /opt/ros/melodic/share/catkin/cmake/empy.cmake:29 (message):
  Unable to find either executable 'empy' or Python module 'em'...
  try installing the package 'python-empy'
```

Run this command to solve the issue:

```powershell
$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

---

## 3. Launch File Setup

### Create the Launch File

1. Create a new launch file:

```powershell
~/catkin_ws/src/ydlidar/launch$ touch all_nodes.launch
```

2.  Add the following content to `all_nodes.launch`:

```powershell
<launch>
  <node name="ydlidar_node" pkg="ydlidar" type="ydlidar_node" output="screen" respawn="false">
    <param name="port" type="string" value="/dev/ydlidar"/>
    <param name="baudrate" type="int" value="128000"/>
    <param name="frame_id" type="string" value="laser_frame"/>
    <param name="low_exposure" type="bool" value="false"/>
    <param name="resolution_fixed" type="bool" value="true"/>
    <param name="auto_reconnect" type="bool" value="true"/>
    <param name="reversion" type="bool" value="false"/>
    <param name="angle_min" type="double" value="-180" />
    <param name="angle_max" type="double" value="180" />
    <param name="range_min" type="double" value="0.1" />
    <param name="range_max" type="double" value="16.0" />
    <param name="ignore_array" type="string" value="" />
    <param name="samp_rate" type="int" value="9"/>
    <param name="frequency" type="double" value="7"/>
  </node>
  <node pkg="tf" type="static_transform_publisher" name="map_to_odom" args="0.0 0.0 0.0 0.0 0.0 0.0 /map /nav 40"/>
  <node pkg="tf" type="static_transform_publisher" name="odom_to_base_link" args="0.0 0.0 0.0 0.0 0.0 0.0 /nav /base_footprint 40"/>
  <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser" args="0.2245 0.0 0.2 0.0 0.0 0.0 /base_footprint /laser_frame 40"/>
  <include file="$(find hector_mapping)/launch/mapping_default.launch" />
  <node pkg="rviz" name="rviz" type="rviz" args="-d $(find ydlidar)/launch/lidar.rviz" />
  <include file="$(find hector_geotiff)/launch/geotiff_mapper.launch" />
</launch>
```

1. Run the launch file:

```powershell
~/catkin_ws/src/ydlidar/launch$ roslaunch ydlidar all_nodes.launch
```

---

## 4. Hector SLAM Setup

### Install Hector SLAM and Dependencies

```powershell
~/catkin_ws$ sudo apt-get install ros-melodic-hector-slam
# If geotiff is missing, install it as well:
~/catkin_ws$ sudo apt-get install ros-melodic-hector-geotiff
```

### Verify Transform Frames

1. Launch the LiDAR view:
    
    ```
    ~/catkin_ws/src/ydlidar/launch$ roslaunch ydlidar lidar_view.launch
    ```
    
2. Check the transform frames:

```powershell
~/catkin_ws/src/ydlidar$ rosrun tf view_frames
```

1. Review the generated PDF to ensure that `base_footprint` is connected to `laser_frame` correctly. Verify the Broadcaster, average rate, most recent transform, and buffer length.

### Final Configuration

1. Create the `all_nodes.launch` file with the content specified in the Launch File Setup section above.
2. Save and close the file.
3. Launch the configuration:

```powershell
~/catkin_ws/src/ydlidar/launch$ roslaunch ydlidar all_nodes.launch
```

---

## 5. Mapping Methods

### First Method

1. Open a new terminal and run:
    
    ```bash
    rostopic pub syscommand std_msgs/String "savegeotiff"
    ```
    
2. When mapping is complete, close all terminals with `CTRL + C`.
3. Find the map file in:
    
    ```jsx
    ~/catkin_ws/src/hector_slam/hector_geotiff/maps
    ```
    
    The file will be named something like `hector_slam_map_##:##:##.tfw`.
    

### Second Method

1. Install the map server:
    
    ```bash
    sudo apt-get install ros-melodic-map-server
    ```
    
2. Create a directory for maps and save the map:
    
    ```bash
    mkdir ~/catkin_ws/maps
    cd ~/catkin_ws/maps
    rosrun map_server map_saver -f my_map
    ```
    
3. To view the map:
    
    ```bash
    cd ~/catkin_ws/maps
    roscore
    ```
    
4. In a new terminal, load the map:
    
    ```bash
    rosrun map_server map_server my_map.yaml
    ```
    
5. In another terminal, run RViz:
    
    ```bash
    rviz
    ```
    
6. In RViz, click the "Add" button in the lower left corner and add a "Map display." Select "/map" under the "Map" section's "Topic" heading.

### Convert Map to PNG

To convert the map to PNG format:

1. Install ImageMagick:
    
    ```bash
    sudo apt-get install imagemagick
    ```
    
2. Convert the map:
   ```bash
   convert my_map.pgm my_map.png
   ```
