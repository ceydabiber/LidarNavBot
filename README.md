## Table of Contents

1. [Description](#Description)
   - [Key Features](#key-features)
2. [System Overview](#system-overview)
3. [Mechanical Design](#mechanical-design)
   - [Technical Drawings and 3D Models](#technical-drawings-and-3d-models)
   - [Static Force Analysis](#static-force-analysis)
4. [Differential Drive](#differential-drive)
5. [Electrical System](#electrical-system)
6. [Software](#software)
   - [Obstacle Avoidance](#obstacle-avoidance)
   - [SLAM](#slam)
7. [Documentation](#documentation)

# Description

Our project features a 4-wheeled, multi-purpose autonomous mobile robot designed to explore and monitor unknown environments. Equipped with advanced navigation and mapping capabilities, the robot autonomously navigates and avoids obstacles while accurately mapping its surroundings.

## Key Features

- **Obstacle Avoidance**: Utilizing ultrasonic radar sensors, the robot detects and avoids obstacles, ensuring smooth and safe operation.
- **Environmental Mapping**: Through LiDAR-based SLAM (Simultaneous Localization and Mapping), the robot creates detailed maps of its environment.

# System Overview

Our 4-wheeled autonomous robot is designed to explore and monitor unknown environments with advanced navigation and mapping capabilities. The robot is built with a combination of sensors, cameras, and controllers that work together to ensure efficient operation.

### Components and Interaction

- **YDLidar X2**: This LiDAR sensor provides accurate environmental mapping by scanning and generating detailed maps of the surroundings. It plays a crucial role in the SLAM (Simultaneous Localization and Mapping) system.
- **HC-SR04 Ultrasonic Sensors**: These sensors are used for obstacle avoidance. They measure the distance to nearby objects and help the robot navigate around obstacles to ensure smooth and safe movement.
- **ESP-32 CAM**: This camera module captures images of the environment, which can be used for visual processing and analysis. It supports wireless communication for easy data transfer.
- **Arduino Uno**: Acts as the main microcontroller that handles sensor data processing and basic control tasks. It manages the inputs from the ultrasonic sensors and coordinates with the Jetson Nano.
- **Jetson Nano**: This powerful computing unit is responsible for running complex algorithms, including SLAM and real-time processing of sensor data. It interfaces with the Arduino Uno to receive data and make high-level decisions.
- **Power Supply**:
    - **9V Batteries**: Power the sensors and other auxiliary components.
    - **15V LiPo Battery**: Provides power to the motors, ensuring the robot can move effectively.

# Mechanical Design

This section presents the technical drawings, 3D models, and an image of the assembled robot.

### Technical Drawings and 3D Models

- The technical drawings for the robot’s mechanical parts are provided for reference
- The 3D models of the robot's components, including the full assembly, can be found in the provided CAD files folder (.stl fils are provided as well). These files can be used for 3D printing or further analysis.
- Below is an image of the completed robot assembly.

## Static Force Analysis

This section details the results and evaluations from the static force analysis applied to the body of our autonomous mobile robot. The analysis was conducted using ANSYS software to ensure the robot can withstand various forces that might be encountered during operation, particularly from falling objects.

### Analysis Parameters

The finite element analysis (FEA) was performed under several simulated scenarios to replicate conditions the robot might face. The parameters for these simulations included:

- **Brick**: 20-80 N
- **Wood Beam**: 50-200 N
- **Door Beam**: 130-350 N
- **Stone**: 40-150 N
- **Tree Branch**: 30-120 N

These scenarios were created based on estimated weights and drop heights of objects that could impact the robot’s body.

### Analysis Process

1. **Model Creation**: A 3D model of the robot body was developed using SolidWorks and imported into ANSYS.
2. **Material Properties**: Material properties of the robot body were defined within ANSYS.
3. **Meshing**: The model was meshed using the finite element method to prepare for analysis.
4. **Force Application**: Fixed boundary conditions were applied to the surfaces in contact with the ground, and force ranges were applied to simulate impact scenarios.
5. **Static Analysis**: A static analysis was performed to assess the deformation and stress distribution under the applied forces.

### Results and Evaluation

The analysis provided insights into the robot’s structural integrity under various force conditions:

- **Deformation**: The total deformation values were measured for minimum and maximum applied forces. The highest deformation occurred at the points of force application, indicating areas that are most affected by impacts.
- **Von Mises Stress**: The von Mises stress distribution was evaluated to identify critical stress regions. The material properties remained within acceptable limits, ensuring the structural integrity of the robot body.

**Results Summary**:

- **Deformation**: The robot body demonstrated acceptable levels of deformation, even under the highest expected forces.
- **Stress Distribution**: The von Mises stress analysis showed that the stress levels were within safe limits, indicating the robot body can safely operate under challenging conditions.

Overall, the static force analysis confirms that the robot body is robust enough to handle the specified force ranges, ensuring reliable performance in real-world scenarios.

### Differential Drive

### Differential Drive Simulation with ROS and Gazebo

To test and refine the differential drive system, we created a simulation environment using **ROS (Robot Operating System)** and **Gazebo**. This simulation allowed us to develop and evaluate control algorithms and sensor integration without involving physical testing.

### Simulation Setup

- **Model Creation**: We designed a URDF (Unified Robot Description Format) model that closely represents the real robot, including:
    - A rectangular chassis
    - Four wheels
    - A camera
    - A 2D LiDAR sensor
- **Motion Control**: Gazebo plugins simulated the differential drive mechanism, enabling the robot to move forward, backward, and turn by adjusting wheel speeds.
- **Sensor Simulation**: The simulation included camera and LiDAR data to test navigation and mapping algorithms.

### Benefits of Simulation

- The **ROS-Gazebo** environment provided a controlled, risk-free setting to fine-tune our algorithms and assess the differential drive system’s performance.
- This simulation was a crucial step in evaluating the system before considering physical implementation.

*Note:  While the simulation provided crucial insights, further work is needed to fully integrate the differential drive and sensor control algorithms into the real-world robot. Detailed instructions for running the simulation can be found in the ROS-Gazebo Instructions.*


# Electrical System

The robot’s electrical system integrates sensors, cameras, and controllers to ensure efficient operation. This includes components like the **HC-SR04 ultrasonic sensors**, **ESP-32 CAM**, **Arduino Uno**, and **Jetson Nano**, all powered by dedicated battery systems.

For detailed information on the wiring and interconnections, as well as any necessary circuitry, please refer to the **Circuit Diagram** and the accompanying **circuit file** provided in the repository.

# Software

## Obstacle Avoidance

Obstacle avoidance is a crucial part of the robot's autonomous navigation system, allowing it to safely explore unknown environments. We designed and implemented an algorithm that utilizes an **ultrasonic sensor** to detect obstacles and guide the robot's movement.

### Algorithm Overview

- **Detection Mechanism**: The ultrasonic sensor continuously emits high-frequency sound pulses and measures the time it takes for them to return. This provides accurate distance data, which is used to detect obstacles in real-time.
- **Threshold**: If an obstacle is detected within **16 inches**, the robot reacts by following a series of predefined movements to avoid a collision:
    1. **Move Backward**: The robot reverses for two seconds to create space.
    2. **Random Turn**: It randomly turns left or right for three seconds.
    3. **Resume Forward Motion**: After avoiding the obstacle, the robot continues moving forward.

This simple yet effective approach allows the robot to explore autonomously while avoiding obstacles.

## SLAM

Our SLAM (Simultaneous Localization and Mapping) system uses the **Hector SLAM** algorithm in conjunction with a **YDLidar X2** sensor to achieve high-precision mapping and localization. This approach is well-suited for real-time applications such as autonomous navigation and exploration.

### SLAM System Overview

- **Algorithm**: We employed the Hector SLAM algorithm, which is known for its accuracy and efficiency in creating detailed 2D maps. This algorithm processes data from the LiDAR to generate precise grid maps while simultaneously localizing the sensor within the map.
- **LiDAR Sensor**: The **YDLidar X2** is used for scanning the environment. It emits laser beams in multiple directions to measure distances, which are used to build a detailed 2D map of the surroundings. The sensor provides 360-degree scanning capability and high-precision distance measurements, making it ideal for dynamic environments.
- **Integration**: The LiDAR data is continuously processed by Hector SLAM, which does not require odometry or IMU data, simplifying the setup and reducing drift.

### Implementation Steps

1. **Sensor Setup**: Mount the YDLidar X2 on the robot to ensure a full 360-degree scan of the environment.
2. **Mapping**: Use Hector SLAM to collect and process LiDAR data, creating a 2D grid map.
3. **Localization**: The algorithm uses the map to estimate the robot's position in real-time.

### Testing and Results

We conducted mapping tests in various environments, including both indoor and outdoor spaces. These tests validated the SLAM system's effectiveness in different scenarios:

- **Indoor Spaces**: The system provided detailed maps and accurate localization in confined environments.

- **Outdoor Areas**: As excepted the SLAM didn’t perform quite well in large open spaces.

For detailed steps on LiDAR setup and map saving, refer to the **SLAM** folderin the repository.
As for all the maps obtained by the robot you may refer to the Results foder in the repository

## Documentation

For a more detailed understanding of the project, including the full report and presentation slides, please refer to the docs folder:

- **Project Report**: A comprehensive report covering all aspects of the project.
- **Presentation Slides**: The slides used for the project presentation.
