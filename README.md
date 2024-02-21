# Athlete Rover 
The Athlete Rover is a ROS-based project designed to simulate and control a six-legged robot, the Athlete Rover. This project includes the Athlete Rover model designed in SolidWorks and a Python script for controlling the rover's motion.

## Getting Started
Before starting, ensure you have a ROS environment set up. If not, please follow the ROS installation instructions on the ROS Wiki.

## Prerequisites
- Python 3
- ROS (version used during development: ROS Noetic)
- rospy - Python library for ROS
- std_msgs - Standard ROS messages
- Gazebo

## Installation

- Clone the repository into your ROS workspace:
```
cd ~/catkin_ws/src/
git clone https://github.com/tarunreddyy/Athlete-Rover-Simulation.git
```

- Navigate back to your ROS workspace and build the project:
```
cd ~/catkin_ws/
catkin_make
```

- Source your workspace:
```
source ~/catkin_ws/devel/setup.bash
```

- Launching the Athlete Rover in Gazebo
To load the Athlete Rover model in Gazebo simulator, use the provided launch file:
```
roslaunch athlete_rover athlete_rover.launch
```
This will launch Gazebo with the Athlete Rover model loaded in.

Athlete Rover in Gazebo

[Athlete Rover in Action](https://youtu.be/uFLJR89Jcns "Athlete Rover in Action")

- Running the script

To run the script, navigate to the scripts directory and run:
```
rosrun athlete_rover test.py
```

## Usage

The test.py script allows you to control the joint angles of each of the Athlete Rover's legs.

The script is currently set to control each joint manually using keyboard inputs. Each key corresponds to a specific action:

'u' - Stand
'h' - Home
'p' - Store
'a' - Roll left
'd' - Roll right
's' - Stop rolling
'g' - Initiate walking stance
'v' - Walk
'w' - Move forward
'x' - Reverse
'c' - Terminate the script
The script also includes functions for inverse kinematics, which allows the user to specify a desired pose, and the IK solver calculates the necessary joint angles to achieve this pose.

## Rover Design
The design of the Athlete Rover has been made in SolidWorks. The SolidWorks files for the rover design are included in the repository and can be accessed for further improvements and modifications.
![Screenshot 2024-02-11 012736](https://github.com/HarshShirsath/Robot-Modelling-Project-2-NASA-Athlete-Rover-/assets/113379668/6845b8d6-7bf6-464d-9f9c-157e0e7ba32c)



## Future Work
The goal is to enhance this project by integrating inverse kinematics to have a sophisticated control over the robot's motion.
