# FFAST

## Introduction

The FFAST vehicle is a 1:10 scale RC car functioning as a test platform for testing dynamic motion planning and control algorithms. This repository contains the software developed throughout the project, including the source codes in the vehicle responsible for autonomous driving and executing maneuvers such as drifting and obstacle avoidance.

## Functionalities

The rear-wheel drive front steering vehicle is capable of driving with speed of up to 6 m/s forward and 3 m/s backwards and achieving steering angles of 30 degrees on either side. It achieves localization reasonably well using data from the Hall effect sensors on the motor as odometry, IMU, LIDAR and the camera on the Jetson developer kit with optic flow. It can be teleoperated or controlled by a software giving it navigation commands. An RF receiver on the vehicle stops vehicle operation when the remote transmitter is pressed, acting as an emergency kill switch.

## Organization

The repository is divided into several subfolders:
- doc  
Directory for instructions for developing the RC car platform and the detailed documentation of the project
- catkin_ws  
Catkin workspace containing the ROS packages that control the vehicle
- matlab_simulator  
MATLAB scripts for simulation of vehicle dynamics and execution of the maneuvers in simulation

## Getting Started

The [setup instructions](doc/setup_instructions.md) will get you a copy of the project up and running on your local machine for development and testing purposes. It has been developed for Linux, and no cross-compatibility effort has been made.

## Running the tests
Refer to the [test runs](doc/test_runs.md) for the commands to run the tests.

## Authors

* **Jordan Ford** - [jsford](https://github.com/jsford)
* **Emily Yunan** - [emilyfy](https://github.com/emilyfy)

See also the list of [contributors](https://github.com/jsford/FFAST/contributors) who participated in this project.

## License

MIT License  
Copyright (C) 2017 Jordan Ford  
The Robotics Institute, Carnegie Mellon University

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## Acknowledgments

* Carnegie Mellon University 
* General Motors Research and Development
