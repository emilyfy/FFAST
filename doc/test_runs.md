# Test Runs

After powering up the Jetson and the VESC with the on-board battery and setting up the necessary remote network connections, the car is ready for running.

## Fundamental launches
```bash
$ roslaunch hardware hardware.launch
```
This starts all the hardware operations, including the VESC, IMU, LIDAR, camera, the corresponding static transforms broadcaster and the emergency kill switch listener on the GPIO. At this point, only odometry data and transform is published.  
The launch arguments are:
- predict_slip  
  Set to true to predict wheel slip. Displacement is underestimated during acceleration and high speed and overestimated during deceleration or braking.
- fuse_odom  
  Set to true to fuse the IMU data with motor sensors data before publishing in odom frame. The yaw will then always be obtained from the IMU.
- odom_tf  
  Set to true to broadcast odom to base_link transform.
```bash
$ roslaunch localization localization.launch
```
This starts the localization process, including scan matching, optic flow calculation, particle filter localization within the map and EKF that fuses all these data and output the final pose estimate.
The launch arguments are:
- fuse_odom  
  Same argument needs to be passed as the one to hardware.launch.
- use_scan_matcher  
  Set to true to perform scan matching on LIDAR scans and include the output in the EKF.
- use_amcl  
  Set to true to perform particle filter localization on the LIDAR scans and include the output in the EKF. If true, a map must be provided.
- map  
  The name of the map. The map files (pgm and yaml) must be placed inside the map folder inside the ffast package.

## Teleoperation of the car
In the local computer with a controller connected,
```bash
$ roslaunch user_interface joy.launch
```
Refer to the [schematic](controller.jpg) for details.

## Navigation to Goal
```bash
$ roslaunch ffast ffast.launch
```
or
```bash
$ roslaunch hardware hardware.launch
$ roslaunch localization localization.launch
$ roslaunch planning move_base.launch
```
Setting the goal on rviz will command the car to navigate towards the goal.

## Evasive Maneuver
After launching the hardware and localization files, the evasive maneuver can be run with
```bash
$ roslaunch planning evasive_maneuver.launch
```
This by default sets the car to run towards a goal 5 m ahead.
Launch arguments:
- obs_detect  
  Set to true to detect the presence of obstacles from the LIDAR scans. When false, virtual obstacles are introduced.
- obs_speed  
  Set to the speed of the obstacle (from 1 to 4, in m/s). Value of 5 introduces multiple obstacles one after another.

## Drift Cornering
Note that this maneuver has not been successfully performed yet.  
The drift cornering maneuver can similarly be run after launching the hardware and localization files with
```bash
$ roslaunch planning drift_cornering.launch
```