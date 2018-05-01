# ROS packages dependencies
- ackermann_msgs
- rosserial, serial
- urg_node, urg_c, csm, laser_proc, pcl_ros
- amcl, map_server, robot_localization
- boost, armadillo
- nav_core move_base base_local_planner costmap_2d costmap_converter
- suitesparse, g2o
- gstreamer (only on Jetson)
```bash
$ sudo apt-get install ros-kinetic-ackermann-msgs
$ sudo apt-get install ros-kinetic-rosserial ros-kinetic-serial
$ sudo apt-get install ros-kinetic-urg-node ros-kinetic-urg-c ros-kinetic-csm ros-kinetic-laser-proc ros-kinetic-pcl-ros
$ sudo apt-get install ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-robot-localization
$ sudo apt-get install libboost-dev libarmadillo-dev
$ sudo apt-get install ros-kinetic-nav-core ros-kinetic-move-base ros-kinetic-base-local-planner ros-kinetic-costmap-2d ros-kinetic-costmap-converter
$ sudo apt-get install python-software-properties libsuitesparse-dev ros-kinetic-libg2o
$ sudo apt-get install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
```

## Installing packages with no installation candidate
On Jetson TX2, many packages may not have their installation candidates.  
This may be solved by
```bash
$ sudo add-apt-repository restricted
$ sudo add-apt-repository multiverse
$ sudo add-apt-repository universe
$ sudo apt-get update
```

## Getting Jetson TX2 to recognize ACM devices
By default, the Jetson TX2 may not detect USB devices that report as ttyACM. The kernel needs to be modified in order to detect ACM USB devices. [These scripts](http://www.jetsonhacks.com/2017/07/31/build-kernel-ttyacm-module-nvidia-jetson-tx2/) by JetsonHacks will build the kernel and modules on the Jetson TX2 that includes the ACM module.
```bash
$ git clone https://github.com/jetsonhacks/buildJetsonTX2Kernel.git
$ cd buildJetsonTX2Kernel
$ ./getKernelSources.sh
```
The script will open an editor on the kernel configuration file. Check the USB modem ACM support module to enable it (you can use ctrl+F and find ACM). Save the configuration file as /usr/src/kernel/kernel-4.4/.config when done editing.  
You can verify that it is enabled by checking that the line `CONFIG_USB_ACM=y` exists on the config file. (Instruction obtained from [here](https://github.com/datlife/jetson-car/issues/7))  
Build the kernel after that.
```bash
$ ./makeKernel.sh
$ ./copyImage.sh
```
After a reboot, the Jetson TX2 should be able to recognize ACM devices.