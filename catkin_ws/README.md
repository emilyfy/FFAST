# ROS Catkin Workspace

This is the catkin workspace for the ROS packages. Packages are categorized into their functionalities such as hardware, localization, planning and user interface. Third-party packages that need to be installed on ROS and not included here are listed in the [dependencies document](../doc/dependencies.md). The third party packages included here have to be built from this source as the codes have been edited for the purpose of this project.

The source codes are divided into several subfolders:
- hardware  
  This is for all the packages responsible for interfacing with the hardware. This includes the VESC, VMU931, camera and GPIO on Jetson.
- localization  
  This is for the robot localization capability, contains the scan matching and optic flow packages.
- planning  
  This is for the motion planners, which include the ROS navigation stack as well as the planners we developed with iLQR.

All the above also contain their own metapackages that has the launch and config files for each of their purposes. There are some other individual packages that do not belong in any category:
- user_interface  
  Contains scripts for user interface purposes
- sysid  
  Scripts run for system identification experiments
- urdf
  URDF model of the car

And finally, the ffast metapackage contains the top level launch file and maps of the environment to be used by AMCL.