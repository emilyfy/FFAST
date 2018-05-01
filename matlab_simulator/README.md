# MATLAB Simulator

A dynamics simulation of drifting in automobile. Parameters currently used refer to a 1/10 scale RC car, but the model is expected to work with full scale vehicles.

Subfolders:
- DDP-Generator  
  MATLAB scripts from [jgeisler0303's repository](https://github.com/jgeisler0303/DDP-Generator) that generates C codes for DDP computation according to problem definitions written in Maxima language.  
  The drift cornering and evasive maneuver problem definitions can be found here.
- utils  
  MATLAB utility functions, for dynamics simulation, visualization and plotting.

Contents of main folder:
- joy_sim.m  
  Simulates vehicle behavior from controller input.
- ros_sim.m  
  Simulates vehicle behavior from controller input and visualizes it together with actual vehicle response by subscribing to ROS topics.
- ramp_steer.m  
  Simulates vehicle response in ramp steer experiment.
- sim_const_input.m  
  Simulates vehicle response under constant control inputs.
  
- testObs.m  
  Evasive maneuver test experiment.
- testObs_sim.m  
  Evasive maneuver full simulation of 100 experiments.
- testObs_ros.m  
  Evasive maneuver test experiment with simplified problem definition (for ROS).
- testObs_ros_sim.m  
  Evasive maneuver with simplified problem definition full simulation.
- drift_sim.m  
  Finds a drift cornering equilibrium and simulates steady state vehicle behavior under constant control input.
- testiLQGDriftCorneringSteadyState.m  
  Steady state drift cornering test experiment with MATLAB function.
- testDriftCorneringSteadyState.m  
  Steady state drift cornering test experiment with C implementation.
- testDriftCorneringSteadyState_sim.m  
  Steady state drift cornering full simulation.
- updatevars.m  
  Common script to be used in evasive maneuver experiments.