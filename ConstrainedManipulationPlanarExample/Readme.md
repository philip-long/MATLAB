# ConstrainedManipulationPlanarExample
This folder contains all files needed to reproduce the planar robot results using the constrained
manipulability polytope. Note in order to run this code you will need the [MPT3 library](https://www.mpt3.org/)

## Matlab_files
Contains the MATLAB files to generate the manipulability polytopes, analyze the workspace etc.

## ROS_files
Ros files to launch the simple planar system for visualization.

  1. Create a empty rospackage [constrained_manipulation]
  2. Copy launch and urdf folders into package, to launch simple example:

```
roslaunch constrained_manipulation  planar_humanoids_2018.launch 
```

## 