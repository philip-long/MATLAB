# MATLAB

1. This folder contains most of the matlab functions during that are reusable and public
2. Note I've also collected user functions from stack/matlab that have been proved useful I've left the header and author information


## Generic Functions

### geometric-functions
 * A set of functions for robot spatial transformations, rotation conversions and simple geometric relations. 

### utility-functions
 * A set of convenience functions for MATLAB
 
### robot-functions
 * A set of often used functions in MATLAB related to robots, including manipulations for visual servoing, simple trajectory generators, general transform jacobians based on DH parameters, and gradient descent based IGM.

## Robot Specific

### planar-RRR
 * A test folder for different tools with an example 3DOF planar robot 
     * Jacobian, Transforms inertia for simple robot
     * Visualizing a danger grid about a planar robot
     * Effective mass modeling for the robot 

### gough-steward-platform
 * A set of function used to model the gough stewart robot with felxible platform.
 * Both rigid functions and flexible functions are included (flexibility is modeled using finite element results with the mode shape superposition method) 

### kuka-lwr
 * Kinematic/Geometric functions used to model Kukalwr robot

### sawyer
 * Kinematic/Geometric functions used to model Rethink's sawyer robot

### ur10
 * Kinematic/Geometric functions used to model UR10 robot

### nao-actuation-schemes
 * Kinematic/Geometric functions used to model Nao robot cooperatively manipulating an object
