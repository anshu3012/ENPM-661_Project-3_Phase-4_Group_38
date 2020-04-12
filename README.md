# ENPM-661-Proj-3--Phase-4-Group 38
## By Jaad Lepak, & Anshuman Singh

## Implementation of A* algorithm on a differential drive (non-holonomic) TurtleBot robot in ROS


In this project we will navigate a TurtleBot in a given map environment from a given start point to a given goal point, considering differential drive constraints while implementing the A* algorithm, with 8-connected action space in ROS

## Dependencies
numpy 

matplotlib

math 

ROS 

Gazebo 

Rviz

## Instructions to run 


 ## Planning Program Explanation
 Searches fixed map to find optimal path from user-defined start and end
 Action set limited to 8 actions  
 
    0,rpm1, 
    
    rpm1,0

    rpm1,rpm1

    0,rpm2

    rpm2,0

    rpm2,rpm2

    rpm1,rpm2

    rpm2,rpm1
 
 While searching, threshold of 0.0204 used for X and Y directions and 30 degrees for angle
 
 The goal is determined reached when within 1.5 radius region of user-defined end coordinates
 
 After solution is found, map is displayed with:
    Obstacles in RED
    Explored vector paths in BLACK
    Goal region in GREEN
    Optimal path shown in BLUE
## Different sections of code:
### Libraries
  Imports all libraries
  
### User Inputs (Error-checking and robot dimensions)
  Error-checking functions
    Helps restrict inputs when asking user
    
  ### Robot radius and clearance
   User inputs for radius and clearance
   
### Map
  Generates the map using algebraic expressions to represent obstacles
  
### Obstacle Check
  Returns true if a point x,y is inside an obstacle
  Returns false if point is outside obstacle
  
### User Input (Coordinates)
  Collects start coordinates, end coordinates,initial angle (in degrees), right wheel RPM, left wheel RPM and clearance
  
### Action Set
  Defines 8 actions sets on the basis of the wheel RPMs

    0,rpm1

    rpm1,0

    rpm1,rpm1

    0,rpm2

    rpm2,0

    rpm2,rpm2

    rpm1,rpm2

    rpm2,rpm1

## Node class and functions
  Functions to store node information and calculate total cost
  
## Generate graph
  Uses A star algorithm to iterate action sets throughout map
  Records visited vectors to plot

## Publisher Node 
   publishes the `/cmd_vel` node of the turtle bot which acts as the listener 
  
  
 

