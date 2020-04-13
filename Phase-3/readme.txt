ENPM661
Project 3 Phase 3
Group 38

Libraries required
  numpy
  opencv

Info
  This code takes user input to find the optimal path of a robot in a fixed map using A* searching algorithm.
  Inputs required are:
    Clearance       How much space you want between the robot and obstacles
    Start X Coord   Starting x coordinate
    Start Y Coord   Starting y coordinate
    Start theta     Starting angle of robot IN DEGREES
    Goal X Coord    Goal x coordinate
    Goal Y Coord    Goal y coordinate
    rpm1            A wheel speed in RPM
    rpm2            Another wheel speed in RPM

  Code uses the two rpm values to generate 8 possible actions for the robot:
    [0,rpm1]
    [rpm1,0]
    [rpm1,rpm1]
    [0,rpm2]
    [rpm2,0]
    [rpm2,rpm2]
    [rpm1,rpm2]
    [rpm2,rpm1]

  Map is shown to the user using matplotlib.
  Obstacles are in red.
  Code generates the explored map in real-time, showing all explored paths in BLACK.
  Goal region is shown as a GREEN circle.
  After goal is reached, optimal path is highlighted in BLUE.

---
Code sections and descriptions
  USER INPUTS (ERROR CHECKING AND ROBOT DIMENSIONS)
    Functions that make sure user inputs numbers only
    Defines wheel dimensions
    Takes user input for clearance

  MAP
    Defines x and y axes
    Plots circle and square obstacles
    Plots map border
    Function to determine if [x,y] coordinate is inside obstacle

  USER INPUTS (COORDINATES)
    Functions to check if number is within X and Y axis range
    User inputs to collect:
      Start X and Y coordinates
      Start theta angle
      Goal X and Y coordinates
      Two RPM values for wheel speeds

    ACTION SET`
      Defines threshold
      Defines 8-action set
      Function to calculate nodes from wheel speed and initial position

    NODE CLASS AND FUNCTIONS
      Node class saves the following info for each node:
        Point coordinate [x,y,theta]
        Cost (calculated as distance traveled)
        Parent node
        Action [left wheel speed, right wheel speed]
      Function to output node with least cost
      Function to calculate cost to goal
      Function to find node's index value
      Function to check if curved path passes through an obstacle
      Function that plots curved path

    GENERATE GRAPH
      Function that generates nodes using A* algorithm
      Plots all explored paths in real-time in BLACK

    BACKTRACK
      Function that backtracks optimal path from goal to start
      Plots optimal path in BLUE
