#!/usr/bin/env python

# ENPM661
# Project 3 Phase 4
# Group 38

# ===== Libraries =====
import numpy as np
import cv2
import math
from math import pi
import time
import rospy
from geometry_msgs.msg import Twist

time.sleep(5)

def move(dt,dx,dy,dtheta):
	"""
	Moves robot in gazebo
	Inputs:
		dt:	Change in time for movement
		dx:	X direciton distance
		dy:	Y direction distance
	"""
	# New node
	rospy.init_node('robot_cleaner', anonymous=True)
	velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
	vel_msg = Twist()

	# Gazebo scaling
	scale = 1
	#dt *= scale
	dx *= scale
	dy *= scale
	dtheta *= scale

	# User input
	distance = math.sqrt(dx**2 + dy**2)
	vx = dx/dt
	vy = dy/dt
	vtheta = dtheta/dt
	speed = math.sqrt(vx**2 + vy**2)

	# Movement
	vel_msg.linear.x = vx
	vel_msg.linear.y = vy
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = vtheta

	# # Current time for distance
	# t0 = rospy.Time.now().to_sec()
	# current_distance = 0
	#
	# # Move specific distance
	# while(current_distance < distance):
	# 	# Publish vel.
	# 	print("Moving ", vx, "x m/s ", vy, "y m/s" )
	# 	velocity_publisher.publish(vel_msg)
	# 	# Vel. calc.
	# 	t1 = rospy.Time.now().to_sec()
	# 	# Distance calculation
	# 	current_distance = speed*(t1-t0)

	# Move robot
	print("Moving ", vx, "x m/s ", vy, "y m/s", vtheta, "theta m/s" )
	velocity_publisher.publish(vel_msg)
	# Wait for specified time
	rospy.sleep(dt)
	#Stop robot after loop
	vel_msg.linear.x = 0
	vel_msg.linear.y = 0
	vel_msg.angular.z = 0
	# Force robot to stop
	velocity_publisher.publish(vel_msg)

def path():
	"""
	Phase 3 code that solves optimal path
	"""
	# ===== User Inputs (Error-checking and robot dimensions) =====
	print("\n\nThis code will implement A* Algorithm for a Rigid Robot with Non-Holonomic Constraints")
	# Error-checking functions
	def inputNum(message):
	    """Only takes an integer as input"""
	    while True:
	        user = input(message)
	        try:
	            output = float(user)
	            break;
	        except ValueError:
	            print('\n!!! ERROR !!! Input needs to be a number. Please try again.\n')
	    return output

	def inputNumZeroPos(message):
	    """Only allows numbers 0 and above"""
	    output = inputNum(message)
	    while output < 0:
	        print("\n!!! ERROR !!! Please enter a number that is 0 or positive.\n")
	        output = inputNum(message)
	    return output

	# Robot wheel radius, distance between wheels, and clearance
	wheel_radius = 0.038
	dist_bw_wheels = 0.354
	clearance = inputNumZeroPos("Please enter the desired clearance: ")
	clr = clearance
	# ===== Map =====
	# Map dimensions
	xMin = -5.1
	xMax = 5.1
	yMin = -5.1
	yMax = 5.1

	# ===== Obstacles =====
	# Function to check if point is inside obstacle
	def inside_obstacle(x,y):
	    """Returns true if x,y coordinate is inside obstacle
	    Returns false if x,y coordinate is NOT inside obstacle"""
	    obstacle_check = 0 # 0 means not inside obstacle, 1 means inside obstacle
	    # circles
	    centres = [(0, 0), (2, 3), (-2, -3), (2, -3)]
	    cirR = 1 + clr
	    for centre in centres:
	        cirX = centre[0]
	        cirY = centre[1]
	        if ((x - cirX)**2 + (y - cirY)**2) <= cirR**2:
	            #print("HIT CIRCLE")
	            obstacle_check = 1
	    # squares
	    corners = [(-4.85, -0.75), (-2.85, 2.35), (3.35, -0.75)]
	    for corner in corners:
	        if x >= (corner[0] - clr) and y >= (corner[1] - clr)\
	         and y <= (corner[1] + 1.5 + clr) and x <= (corner[0] + 1.5 + clr):
	            #print("HIT SQUARE")
	            obstacle_check = 1
	    # border
	    rc = clr + 0.1 # border of map

	    if x <= xMin + rc or x >= xMax - rc:
	        #print("HIT BORDER")
	        obstacle_check = 1
	    if y <= yMin + rc or y >= yMax - rc:
	        #print("HIT BORDER")
	        obstacle_check = 1

	    if obstacle_check == 1:
	        return True
	    else:
	        return False

	# ===== User Inputs (Coordinates) =====
	# Error Checking functions
	def inputNumX(message):
	    """Only allows numbers between start and end"""
	    output = inputNum(message)
	    while output < xMin or output > xMax:
	        print("\n!!! ERROR !!! Please enter a number between ", xMin, " and ", xMax, "\n")
	        output = inputNum(message)
	    return output

	def inputNumY(message):
	    """Only allows numbers between start and end"""
	    output = inputNum(message)
	    while output < yMin or output > yMax:
	        print("\n!!! ERROR !!! Please enter a number between ", yMin, " and ", yMax, "\n")
	        output = inputNum(message)
	    return output

	# Start point coordinates
	start_cords_x = inputNumX("Please enter the starting x coordinate: ")
	start_cords_y = inputNumY("Please enter the starting y coordinate: ")
	# Obstacle check
	while inside_obstacle(start_cords_x,start_cords_y) == 1:
	        print("\n!!! ERROR !!! Coordinates inside obstacle. Please try again.\n")
	        start_cords_x = inputNumX("Please enter the starting x coordinate: ")
	        start_cords_y = inputNumY("Please enter the starting y coordinate: ")
	# Starting angle
	start_theta = inputNum("Please enter the starting angle in degrees: ")
	start_theta = start_theta*pi/180 # convert deg to radians
	# Goal point coordinates
	goal_cords_x = inputNumX("Please enter the goal x coordinate: ")
	goal_cords_y = inputNumY("Please enter the goal y coordinate: ")
	# Obstacle check
	while inside_obstacle(goal_cords_x,goal_cords_y) == 1:
	        print("\n!!! ERROR !!! Coordinates inside obstacle. Please try again.\n")
	        goal_cords_x = inputNumX("Please enter the goal x coordinate: ")
	        goal_cords_y = inputNumY("Please enter the goal y coordinate: ")
	# Wheel velocity inputs
	rpm1 = inputNum("Please enter a wheel speed in RPM: ")
	rpm2 = inputNum("Please enter another wheel speed in RPM: ")

	# ===== Action Set =====
	# Threshold
	thresholdXY = 0.4 # Creates a 500 x 500 map to explore
	thresholdTheta = 30 # Angle [deg] threshold for theta

	# 8-action set using user-defined wheel speeds
	actions = [[0,rpm1],\
	            [rpm1,0],\
	            [rpm1,rpm1],\
	            [0,rpm2],\
	            [rpm2,0],\
	            [rpm2,rpm2],\
	            [rpm1,rpm2],\
	            [rpm2,rpm1]]

	# Generate node
	def generate_node_location(uL,uR,point):
	    """
	    8-action space
	        Inputs:
	            uL:     Left wheel RPM
	            uR:     Right wheel RPM
	            x:      Initial X cartesian coordinate of robot
	            y:      Initial Y cartesian coordinate of robot
	            theta:  Initial theta angle of robot in RADIANS
	        Output:
	            xF:      Final X cartesian coordinate of robot
	            yF:      Final Y cartesian coordinate of robot
	            thetaF:  Final theta angle of robot in RADIANS
	            cost:    Final cost
	    """
	    # Parameters
	    x = point[0]
	    y = point[1]
	    theta = point[2]
	    r = wheel_radius
	    L = dist_bw_wheels
	    #print("point ", point)

	    # Final position and cost calculation
	    t=0
	    dt=0.1
	    cost = 0

	    while t<2:

	        t=t+dt

	        dx=(r/2)*(uL+uR)*math.cos(theta)*dt
	        dy=(r/2)*(uL+uR)*math.sin(theta)*dt
	        dtheta=(r/L)*(uR-uL)*dt
	        dcost = math.sqrt(dx**2 + dy**2)

	        x=x+dx
	        y=y+dy


	        theta=theta+dtheta
	        cost += dcost

	    new_point = [x,y,theta]
	    base_cost = cost

	    #print("new_point ", new_point)
	    #print("base_cost ", base_cost)
	    return new_point, base_cost

	# ===== Node class and functions =====
	class Node:
	    def __init__(self, point):
	        self.point = point # [x, y, theta]
	        self.cost = float('inf')  # initially all the new nodes have infinite cost attached to them
	        self.parent = None
	        self.action = [0,0] # [w1,w2] left and right wheel speeds

	def pop_queue_element(queue):  # Priority Queue, outputs the node with least cost attached to it
	    min_a = 0
	    for elemt in range(len(queue)):
	        if queue[elemt].cost < queue[min_a].cost:
	            min_a = elemt
	    return queue.pop(min_a)

	def cost_to_goal(point, goal_node_pos):
	    point = [point[0],point[1]]
	    point_x = point[0]
	    point_y = point[1]
	    goal_x = goal_node_pos[0]
	    goal_y = goal_node_pos[1]
	    euc_dist = np.sqrt((point_x-goal_x)**2 + (point_y-goal_y)**2)
	    point = np.array(point)
	    goal = np.array(goal_node_pos)
	    euc_dist = np.linalg.norm(point - goal)
	    return euc_dist

	def find_node(point, queue):
	    for elem in queue:
	        if elem.point == [[int(point[0]/thresholdXY)],[int(point[1]/thresholdXY)],[int(point[2]/thresholdTheta)]]:
	            return queue.index(elem)
	        else:
	            return None
	# Check if path crosses inside obstacle
	def path_inside_obstacle(uL,uR,point):
	    """
	    Checks if curve path goes inside an obstacle
	    Returns true if path crosses obstacle
	    Returns false if path is free of obstacles
	    Inputs:
	        uL: Left wheel speed
	        uR: Right wheel speed
	        point: [x,y,theta] robot position and angle
	    """
	    path_check = 0
	    #   0 is path is NOT inside obstacle
	    #   1 is path is inside obstacle

	    # Parameters
	    x = point[0]
	    y = point[1]
	    theta = point[2]
	    r = wheel_radius
	    L = dist_bw_wheels

	    # Curve path
	    t=0
	    dt=0.001 #fine resolution to detect while path draws inside an obstacle

	    while t<2:

	        t=t+dt

	        dx=(r/2)*(uL+uR)*math.cos(theta)*dt
	        dy=(r/2)*(uL+uR)*math.sin(theta)*dt
	        dtheta=(r/L)*(uR-uL)*dt

	        x=x+dx
	        y=y+dy
	        theta=theta+dtheta

	        if inside_obstacle(x,y) == 1:
	            path_check = 1
	            break

	    if path_check == 1:
	        return True
	    else:
	        return False

	# Plot curve
	def plot_curve(uL,uR,point):
	    """
	    Plots curve in matplotlib
	    Inputs:
	        uL: A user-defined wheel speed in RPM
	        uR: Another user-defined wheel speed in RPM
	        point:  [x,y,theta] current point and angle of robot
	    """
	    # Parameters
	    x = point[0]
	    y = point[1]
	    theta = point[2]
	    r = wheel_radius
	    L = dist_bw_wheels
	    #print("point ", point)

	    # Plot
	    t=0
	    dt=0.1

	    while t<2:

	        t=t+dt

	        dx=(r/2)*(uL+uR)*math.cos(theta)*dt
	        dy=(r/2)*(uL+uR)*math.sin(theta)*dt
	        dtheta=(r/L)*(uR-uL)*dt

	        move(dt,dx,dy,dtheta)

	        x=x+dx
	        y=y+dy
	        theta=theta+dtheta

	# ===== Generate graph =====
	print("Generating graph")
	# A star search algorithm
	def a_star_algo(clr, start_node_pos, goal_node_pos):
	    # Initial parameters
	    start_node = Node(start_node_pos)
	    start_node.cost = 0
	    xRange = abs(xMin - xMax)
	    yRange = abs(yMin - yMax)
	    thetaRange = 360
	    visited = np.zeros([int(xRange/thresholdXY),int(yRange/thresholdXY),int(thetaRange/thresholdTheta)])
	    queue = [start_node]
	    nodes = [[start_cords_x,start_cords_y,start_theta,0,0]]
	    counter = 0
	    # Show goal region
	    goal_region = 0.3
		# A star search
	    while queue:
	        current_node = pop_queue_element(queue)
	        current_point = current_node.point
	        visited[int(current_point[0]/thresholdXY)][int(current_point[1]/thresholdXY)][int(current_point[2]/thresholdTheta)] = 1
	        if ((current_point[0] - goal_node_pos[0])**2 + (current_point[1] - goal_node_pos[1])**2) <= goal_region**2:
	            print("Goal reached")
	            #print("current_point[0] ", current_point[0])
	            #print("current_point[1] ", current_point[1])
	            return current_node, nodes, counter
	        for action in actions:
	            w1 = action[0]
	            w2 = action[1]
	            new_point, base_cost = generate_node_location(w1,w2,current_point)
	            in_obstacle = inside_obstacle(new_point[0],new_point[1])
	            path_in_obstacle = path_inside_obstacle(w1,w2,new_point)
	            if in_obstacle != 1 and path_in_obstacle == False:

	                new_node = Node(new_point)
	                new_node.parent = current_node
	                new_node.action = [w1,w2]

	                if visited[int(new_point[0]/thresholdXY)][int(new_point[1]/thresholdXY)][int(new_point[2]/thresholdTheta)] == 0:
	                    new_node.cost = base_cost + new_node.parent.cost + cost_to_goal(new_point, goal_node_pos)
	                    visited[int(new_point[0]/thresholdXY)][int(new_point[1]/thresholdXY)][int(new_point[2]/thresholdTheta)] = 1
	                    queue.append(new_node)
	                    nodes.append([current_point[0],current_point[1],current_point[2],w1,w2])
	                    counter += 1

	                    if ((new_point[0] - goal_node_pos[0])**2 + (new_point[1] - goal_node_pos[1])**2) <= goal_region**2:
	                        print("Goal reached")
	                        #print("current_point[0] ", current_point[0])
	                        #print("current_point[1] ", current_point[1])
	                        return new_node, nodes, counter

	                else:
	                    node_exist_index = find_node(new_point, queue)
	                    if node_exist_index is not None:
	                        temp_node = queue[node_exist_index]
	                        if temp_node.cost > base_cost + new_node.parent.cost + cost_to_goal(new_point, goal_node_pos):
	                            temp_node.cost = base_cost + new_node.parent.cost + cost_to_goal(new_point, goal_node_pos)
	                            temp_node.parent = current_node
	            else:
	                continue
	    return None, nodes, counter

	start_node_pos = [start_cords_x, start_cords_y, start_theta]
	goal_node_pos = [goal_cords_x, goal_cords_y] # goal theta ignored
	# Record time
	start_time = time.time()
	# Search map
	result, nodes, counter = a_star_algo(clr, start_node_pos, goal_node_pos)
	# Print final time
	print("Time explored = %2.3f seconds " % (time.time() - start_time))

	# ===== Backtrack =====

	print("Backtracking...")
	def track_back(node):
	    p = list()
	    p.append(node)
	    p.append(node.parent)
	    parent = node.parent
	    if parent is None:
	        return p
	    while parent is not None:
	        p.append(parent)
	        parent = parent.parent
	    p_rev = list(p)
	    return p_rev

	if result is not None:
	    nodes_list = track_back(result)
	    x = []
	    y = []
	    theta = []
	    uL = []
	    uR = []
	    for elem in nodes_list:
	        x.insert(0,elem.point[0])
	        y.insert(0,elem.point[1])
	        theta.insert(0,elem.point[2])
	        uL.insert(0,elem.action[0])
	        uR.insert(0,elem.action[1])
	    for index in range(1,len(x)):
	        if index != (len(x) - 2):
	            X = x[index - 1]
	            Y = y[index - 1]
	            Theta = theta[index - 1]
	            UL = uL[index]
	            UR = uR[index]
	            print("x ", X, "y ", Y, "theta ", Theta, "uL ", UL, "uR ", UR)
	            plot_curve(UL,UR,[X,Y,Theta])
	    print("\nProgram complete.\n")

	else:
	    print("\nSorry, result could not be reached\n")

	print("Use 'Ctrl+C' to exit.\n")

if __name__ == '__main__':
	try:
		path()
	except rospy.ROSInterruptException: pass
