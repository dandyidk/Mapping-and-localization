#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# Global variables for robot position and map data
robot_position = None
occupancy_grid = None
map_info = None

# parameters that must be calculated pre match

#0.28 in length in simulation
#0.15 in wedth in simulation, we are going to cut them both in half as the sensor is in the middle
current_cmd_vel = Twist()
length_robot = 0.14

width_robot = 0.7
# this is simulation dependent, but in general the odom measurements should be shifted to 0.22 + robot length for x and 1 + robot width for y
position_shift_x =0.09

position_shift_y = 1 

# Callback for the /odom topic to get the robot's position
def odom_callback(data):
    global robot_position
    robot_position = (data.pose.pose.position.x-position_shift_x, data.pose.pose.position.y+position_shift_y)
    rospy.loginfo(f"Robot Position: {robot_position}")

# Callback for the /map topic to get the occupancy grid
def map_callback(data):
    global occupancy_grid, map_info
    occupancy_grid = np.array(data.data).reshape(data.info.height, data.info.width)
    map_info = data.info

# Function to convert robot world position to map grid coordinates
def world_to_map(x, y):
    if map_info:
        map_x = int((x - map_info.origin.position.x) / map_info.resolution)
        map_y = int((y - map_info.origin.position.y) / map_info.resolution)
        return map_x, map_y
    return None, None

# Function to check if the robot is in or near a red zone
def is_near_red_zone():
    if robot_position and occupancy_grid is not None:
        map_xpsv, map_ypsv = world_to_map(robot_position[0]+length_robot, robot_position[1]+length_robot)
        if map_xpsv is not None and map_ypsv is not None:
            if occupancy_grid[map_ypsv][map_xpsv] == -1:
                return True
        map_xngv, map_yngv = world_to_map(robot_position[0]-length_robot, robot_position[1]-length_robot)
        if map_xpsv is not None and map_ypsv is not None:
            if occupancy_grid[map_yngv][map_xngv] == -1:
                return True
    return False

# Function to avoid the red zone by adjusting robot's velocity
def avoid_red_zone():
    global in_red_zone
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    stop_pub = rospy.Publisher('/stop',Bool,queue_size=10)
    
    if is_near_red_zone():
        rospy.logwarn("Robot is approaching or is in the red zone! Reversing direction...")

        bool = Bool()
        bool.data = True
        stop_pub.publish(bool)
        
        # Reverse linear and angular velocities
        
        adjusted_cmd_vel = Twist()
        adjusted_cmd_vel.linear.x = -current_cmd_vel.linear.x
        adjusted_cmd_vel.linear.y = -current_cmd_vel.linear.y
        adjusted_cmd_vel.linear.z = -current_cmd_vel.linear.z
        adjusted_cmd_vel.angular.x = -current_cmd_vel.angular.x
        adjusted_cmd_vel.angular.y = -current_cmd_vel.angular.y
        adjusted_cmd_vel.angular.z = -current_cmd_vel.angular.z
        
        # Publish the adjusted cmd_vel
        while(is_near_red_zone()):
            cmd_vel_pub.publish(adjusted_cmd_vel)
    else:
        # If not near red zone, continue with the original cmd_vel
        cmd_vel_pub.publish(current_cmd_vel)

def cmd_vel_callback(data):
    global current_cmd_vel
    current_cmd_vel = data

def main():
    rospy.init_node('red_zone_avoidance')

    # Subscribe to the map and odometry topics
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)


    # Loop to continuously check the robot's position and avoid red zones
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        avoid_red_zone()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
