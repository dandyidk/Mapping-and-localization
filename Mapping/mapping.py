#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Global variables for robot position and map data
robot_position = None
occupancy_grid = None
map_info = None

position_shift_x = 0.22

position_shift_y = 1

# Callback for the /odom topic to get the robot's position
def odom_callback(data):
    global robot_position
    robot_position = (data.pose.pose.position.x+position_shift_x, data.pose.pose.position.y+position_shift_y)
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
        map_x, map_y = world_to_map(robot_position[0], robot_position[1])
        if map_x is not None and map_y is not None:
            if occupancy_grid[map_y][map_x] == -1:
                return True
    return False

# Function to avoid the red zone by adjusting robot's velocity
def avoid_red_zone():
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()

    # If the robot is near a red zone, stop or move it in the opposite direction
    if is_near_red_zone():
        rospy.logwarn("Robot is near or in the red zone! Adjusting course...")
        # Simple avoidance logic: Move backward
        move_cmd.linear.x = -0.6  # Move backward
        move_cmd.angular.z = 0.5  # Turn to change direction

    # Publish the velocity command
    cmd_vel_pub.publish(move_cmd)

def main():
    rospy.init_node('red_zone_avoidance')

    # Subscribe to the map and odometry topics
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

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
