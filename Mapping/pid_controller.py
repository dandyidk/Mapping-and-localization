#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool

flag = False

class ShatoPID:
    def __init__(self):
        rospy.init_node('shato_pid_controller', anonymous=True)
        
        # Publisher for velocity command (linear and angular velocity)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscriber to robot odometry
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.rate = rospy.Rate(10)

        # Robot pose [x, y, theta]
        self.pose = [0.0, 0.0, 0.0]

        # Desired pose [x, y, theta]
        self.goal_pose = [float(input("Set your x goal: ")),
                          float(input("Set your y goal: ")),
                          float(input("Set your theta goal (in degrees): "))]

        # PID coefficients
        self.kp_rho = 0.5
        self.kp_alpha = 1.0
        self.kp_beta = -0.3  # Negative to ensure correct turning direction
        
        # Robot parameters for Shato robot (modify as needed)
        self.L = 0.2  # Example wheelbase in meters
        self.r = 0.05  # Example wheel radius in meters

    def update_pose(self, data):
        """Callback function to update the robot's pose."""
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation

        # Convert quaternion orientation to Euler angles
        roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y,
                                                  orientation.z, orientation.w])

        # Update robot's current pose
        self.pose[0] = position.x
        self.pose[1] = position.y
        self.pose[2] = yaw  # In radians

    def euclidean_distance(self):
        """Calculate the Euclidean distance to the goal."""
        return math.sqrt((self.goal_pose[0] - self.pose[0]) ** 2 +
                         (self.goal_pose[1] - self.pose[1]) ** 2)

    def steering_angle(self):
        """Calculate the steering angle to the goal."""
        return math.atan2(self.goal_pose[1] - self.pose[1],
                          self.goal_pose[0] - self.pose[0])

    def angular_error(self):
        """Calculate the difference between the desired heading and current heading."""
        return self.normalize_angle(self.steering_angle() - self.pose[2])

    def normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi

        while angle < -math.pi:
            angle += 2 * math.pi

        return angle

    def linear_vel(self):
        """Calculate linear velocity."""
        return self.kp_rho * self.euclidean_distance()

    def angular_vel(self):
        """Calculate angular velocity."""
        return self.kp_alpha * self.angular_error() + self.kp_beta * self.normalize_angle(self.goal_pose[2] * math.pi / 180 - self.pose[2])

    def stop_program(self,data):
        global flag
        flag = data.data

    def move_to_goal(self):
        """Move the robot towards the goal."""
        vel_msg = Twist()

        rospy.Subscriber("/stop",Bool,self.stop_program)


        while(flag):
            rospy.Subscriber("/stop",Bool,self.stop_program)

        while not rospy.is_shutdown():
            # Calculate linear and angular velocities
            v = self.linear_vel()
            omega = self.angular_vel()

            # Compute right and left wheel velocities
            v_r = (v + (self.L / 2) * omega) / self.r
            v_l = (v - (self.L / 2) * omega) / self.r

            rospy.loginfo(f"Publishing velocities: v_r={v_r}, v_l={v_l}")

            # Set velocities in the Twist message
            vel_msg.linear.x = v
            vel_msg.angular.z = omega
            
            # Cap velocities if necessary
            vel_msg.linear.x = min(max(vel_msg.linear.x, -0.5), 0.5)  # Example limits
            vel_msg.angular.z = min(max(vel_msg.angular.z, -1.0), 1.0)  # Example limits

            # Publishing velocity
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stop the robot after reaching the goal
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        controller = ShatoPID()
        controller.move_to_goal()
    except rospy.ROSInterruptException:
        pass
