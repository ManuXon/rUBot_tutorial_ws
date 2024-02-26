#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'move_turtle' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('move_turtle', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.goal_pose = Pose()
        self.goal_pose.x = rospy.get_param("~x")
        self.goal_pose.y = rospy.get_param("~y")
        self.orientation_pose = Pose()
        self.orientation_pose.x = rospy.get_param("~a") # Central position of the screen
        self.orientation_pose.y = rospy.get_param("~b") # Central position of the screen
        self.distance_tolerance = rospy.get_param("~tol")
        self.angle_tolerance = rospy.get_param("~atol")
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)
    def orientation2goal(self, goal_pose):
        # Angle distance to the goal
        vel_msg = Twist()
        # Tolerance to angle distance.
        angle_tolerance = self.angle_tolerance
        # If the angle to goal is less than tolerance we stop.
        while abs(self.steering_angle(goal_pose) - self.pose.theta) >= angle_tolerance:
            # Set angular velocity z to match angle goal.
            vel_msg.angular.z = self.angular_vel(goal_pose)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Set angular velocity to 0.
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = self.goal_pose.x
        goal_pose.y = self.goal_pose.y

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = self.distance_tolerance

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Robot Reached destination")
        
        # Orientation of the robot will be always to the middle of the screen.
        self.orientation2goal(self.orientation_pose)
        rospy.loginfo("Robot Reached orientation")
        rospy.logwarn("Stopping robot")
        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        turtle = TurtleBot()
        turtle.move2goal()
    except rospy.ROSInterruptException:
        pass