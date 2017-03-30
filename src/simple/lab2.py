#!/usr/bin/env python
import rospy, tf, copy, math
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String
# Add additional imports for each of the message types used



def navToPose(goal):
	#Know the starting position of the turtlebot
	pose = Pose()
	starting_x = pose.position.x
	starting_y = pose.position.y
	starting_theta = math.degrees(pose.orientation.z)

	#Find the position from the goal
	diff_x = goal.pose.position.x - starting_x
    diff_y = goal.pose.position.y - starting_y
    diff_theta = goal.pose.position.theta - starting_theta

    distToGoal = math.sqrt((diff_x * diff_x) + (diff_y * diff_y))

    driveStraight(distToGoal)
    rotate(current_theta - diff_theta + starting_theta) 


def executeTrajectory():
	driveStraight(0.3, 0.6)
    rotate(90)
    driveStraight(0.45, 0.45)
    rotate(-135)
  

def spinWheels(u1, u2, time):
	#Not sure!!


def driveStraight(speed, distance):
	new_x = pose.position.x 
	new_y = pose.position.y
    diff_x = new_x - starting_x
    diff_y = new_y - starting_y
    currPos = math.sqrt((diff_x * diff_x) + (diff_y * diff_y))

    publishTwist(speed,0)
    

def rotate(angle):

	starting_angle = math.degrees(pose.orientation.z)
	while ((abs(error) >= 2) and not rospy.is_shutdown()):
        publishTwist(0, rotation_speed)
        error = angle - (math.degrees(pose.orientation.z) - starting_angle)
    publishTwist(0, 0)#stop the robot

def driveArc(radius, speed, angle):
	pass  # Delete this 'pass' once implemented
	publishTwist(lin_vel, ang_vel) #have both the linear and angular speed


def readBumper(msg):
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
        publishTwist(0,0)
        pass  # Delete this 'pass' once implemented


# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):

	global pub
	global pose
	global odom_tf
	global odom_list
	pose = Pose()


    (position, orientation) = odom_list.lookupTransform('...','...', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
	
    pass # Delete this 'pass' once implemented

	# Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
	pub = rospy.Publisher() # Publisher for commanding robot motion
	bumper_sub = rospy.Subscriber() # Callback function to handle bumper events
	nav_subscriber = rospy.Subscriber() # handle nav goal events
	
	rospy.Timer(rospy.Duration(.1), timerCallback)


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('sample_Lab_2_node')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global pose
    global odom_tf
    global odom_list
   
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('...', ...) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('...', ..., readBumper, queue_size=10) # Callback function to handle bumper events

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"

    #make the robot keep doing something...
    rospy.Timer(rospy.Duration(...), timerCallback)

    # Make the robot do stuff...
    print "Lab 2 complete!"

