#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
# Add additional imports for each of the message types used



def publishTwist(linVel, angVel):
    global pub
    msg = Twist()
    msg.linear.x = linVel
    msg.angular.z = angVel
    pub.publish(msg)


#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):

#
    global pose
    pose = Pose()
    starting_x = pose.position.x
    starting_y = pose.position.y
    starting_theta = math.degrees(pose.orientation.z)

    diff_x = goal.pose.position.x - starting_x
    diff_y = goal.pose.position.y - starting_y
    dist = math.sqrt((diff_x * diff_x) + (diff_y * diff_y))
    w_theta_one = math.degrees(math.atan2(diff_y, diff_x))
    theta_one = w_theta_one - starting_theta

    print "start_theta: ", str(starting_theta),"difference_theta: ", str(w_theta_one), "change=: ", str(theta_one)
    
    w_theta_final = math.degrees(goal.pose.orientation.z)
    theta_two = w_theta_final - theta_one

    print "Second orientation= ", str(w_theta_final)
    print"Change for theta_two=: ", str(theta_two)

    print "spin.."
    rotate(theta_one)
    print "move to pos1"
    driveStraight(0.2, dist)
    print "spin to final destination"
    rotate(theta_two)


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    print "driveStraight function in action..."
    driveStraight(0.3, 0.6)
    print "Rotating..."
    rotate(90)
    print "driveStraight function in action 2..."
    driveStraight(0.45, 0.45)
    print "Rotating2..."
    rotate(-135)


#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    global pose
    pose = Pose()
    global publishes
    lin_vel = (u1 + u2)/2
    ang_vel = (u1 - u2) / 0.23 #from 9 inches to SI units aka meters

#    twist_msg = Twist();
#    stop_msg  = Twist();
    

#    twist_msg.linear.x = lin_vel
#    twist_msg.angular.z = ang_vel
#    stop_msg.linear.x=0
#    stop_msg.angular.z=0

    now = rospy.Time.now().secs #records the current time

    while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()): #runs for the time parameter of the function
    	publishTwist(lin_vel,ang_vel) #publishes the non zero values of the vel and the angle
        print "angle = ",str(math.degrees(pose.orientation.z))

    publishTwist(0,0) #publishes the stop message


#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose
    pose = Pose()
    aFlag=False
    flag = False

    start_x = pose.position.x
    start_y = pose.position.y
    
    while (not flag and not rospy.is_shutdown()):
    	
        curr_x = pose.position.x 
        curr_y = pose.position.y
        diff_x = curr_x - start_x
        diff_y = curr_y - start_y
        curr_pos = math.sqrt((diff_x * diff_x) + (diff_y * diff_y))
        print "x0 = ",str(start_x), " y0 = ",str(start_y) + " curr_x = ",str(curr_x) + " curr_y = ",str(curr_y)
        
        if (curr_pos >= distance):
            flag = True
            publishTwist(0, 0)
            
        else:
            publishTwist(speed,0)
            rospy.sleep(0.1000)

    

#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global pub
    global pose
    pose = Pose()
    
    lin_vel = 0
    ang_vel = 0.5

    if(angle < 0):
        ang_vel = -1 * ang_vel

    twist_msg = Twist();
    stop_msg = Twist();

    stop_msg.linear.x = 0
    stop_msg.angular.z = 0

    
    print "angle = ", angle
    angle = angle + math.degrees(pose.orientation.z)


    print "robotAngle =", math.degrees(pose.orientation.z)
    print "Originalangle", angle
    if (angle > 180):
        angle = angle - 360
    if(angle < -180):
        angle = angle + 360
    atTarget = False

    error = angle - math.degrees(pose.orientation.z)
    
    print "InitError=", error
    print "ang_vel=", ang_vel

    #twist_msg.linear.x = lin_vel
    #twist_msg.angular.z = ang_vel
    
    while((abs(error) >= 2.5 and not rospy.is_shutdown()):
        error = angle - abs(math.degrees(pose.orientation.z))
        print "error=", error, "robotAngle =", math.degrees(pose.orientation.z)
        #pub.publish(twist_msg)
        publishTwist(lin_vel, ang_vel)
        rospy.sleep(0.05)
    #pub.publish(stop_msg)
    publishTwist(0,0)




#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    global odom_list
    global pose
    pose = Pose()
    
    omega = speed / radius
    vel = Twist();

    error = angle - math.degrees(pose.orientation.z)

    while ((abs(error) >= 2) and not rospy.is_shutdown()):
        publishTwist(speed, omega)
        error = angle - (math.degrees(pose.orientation.z)) #made abs
        print "theta: %d" % math.degrees(pose.orientation.z)

    publishTwist(0, 0)



#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        # What should happen when the bumper is pressed?
        print "Bumper is hit!!!!!!"
        executeTrajectory()


# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    global pose
    pose = Pose()


    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)

    pose.position.x = position[0]
    pose.position.y = position[1]

    xPosition= position[0]
    yPosition= position[1]

    odomW = orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw =euler_from_quaternion(q)

    #converts yaw to deg

    pose.orientation.z = yaw
    theta = math.degrees(yaw)



# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('kkumar_lab2')

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global pose
    global odom_tf
    global odom_list
   
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion cmd_vel_mux/input/teleop
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    nav_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, navToPose, queue_size=10) # 
    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"

    #make the robot keep doing something...
    rospy.Timer(rospy.Duration(0.01), timerCallback)

    # Make the robot do stuff...
    #driveStraight(0.5,2)
    #spinWheels(0.0,0.07,25)
    #rotate(30)
    #navToPose(2)
    #driveArc(1, 0.5, 20) #driveArc(radius, speed, angle)
    executeTrajectory()

    print "Lab 2 complete!"

