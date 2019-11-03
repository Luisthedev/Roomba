#!/usr/bin/env python
#importa all of the libraries
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import math



#scan call back method
def scan_callback(msg):
    #creates all global variables
    global g_closest_range_ahead
    global g_closest_range_left
    global g_closest_range_right
    global g_closest_range_back

    global g_closest_range_diag_left
    global g_closest_range_diag_right

    #sets the values  of the global variables with the odom data
    g_closest_range_ahead = msg.ranges[359]
    g_closest_range_left = msg.ranges[90]
    g_closest_range_diag_left = msg.ranges[90-45]
    g_closest_range_right = msg.ranges[270]
    g_closest_range_diag_right = msg.ranges[270+45]
    g_closest_range_back = msg.ranges[180]

#odom call back method that gets the eular angle
def odom_callback(msg):
    #creates global variables
    global x
    global y
    global theta
    #gets the position of the robot and sets it as x
    x = msg.pose.pose.position.x
    #gets the position of the robot and sets it as x
    x = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll,pitch,theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

#method that determines if we can go forward with a given threshold
def can_go_for():
    #if the distance is small then .6 meeters and does not equal infinite (sim left over)
    if g_closest_range_ahead >.6 and g_closest_range_left != 'inf':
        return True
    #real life 0 means cant measure, so it would be too far
    elif g_closest_range_ahead == 0.0:
        return True
    else:
        return False
# method that determines if we can turn left with a given threshold
def can_turn_left():
    if g_closest_range_left > .5 and g_closest_range_left != 0 and g_closest_range_left != 'inf':
        return True
    return False
# method that determines if we can turn right with a given threshold
def can_turn_right():
    if g_closest_range_right >.5 and g_closest_range_right !=0 and g_closest_range_right !='inf':
        return True
    return False
# method that determines if we can go backwards with a given threshold
def can_go_back():
    if g_closest_range_back > .5 and g_closest_range_back !=0 and g_closest_range_back != 'inf':
        return True
    return False
# theorically if we are stuck in a corned this is supposed to detect it and thenreverse
def should_go_back():
    if can_go_for() == False and can_turn_left() == False and can_turn_right() == False and can_go_back() == True:
        return True
    return False
# method that determines if we getting close to a wall on its left with a given threshold
def geting_close_left():
    if g_closest_range_left < .3 and g_closest_range_left != 0 and g_closest_range_left != 'inf':
        return True
    return False
# method that determines if we getting close to a wall on its right with a given threshold
def geting_close_right():
    if g_closest_range_right < .3 and g_closest_range_right != 0 and g_closest_range_right != 'inf':
        return True
    return False
# method that determines if we getting close to a wall on its  diagonal left with a given threshold
def getting_close_diag_left():
    if g_closest_range_diag_left < .3 and g_closest_range_diag_left != 0 and g_closest_range_diag_left != 'inf':
        return True
    return False
# method that determines if we getting close to a wall on its  diagonal right with a given threshold
def getting_close_diag_right():
    if g_closest_range_diag_right <.3 and g_closest_range_diag_right != 0 and g_closest_range_diag_right != 'inf':
        return True
    return False
# if we are moving diagonaly to a wall then return true as we are approaching a wall to its left
def wall_apr_diag_left():
    if can_go_for() == True and getting_close_diag_left() == True:
        return True
    return False
# if we are moving diagonaly to a wall then return true as we are approaching a wall to its right
def wall_apr_diag_right():
    if can_go_for() == True and getting_close_diag_right() == True:
        return True
    return False


def run_ros():
    rate = rospy.Rate(2)
    twist = Twist()

    while not rospy.is_shutdown():
        #range_ahead_front = msg_globe.ranges[len(msg_globe.ranges)/2]
        print 'front',g_closest_range_ahead
        print 'back', g_closest_range_back
        print 'right', g_closest_range_right
        print 'diag_right', g_closest_range_diag_right
        print 'left', g_closest_range_left
        print 'diag_left', g_closest_range_diag_left

        print g_closest_range_back == 'inf'
        print twist

        print can_go_for()
        print g_closest_range_ahead
        print geting_close_left()
        print geting_close_right()

        # if we can go forward: and i am nor getting close to any walls go forward
        if can_go_for() == True and  geting_close_left()== False and geting_close_right()== False:
            print 'We, Are rolling forward'
            # sets the linear x velocity to .5
            twist.linear.x = .3
            # Publishes twist on cmd_vel
            cmd_vel_pub.publish(twist)
        # we are close to a wall and we can turn: Then turn
        elif can_go_for() == False and geting_close_left()== False and geting_close_right()== False and wall_apr_diag_left() == False and wall_apr_diag_right() == False:
            print 'Getting close to a Wall'
            # Stops the bot
            twist.linear.x = 0
            cmd_vel_pub.publish(twist)
            # loop until you can go forward
            while can_go_for() == False:
                #keep twisting
                print g_closest_range_ahead
                twist.angular.z = .1
                cmd_vel_pub.publish(twist)
            rate.sleep()
            # Stop the bot
            twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)
        # If we are stuck in a corner: go backwards
        elif should_go_back():
            # loop until both values are true
            while can_turn_right() == False and can_turn_left() == False:
                print 'going back'
                # Go backwards
                twist.linear.x = -0.1
                cmd_vel_pub.publish(twist)
            # Stop the bot
            twist.linear.x = 0
            cmd_vel_pub.publish(twist)

        # if we are apruching wall at a diagonal angle
        elif(can_go_for() == False and geting_close_left() == True):
            twist.linear.x = 0
            twist.angular.z = 0
            cmd_vel_pub.publish(twist)
            while can_go_for() == False :
                twist.linear.x = .2
                print 'we are turning'
                cmd_vel_pub.publish(twist)
                rate.sleep()
            twist.linear.x = 0
            cmd_vel_pub.publish(twist)

        elif can_go_for() == True and geting_close_left() == True and wall_apr_diag_left() == True  :
            print 'getting close, lets go left '
            # sets all values to 0
            twist.linear.x = 0
            twist.angular.z = 0
            cmd_vel_pub.publish(twist)
            # Turn while we can turn
            twist.angular.z = -3
            while can_turn_left() == False :
                print 'we are turning'

                print g_closest_range_left
                cmd_vel_pub.publish(twist)
                print g_closest_range_left
                rate.sleep()

            count = 0
            twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)
            twist.linear.x = .3
            # while we can go forward
            while count < 2 and can_go_for() == True:
                print 'geting away from a wall '
                couunt = count +1
                cmd_vel_pub.publish(twist)
                rate.sleep()
            # Sets all vals to 0 to stop the bot
            twist.angular.z = 0.0
            twist.linear.x = 0.0
            cmd_vel_pub.publish(twist)
        #  if we get cloose to a wall on its right
        elif can_go_for() == True and geting_close_right() == True or can_go_for() == False and geting_close_right() == True:
            print 'getting close, lets go right '
            # Stops bot
            twist.linear.x = 0
            cmd_vel_pub.publish(twist)
            twist.angular.z = 0.2
            count = 0
            while  can_turn_right() == False:
                couunt = count +1
                print 'we are turning'
                print g_closest_range_right
                cmd_vel_pub.publish(twist)
                print g_closest_range_right
                rate.sleep()
            #stops bot
            twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)

            count = 0
            twist.angular.z = 0.0
            cmd_vel_pub.publish(twist)
            twist.linear.x = .3
            #gor forward for a while to get away from the wall
            while count < 2 and can_go_for() != True:
                couunt = count +1
                cmd_vel_pub.publish(twist)
                rate.sleep()

            twist.angular.z = 0.0
            twist.linear.x = 0.0
            cmd_vel_pub.publish(twist)
        rate.sleep()

# Main program
g_range_ahead = 1 # anything to start

# Declare a subscriber to message 'scan' with message class LaserScan
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)

odom_call =  rospy.Subscriber('odom',Odometry, odom_callback)

# Same code can be a publisher and a subscriber, this is no problem
# be ready to publish the cmd_vel topic of class Twist
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Declare us to be a node
rospy.init_node('turtle_roomba')
state_change_time = rospy.Time.now()

# sets values to all globals at the start of the program
g_closest_range_ahead = 1
g_closest_range_back = 1
g_closest_range_right = 1
g_closest_range_left = 1
g_closest_range_diag_left = 1
g_closest_range_diag_right = 1
# method that runs the program
run_ros()
