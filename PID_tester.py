#!/usr/bin/env python3

import rclpy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


mynode_ = None
pub_ = None
regions_ = {
    'left': 0,
    'right': 0,
    'fLeft': 0,
    'fRight': 0,
    'front1': 0,
    'front2': 0,
}
twstmsg_ = None
count = 0

# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)


def clbk_laser(msg):
    global regions_, twstmsg_, count

    print("Data Points: ", len(msg.ranges))
    
    regions_ = {
        #LIDAR readings are anti-clockwise, starting at 0 on the right-most edge of the LiDaR FOV.
        'right':  find_nearest(msg.ranges[21:23]) #lab3 slide instruction
    }

    twstmsg_= movement()

def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)


#Basic movement method
def movement():
    global regions_, mynode_
    regions = regions_
    
    print(f"Right: {regions['right']}")
    
    msg = Twist()


    if regions['front'] < 0.8:
        msg.linear.x = 0.0
        msg.angular.z = 0.2
    elif regions['right'] < 0.3:
        msg.linear.x = 0.1
        msg.angular.z = 0.3
    elif regions['right'] > 0.5 and regions['right'] < 1.5:
        msg.linear.x = 0.1
        msg.angular.z = -0.2
    else:
        msg.linear.x = 0.3
        msg.angular.z = 0.0
    return msg

#used to stop the rosbot
def stop():
    global mynode_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    return(msg)


def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    # publisher for twist velocity messages
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds 
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle interrupts
    try:
        rclpy.spin(mynode_)
    except Exception as e:
        print(e)
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()

if __name__ == '__main__':
    main()