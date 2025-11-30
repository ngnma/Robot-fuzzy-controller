#!/usr/bin/env python3
class fuzzy_controller:
    def __init__(self):
        self.sensor_zone = {'near': [0, 0.25, 0.5], 'medium': [0.25, 0.5, 0.75], 'far': [0.5, 0.75, 10]}

        self.linear_zone = {'slow': [0.025, 0.05, 0.075], 'medium': [0.075, 0.1, 0.125], 'fast': [0.125, 0.15, 0.3]} #'fast': [0.125, 0.15, 0.175]
        self.angular_zone = {'right': [-0.5, -0.3, -0.1], 'front': [-0.1, 0, 0.1], 'left': [0.1, 0.2, 0.3]}

        self.rule_base = {
            ('near', 'near'): ('medium', 'left'),
            ('near', 'medium'): ('slow', 'left'),
            ('near', 'far'): ('medium', 'left'),
            ('medium', 'near'): ('slow', 'left'), #('medium', 'near'): ('slow', 'right'),
            ('medium', 'medium'): ('medium', 'front'),
            ('medium', 'far'): ('medium', 'left'), #('medium', 'far'): ('medium', 'left'),
            ('far', 'near'): ('fast', 'right'),
            ('far', 'medium'): ('fast', 'right'),
            ('far', 'far'): ('fast', 'right')
        }

    def calculate_membership(self, x, zones):
        # This dist contains the degree of membership of each set for x.
        # output exp. {'near': 0, 'medium': 0, 'far': 1}
        outputs = dict()

        for fuzzy_set, corners in zones.items():

            a, b, c = corners[0], corners[1], corners[2]

            if fuzzy_set == 'near':
                if x <= b:
                    outputs[fuzzy_set] = 1
                elif b < x <= c:
                    outputs[fuzzy_set] = (c-x)/(c-b)
                else:
                    pass

            if fuzzy_set == 'medium':
                if a < x <= b:
                    outputs[fuzzy_set] = (x-a)/(b-a)
                elif b < x <= c:
                    outputs[fuzzy_set] = (c-x)/(c-b)
                else:
                    pass

            if fuzzy_set == 'far':
                if a < x <= b:
                    outputs[fuzzy_set] = (x-a)/(b-a)
                elif b < x:
                    outputs[fuzzy_set] = 1
                else:
                    pass

        return outputs
    

    def calculate_fired_rules(self, rfs_values, rbs_values):
        fired_rules = []
        for rfs_var, rfs_degree in rfs_values.items():

            for rbs_var, rbs_degree in rbs_values.items():
                sensors = (rfs_var, rbs_var)
                degrees = (rfs_degree, rbs_degree)

                fire_stregth = min(rfs_degree, rbs_degree)
                
                rule = self.rule_base.get(sensors)
                print('FR:',sensors, '-> ',rule, ': ',fire_stregth)

                fired_rules.append((rule,fire_stregth))
        # print("Fired Rules: ", fired_rules)
        print(100*'-')
        return fired_rules
    
    def centroid_calculation(self, shape, corners):
        if shape == 'triangle':
            centroid = (corners[0] + corners[2])/2
        else:
            pass
        return centroid
    

    def fuzzification(self, distances):
        rfs_distance, rbs_distance = distances

        rfs_values = self.calculate_membership(rfs_distance, self.sensor_zone)
        rbs_values = self.calculate_membership(rbs_distance, self.sensor_zone)
        # print("RFS Membership Values: ", rfs_values)
        # print("RBS Membership Values: ", rbs_values)

        return rfs_values, rbs_values

    def defuzzification(self, fired_rules, linear_zone, angular_zone):
        linear_numerator = 0
        angular_numerator = 0
        denominator = 0

        # for each rule (crisp output value for linear and angular speeds and its strength)
        for (linear_var, angular_var), strength in fired_rules: # fired_rules exp: (('slow', 'left'), 0.75)

            # calculate centroids for each output fuzzy set
            linear_centroid = self.centroid_calculation('triangle', linear_zone[linear_var]) 
            angular_centroid = self.centroid_calculation('triangle', angular_zone[angular_var])   

            # calculate the numerators for linear and angular velocities (weighted sum of centroids and strengths)
            linear_numerator += linear_centroid * strength
            angular_numerator += angular_centroid * strength

            # calculate the denominator (sum of strengths)
            denominator += strength

        # calculate final linear and angular velocities
        linear_velocity = linear_numerator / denominator
        angular_velocity = angular_numerator / denominator

        return linear_velocity, angular_velocity
    
    def run(self, sensor_distances):

        rfs_values, rbs_values = self.fuzzification(sensor_distances)
        
        fired_rules = self.calculate_fired_rules(rfs_values, rbs_values)

        linear_velocity, angular_velocity = self.defuzzification(fired_rules, self.linear_zone, self.angular_zone)

        # print(f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}")
        return linear_velocity, angular_velocity

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

    # print("Data Points: ", len(msg.ranges))
    
    regions_ = {
        #LIDAR readings are anti-clockwise, starting at 0 on the right-most edge of the LiDaR FOV.
        # 'front': find_nearest(msg.ranges[120:122]),
        # 'right':  find_nearest(msg.ranges[30:32]),
        # 'left': find_nearest(msg.ranges[210:212])

        'front': find_nearest(msg.ranges[70:122]),
        'right':  find_nearest(msg.ranges[10:32]),
        'left': find_nearest(msg.ranges[210:212])

    }

    twstmsg_= movement()

def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)


#Basic movement method
def movement():
    global regions_, mynode_
    regions = regions_
    
    # print(f"Left: {regions['left']} | Front: {regions['front']} | Right: {regions['right']} \n")
    
    distances = (regions['front'], regions['right'])
    msg = Twist()

    x, z = fuzzy_flc.run(distances)


    msg.linear.x = x
    msg.angular.z = z
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
    fuzzy_flc = fuzzy_controller()
    main()
