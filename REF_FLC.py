#!/usr/bin/env python3
class FuzzyController:
    def __init__(self, sensor_zone={}, linear_zone={}, angular_zone={}, rule_base={}):
        self.sensor_zone = sensor_zone
        self.linear_zone = linear_zone
        self.angular_zone = angular_zone
        self.rule_base = rule_base

    def set_rule(self, antecedents, consequents):
        self.rule_base[antecedents] = consequents
        # example -> flc.set_rule(('far', 'far'), ('fast', 'right'))

    def set_sensor_membership(self, set_name, shape, corners):
        self.sensor_zone[set_name] = {'shape':shape, 'corners':corners}

    def set_linear_membership(self, set_name, shape, corners):
        self.linear_zone[set_name] = {'shape':shape, 'corners':corners}

    def set_angular_membership(self, set_name, shape, corners):
        self.angular_zone[set_name] = {'shape':shape, 'corners':corners}

    def calculate_membership(self, x):
        # This dist contains the degree of membership of each set for x.
        # output exp. {'near': 0, 'medium': 0, 'far': 1}
        outputs = dict()

        for fs in self.sensor_zone.items():
            set_name = fs[0]
            shape = fs[1]['shape']
            corners = fs[1]['corners']
            a, b, c = corners[0], corners[1], corners[2]

            if shape == 'left-trap':
                if x <= b:
                    outputs[set_name] = 1
                elif b < x <= c:
                    outputs[set_name] = (c-x)/(c-b)
                else:
                    pass

            if shape == 'tri-angle':
                if a < x <= b:
                    outputs[set_name] = (x-a)/(b-a)
                elif b < x <= c:
                    outputs[set_name] = (c-x)/(c-b)
                else:
                    pass

            if shape == 'right-trap':
                if a < x <= b:
                    outputs[set_name] = (x-a)/(b-a)
                elif b < x:
                    outputs[set_name] = 1
                else:
                    pass

        return outputs
    
    def fuzzification(self, distances):
        # calculate degree of membership to each fuzzy set for all sensor's outputs(distances)
        rfs_distance, rbs_distance = distances

        rfs_values = self.calculate_membership(rfs_distance)
        rbs_values = self.calculate_membership(rbs_distance)
        # print("RFS Membership Values: ", rfs_values)
        # print("RBS Membership Values: ", rbs_values)

        return rfs_values, rbs_values

    def calculate_fired_rules(self, rfs_values, rbs_values):
        # this function just works for 2 sensors. for 3 sensors we should have 3 for loops.
        fired_rules = []
        # right-front sensor
        for rfs_var, rfs_degree in rfs_values.items():
            # right-back sensor
            for rbs_var, rbs_degree in rbs_values.items():
                # calculate the fire_strength 
                sensors = (rfs_var, rbs_var)
                degrees = (rfs_degree, rbs_degree)
                # fire_strength is the min (AND)
                fire_stregth = min(degrees) # min(rfs_degree, rbs_degree)
                
                rule = self.rule_base.get(sensors) # search in rule-base for this rule
                # print('FR:',sensors, '-> ',rule, ': ',fire_stregth)

                fired_rules.append((rule,fire_stregth)) # if the rule is fired it will be append to fired_rules
        # print("Fired Rules: ", fired_rules)
        print(100*'-')
        return fired_rules
    
    def centroid_calculation(self, corners, shape = 'triangle'):
        if shape == 'triangle':
            centroid = (corners[0] + corners[2])/2
        else:
            pass
        return centroid
    
    def defuzzification(self, fired_rules):
        linear_numerator = 0
        angular_numerator = 0
        denominator = 0

        # for each rule (crisp output value for linear and angular speeds and its strength)
        for (linear_var, angular_var), strength in fired_rules: # fired_rules exp: (('slow', 'left'), 0.75)

            # calculate centroids for each output fuzzy set
            linear_centroid = self.centroid_calculation(self.linear_zone[linear_var]) 
            angular_centroid = self.centroid_calculation(self.angular_zone[angular_var])   

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
        # sensor_values -> Fuzzification -> Inference -> Defuzzification -> motors_speeds

        # Fuzzification: calculate the degree of membership to each fuzzy set from crisp inputs.
        rfs_values, rbs_values = self.fuzzification(sensor_distances)

        # Inference: For each rule in rule_base calculate its firing_strenght and returns all fired rules with their firing strengh.

        fired_rules = self.calculate_fired_rules(rfs_values, rbs_values)

        # Defuzzification: calculate crisp output for motor's speeds from linguistics variables
        linear_velocity, angular_velocity = self.defuzzification(fired_rules)

        # print(f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}")
        return linear_velocity, angular_velocity


# Robot Code
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
        
    # read distances from sensors
    distances = (regions['front'], regions['right'])
    msg = Twist()
    x, z = flc.run(distances)

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
     # Fuzzy sets for the three sensors
    sensor_zone = {
        'near': {'shape':'left-trap', 'corners':[0, 0.25, 0.5]}, 
        'medium': {'shape':'tri-angle', 'corners':[0.25, 0.5, 0.75]}, 
        'far': {'shape':'right-trap', 'corners':[0.5, 0.75, 10]}
    }

    # Speeds (direction and speed)
    linear_zone = {
        'slow': [0.025, 0.05, 0.075], 
        'medium': [0.075, 0.1, 0.125], 
        'fast': [0.125, 0.15, 0.3]
    }
    angular_zone = {
        'right': [-0.5, -0.3, -0.1], 
        'front': [-0.1, 0, 0.1], 
        'left': [0.1, 0.2, 0.3]
    }

    # Rule base (right front sensor, right back sesnor): (linear motor, angular motor)
    rule_base = {
            ('near', 'near'): ('medium', 'left'), # ||
            ('near', 'medium'): ('slow', 'left'), # /|
            ('near', 'far'): ('medium', 'left'), # -|
            ('medium', 'near'): ('slow', 'left'), #('medium', 'near'): ('slow', 'right'), \|
            ('medium', 'medium'): ('medium', 'front'), # | |
            ('medium', 'far'): ('medium', 'left'), # / |
            ('far', 'near'): ('fast', 'right'), # \|
            ('far', 'medium'): ('fast', 'right'), # \ |
            ('far', 'far'): ('fast', 'right') # \   |
        }
        
    # create instance of controller and initial sensors zones, motors zones and rule base
    flc = FuzzyController(
        sensor_zone = sensor_zone, 
        linear_zone = linear_zone, 
        angular_zone = angular_zone,
        rule_base = rule_base
    )

    main()
