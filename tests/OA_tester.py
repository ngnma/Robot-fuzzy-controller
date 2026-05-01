#!/usr/bin/env python3
class FuzzyController:
    def __init__(self, front_sensor_zone={}, front_right_sensor_zone={}, front_left_sensor_zone={}, linear_zone={}, angular_zone={}, rule_base={}):
        self.front_sensor_zone = front_sensor_zone
        self.front_right_sensor_zone = front_right_sensor_zone
        self.front_left_sensor_zone = front_left_sensor_zone
        self.linear_zone = linear_zone
        self.angular_zone = angular_zone
        self.rule_base = rule_base

    def set_rule(self, antecedents, consequents):
        self.rule_base[antecedents] = consequents
        # example -> flc.set_rule(('far', 'far'), ('fast', 'right'))

    def set_front_right_sensor_membership(self, set_name, shape, corners):
        self.front_right_sensor_zone[set_name] = {'shape':shape, 'corners':corners}

    def set_front_left_sensor_membership(self, set_name, shape, corners):
        self.front_left_sensor_zone[set_name] = {'shape':shape, 'corners':corners}

    def set_front_sensor_membership(self, set_name, shape, corners):
        self.front_sensor_zone[set_name] = {'shape':shape, 'corners':corners}

    def set_linear_membership(self, set_name, shape, corners):
        self.linear_zone[set_name] = {'shape':shape, 'corners':corners}

    def set_angular_membership(self, set_name, shape, corners):
        self.angular_zone[set_name] = {'shape':shape, 'corners':corners}

    def calculate_membership(self, x, sensor_position):
        # This dist contains the degree of membership of each set for x.
        # output exp. {'near': 0, 'medium': 0, 'far': 1}
        outputs = dict()

        if sensor_position == 'FR':
            sensor = self.front_right_sensor_zone
        elif sensor_position == 'F':
            sensor = self.front_sensor_zone
        else: # sensor_position is LF
            sensor = self.front_left_sensor_zone

        for fs in sensor.items():
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
        frs_distance, fs_distance, fls_distance = distances

        frs_values = self.calculate_membership(frs_distance, 'FR') # front-right
        fs_values = self.calculate_membership(fs_distance, 'F') # front
        fls_values = self.calculate_membership(fls_distance, 'FL') # front-left

        print("front-right Membership Values: ", frs_values)
        print("front Membership Values: ", fs_values)
        print("front-left Membership Values: ", fls_values)

        return frs_values, fs_values, fls_values
    

    def calculate_fired_rules(self, frs_values, fs_values, fls_values):
        # this function just works for 3 sensors. each loop belongs to one sensor
        fired_rules = []
        # front-right sensor
        for frs_var, frs_degree in frs_values.items():
            # front sensor
            for fs_var, fs_degree in fs_values.items():
            # front-left sensor
                for fls_var, fls_degree in fls_values.items():
                    # calculate the fire_strength 
                    sensors = (frs_var, fs_var, fls_var)
                    degrees = (frs_degree, fs_degree, fls_degree)
                    fire_strength = min(degrees) # fire_strength is the min (AND)

                    rule = self.rule_base.get(sensors) # search in rule-base for this rule
                    print('[FR-F-FL]:',sensors, '-> ', rule, ': ', f"{fire_strength:.2f}") # print the fired rule
                    fired_rules.append((rule, fire_strength)) # if the rule is fired it will be append to fired_rules
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
        frs_values, fs_values, fls_values = self.fuzzification(sensor_distances)

        # Inference: For each rule in rule_base calculate its firing_strenght and returns all fired rules with their firing strengh.
        fired_rules = self.calculate_fired_rules(frs_values, fs_values, fls_values)

        # Defuzzification: calculate crisp output for motor's speeds from linguistics variables
        linear_velocity, angular_velocity = self.defuzzification(fired_rules)

        # print(f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}")
        return linear_velocity, angular_velocity

# Robot Codes
def test():

    # Fuzzy sets for the three sensors
    front_sensor_zone = {
        'near': {'shape':'left-trap', 'corners':[0.0, 0.25, 0.75]}, 
        'medium': {'shape':'tri-angle', 'corners':[0.25, 0.75, 0.85]}, 
        'far': {'shape':'right-trap', 'corners':[0.75, 0.85, 1.0]}
    }
    
    front_right_sensor_zone = {
        'near': {'shape':'left-trap', 'corners':[0.0, 0.25, 0.6],}, 
        'medium': {'shape':'tri-angle', 'corners':[0.25, 0.6, 0.7]}, 
        'far': {'shape':'right-trap', 'corners':[0.6, 0.7, 1.0]}
    }

    front_left_sensor_zone = {
        'near': {'shape':'left-trap', 'corners':[0.0, 0.25, 0.6]}, 
        'medium': {'shape':'tri-angle', 'corners':[0.25, 0.6, 0.7]}, 
        'far': {'shape':'right-trap', 'corners':[0.6, 0.7, 1.0]}
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

    # Rule base (right front sensor, front sesnor, left front sensor): (linear motor, angular motor)
    rule_base = { 
        ('near', 'near', 'near'): ('slow', 'left'), # |-| -> Always left
        ('near', 'near', 'medium'): ('slow', 'left'), # | -|
        ('near', 'near', 'far'): ('slow', 'left'), # -|
        ('near', 'medium', 'near'): ('slow', 'left'), # |‾|
        ('near', 'medium', 'medium'): ('slow', 'left'), # | ‾|
        ('near', 'medium', 'far'): ('slow', 'Left'), # ‾|
        ('near', 'far', 'near'): ('slow', 'left'), # |.| (coridoor)-> ('medium', 'front')
        ('near', 'far', 'medium'): ('slow', 'left'), # | .|
        ('near', 'far', 'far'): ('slow', 'left'), # .|
        ('medium', 'near', 'near'): ('slow', 'right'), # |- |
        ('medium', 'near', 'medium'): ('slow', 'left'), # | - |
        ('medium', 'near', 'far'): ('slow', 'left'), # - |
        ('medium', 'medium', 'near'): ('slow', 'right'), # |‾ |
        ('medium', 'medium', 'medium'): ('slow', 'left'), # | ‾ |  -> Always left
        ('medium', 'medium', 'far'): ('slow', 'left'), #  ‾ |
        ('medium', 'far', 'near'): ('slow', 'right'), # |. |
        ('medium', 'far', 'medium'): ('slow', 'left'), # | . | (coridoor)-> ('medium', 'front')
        ('medium', 'far', 'far'): ('slow', 'right'), # . | ('medium', 'left')
        ('far', 'near', 'near'): ('slow', 'left'), # |- ('fast', 'right')
        ('far', 'near', 'medium'): ('slow', 'left'), # | - ('fast', 'right')
        ('far', 'near', 'far'): ('slow', 'left'), # - ('fast', 'left') -> Always left
        ('far', 'medium', 'near'): ('slow', 'left'), # |‾ ('fast', 'right')
        ('far', 'medium', 'medium'): ('slow', 'left'), # | ‾ ('medium', 'right')
        ('far', 'medium', 'far'): ('slow', 'left'), # ‾
        ('far', 'far', 'near'): ('slow', 'right'), # |. ('medium', 'right')
        ('far', 'far', 'medium'): ('slow', 'right'), # | .
        ('far', 'far', 'far'): ('slow', 'front'), # (coridoor)-> ('fast', 'front')
    }
        
    flc = FuzzyController(
        front_sensor_zone = front_sensor_zone, 
        front_right_sensor_zone = front_right_sensor_zone,
        front_left_sensor_zone = front_left_sensor_zone,
        linear_zone = linear_zone, 
        angular_zone = angular_zone,
        rule_base = rule_base
    )

    distances = [(0.1, 0.5, 0.1), (0.26, 0.6, 0.5), (0.7, 3, 1.5)]
    for d in distances:
        x, z = flc.run(d)
        print(x, f"{z:.2f}")





if __name__ == '__main__':
    test()




    # regions_ = {
    #     #LIDAR readings are anti-clockwise, starting at 0 on the right-most edge of the LiDaR FOV.
    #     'front': find_nearest(msg.ranges[0:10]),
    #     'front_right': find_nearest(msg.ranges[310:320]),
    #     'front_left': find_nearest(msg.ranges[40:50])
    #     }
    
    # distances = (regions['front_right'], regions['front'], regions['front_left'])
