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

        self.fuzzy_sets = {'near': {'shape':'left-trap', 'corners':[0, 0.25, 0.5]}, 
                           'medium': {'shape':'tri-angle', 'corners':[0.25, 0.5, 0.75]}, 
                           'far': {'shape':'right-trap', 'corners':[0.5, 0.75, 10]}}
        
    def calculate_membership(self, x):
        # This dist contains the degree of membership of each set for x.
        # output exp. {'near': 0, 'medium': 0, 'far': 1}
        outputs = dict()

        for fs in self.fuzzy_sets.items():
            fuzzy_set = fs[0]
            shape = fs[1]['shape']
            corners = fs[1]['corners']
            a, b, c = corners[0], corners[1], corners[2]

            if shape == 'left-trap':
                if x <= b:
                    outputs[fuzzy_set] = 1
                elif b < x <= c:
                    outputs[fuzzy_set] = (c-x)/(c-b)
                else:
                    pass

            if shape == 'tri-angle':
                if a < x <= b:
                    outputs[fuzzy_set] = (x-a)/(b-a)
                elif b < x <= c:
                    outputs[fuzzy_set] = (c-x)/(c-b)
                else:
                    pass

            if shape == 'right-trap':
                if a < x <= b:
                    outputs[fuzzy_set] = (x-a)/(b-a)
                elif b < x:
                    outputs[fuzzy_set] = 1
                else:
                    pass

        return outputs
    


    # def calculate_membership(self, x, zones):
    #     # This dist contains the degree of membership of each set for x.
    #     # output exp. {'near': 0, 'medium': 0, 'far': 1}
    #     outputs = dict()

    #     for fuzzy_set, corners in zones.items():

    #         a, b, c = corners[0], corners[1], corners[2]

    #         if fuzzy_set == 'near':
    #             if x <= b:
    #                 outputs[fuzzy_set] = 1
    #             elif b < x <= c:
    #                 outputs[fuzzy_set] = (c-x)/(c-b)
    #             else:
    #                 pass

    #         if fuzzy_set == 'medium':
    #             if a < x <= b:
    #                 outputs[fuzzy_set] = (x-a)/(b-a)
    #             elif b < x <= c:
    #                 outputs[fuzzy_set] = (c-x)/(c-b)
    #             else:
    #                 pass

    #         if fuzzy_set == 'far':
    #             if a < x <= b:
    #                 outputs[fuzzy_set] = (x-a)/(b-a)
    #             elif b < x:
    #                 outputs[fuzzy_set] = 1
    #             else:
    #                 pass

    #     return outputs
    

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

        rfs_values = self.calculate_membership(rfs_distance)
        rbs_values = self.calculate_membership(rbs_distance)
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

# Robot Codes

if __name__ == '__main__':
    fuzzy_flc = fuzzy_controller()

    distances = [(0.1, 0.5), (0.26, 0.8), (0.7,3)]
    for d in distances:
        x, z = fuzzy_flc.run(d)
        print(x,z)

    # fuzzy_sets = {'near': {'shape':'left-trap', 'corners':[0, 0.25, 0.5]}, 
    #                        'medium': {'shape':'tri-angle', 'corners':[0.25, 0.5, 0.75]}, 
    #                        'far': {'shape':'right-trap', 'corners':[0.5, 0.75, 10]}}
    # # print(fuzzy_sets['near']['corners'][1])
    # for fs in fuzzy_sets.items():
    #     print(fs[1]['shape'])

    # main()
