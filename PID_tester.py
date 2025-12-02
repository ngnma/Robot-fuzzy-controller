#!/usr/bin/env python3
class PIDController:
    def __init__(self, desired_distance = 0.4):
        # PID constants
        self.kp = 0 #1
        self.ki = 0 #0.05
        self.kd = 0 #0.2
        self.ep = 0
        self.ei = 0
        self.ed = 0
        self.desired_distance = desired_distance

    def run(self, current_distance):
        e = self.desired_distance - current_distance # calculate new error
        self.ei += e # update integral error (accumulative error)
        self.ed = e - self.ep # calculate derivative error
        self.ep = e # update previous error for next movement
        pid = self.kp * self.ep + self.ki * self.ei + self.kd * self.ed # pid is final angular speed

        print(f"Error: {e:.2f}||e_i: {self.ei:.2f}||e_d: {self.ed:.2f}||e_p: {self.ep:.2f} -> pid: {pid:.2f}")

        return pid


# Robot Codes

def test():
    
    flc = PIDController()

    distances = [0.1, 0.5, 0.26, 0.6, 0.7,3]
    for d in distances:
        pid = flc.run(d)

if __name__ == '__main__':
    test()

