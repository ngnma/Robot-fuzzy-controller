# Robot-controller
This repository provides a set of controllers for the AgileX Limo robot using ROS 2. The goal is to enable reliable right-edge following, obstacle avoidance, or a combination of both through PID and Fuzzy-Logic–based approaches.
## PID Controller
A classical PID controller for right-edge following.
It maintains a desired distance from the right wall by correcting proportional, integral, and derivative errors at the same time.
## Fuzzy-Logic Controllers
The fuzzy controller processes sensor inputs through three main stages:

- **Fuzzification** : 
Converts crisp sensor measurements into linguistic terms using membership functions.
Example: a right-side distance of 0.1 → classified as “near”.

- **Inference** : 
Matches the current fuzzy inputs against the rule base to determine which rules should fire.
Example: If right side is near → turn left.

- **Defuzzification** : 
Converts the selected linguistic outputs back into a crisp control command.
Example: “left” → angular velocity = 0.3.
### Right-Edge Following
A fuzzy controller that keeps the robot aligned with and following the right wall.
### Obstacle Avoidance
A fuzzy controller that enables the robot to react to and avoid frontal obstacles.
### Multi-Behaviour Controller (Context-Blending)
A combined fuzzy controller that blends right-edge following and obstacle avoidance, enabling the robot to follow the wall while simultaneously reacting to nearby obstacles.
