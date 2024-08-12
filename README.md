# Robotica: Simplify Learning Robotics

## Problem Statement: 
Robots are complex and require highly integrated software components to simply command and move a robot. Such components include: Networking, Control, Kinematics, Motion Planning, Collision Avoidance, etc.
These topics are typically taught out of books individually, with lack of application given the surplus amount of requirements needed to get a robot working. Simulators are popular in robot learning, 
however current robot simulators are designed for intermediate-advanced users, and contain a steep learning curve that is challenging to get through without prior knowledge in robotics and programming.
This steep learning curve makes it challenging for students to apply robotics concepts independently, and can be challenging for Professors to integrate such software in their curriculum given the
prerequisites required. Therefore, there is a need at the undergraduate/graduate level for a robotics platform that is easily adoptable by students and faculty, 
and provides a platform that is designed for learning robotics through application. 

## Objective: 
Provide a software framework that abstracts the complexity in robotics and enables students to easily build a simulated robot and write algorithms for the following: 
- Forward, Inverse, and Differential Kinematics 
- Trajectory Planners 
- Path Planners 
- Controllers 

## Framework
The following framework will provide a plugin interface for students to insert custom algorithms they have written for the four topics mentioned above, and an easily adaptable API that students can use to program their robot. The framework will also provide a simulation environment that handles physics and collision checking, as well as tools to visualize transform trees, robot workspace, configuration space, planned paths, arm trajectories, joint trajectories and more. The main interface to this framework will be a GUI that the student can interact with and control aspects of the robot/simulation environment, as well as a python interface where students can write custom algorithms and test. 

### Software Diagram
![image](https://github.com/user-attachments/assets/1657b3c5-60c3-4d06-8ca1-beb23741ff3f)
#### User Interfaces:
- Python Plugin Interface: Users can write python code using a plugin templates and a simplified API to make calls to planners and controllers.
- GUI: Users can interact with the simulation enviorment directly, visualize kinematics and planning aspects, and switch plugins. 

### GUI Overview
![image](https://github.com/user-attachments/assets/a4da12a1-7f87-4492-8381-eccd5b97fcf5)

### Plugins
Users can rapidly develop different kinematic, path planning, trajectory planning, or control algorithms and easily deploy/test such algorithms by loading them in the GUI. Moreover, the simulation environment allows the users to
visualize the different outputs produces by the loaded plugin. 

#### Path Planner Examples
![image](https://github.com/user-attachments/assets/6112d565-2911-4044-bcd7-ab81ac619507)

#### Trajectory Planner Examples
![image](https://github.com/user-attachments/assets/7b3a0f6b-4076-468c-9a64-65448600f804)

### Applications/Behavior Programming
Physics based simulation enables applications such as pick and place and allows users to program meaningful robotics tasks. 
#### Pick
![image](https://github.com/user-attachments/assets/8aed2e82-f4fc-4a47-a64b-67f559db2bf2)
### Place
![image](https://github.com/user-attachments/assets/e303dd8f-3526-4d87-bada-7ba1d2c5091f)


