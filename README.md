# Sim2Real2Sim: Bridging the Gap Between Simulation and Real-World in Flexible Object Manipulation


### 1. System requirement and installation

##### Ubuntu 16.04 + [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
##### [Kinova Jaco v2](https://github.com/Kinovarobotics/kinova-ros)
##### [Openrave](https://github.com/yueyeyuniao/openrave)

### 2. Steps to run the task in simulation

##### (1)launch the Jaco robot and the world in gazebo (should put the 'launch' and 'world' folders under 'kinova_gazebo' folder)
roslaunch kinova_gazebo robot_launch_with_cable_kinect.launch kinova_robot_type:=j2n6s300
##### (2)launch moveit and use moveit to move the robot to the home position
roslaunch j2n6s300_moveit_config j2n6s300_gazebo_demo.launch
##### (3)run velocity controller node (should put the research folder in your catkin workspace)
rosrun research_s2r kinova_cart_vel
##### (4)run modeling
rosrun research_s2r modeling_sim
##### (5)run task
rosrun research_s2r task_sim

### 3. Transfer the methods from simulation to real-world
##### The science from simulation to real-world are mostly the same.
##### The differences would be:
##### (1)The PID parameters need to be tuned a bit. 
##### (2)The socket (target) pose detection is added in the real world.
##### (3)[Yolo](https://github.com/pjreddie/darknet/wiki/YOLO:-Real-Time-Object-Detection) plus color detection is used in the real world for cable detection which helps with modeling.
##### (4)Gripper is modified in the real world to reach a better grasp of the cable, while in the simulation, a [link attacher](https://github.com/pal-robotics/gazebo_ros_link_attacher) is used instead.

### 4. Update the simulation model by using the real-world data
##### (1)Collecting real-world data (joint values) with Apriltags attached on the cable and adding weights to serve as external force
##### (2)Identifying joint stiffness and damping parameters, and these parameters can be used in the simulation
Openrave is used to help with modeling and calculation. The Openrave model is under ‘openrave_model’ folder. The source code for identifying joint stiffness and damping parameters is located at ‘research_r2s’ folder.

##### Simulation video without correct joint stiffness and damping parameters
![alt-text](https://github.com/yueyeyuniao/Sim2Real2Sim/blob/master/gifs/PlugTask_noDangle.gif)
##### Simulation video with identified joint stiffness and damping parameters
![alt-text](https://github.com/yueyeyuniao/Sim2Real2Sim/blob/master/gifs/PlugTask_sim2real2sim.gif)

### 5. Copyrights
Sim2Real2Sim was developed at the [RIVeR Lab, Northeastern University](http://robot.neu.edu/).

