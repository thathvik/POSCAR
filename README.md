# POSCAR
Implemented NVIDIA DOPE algorithm on CPU to implement a bin picking algorithm. 

## Cloning the whole repo

This repo needs to be added to the source folder in the catkin workspace in your ROS enabled computer (Suggested: Ubuntu 18.04, with ROS-Melodic). 

In `catkin_ws/src`, clone the following:
```bash
$ git clone https://github.com/thathvik/POSCAR.git
```

then move to the directory `/src/POSCAR`, and clone the following repos:
```bash 
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/16292b167108f33cfa93797bcb35c699431766ee

$ git clone https://github.com/fmauch/universal_robot/tree/90c141b4719dd0df87baef12c46c26c374608ba1
```

Download [these files](https://drive.google.com/open?id=1DfoA3m_Bm0fW8tOWXGVxi4ETlLEAgmcg) and place them in the folder `../POSCAR/Deep_Object_Pose/weights/`.

The following programs must be installed for the program to work:
<ol>
  <li>git</li>
  <li>ros-melodic</li>
  <li>MoveIt</li>
  <li>pytorch (cpu version) </li>
  <li>python-pip </li>
  <li>Nvidia CUDA toolkit </li>
</ol>

Move to the catkin workspace folder `catkin_ws` and run:
```bash
$ catkin_make 
```
## Running the program

### Start the Simulaltion environment and DOPE
Run the following command to start the gazebo environment and RViZ:
```bash
$ roslaunch triple_s_util planning_environment.launch sim:=true
```
In the gazebo environment, make the kinematics of the environment_table in the ur5 True. This needs to be done in order for the objects that have gravity enabled to not fall through the table.

In a different Terminal Window, run the following command to start DOPE:
```bash
$ roslaunch triple_s_util dope.launch
```

In another/new Terminal Window, run the following command to spawn a few objects in the Gazebo environment (all gravity enabled):
```bash
$ roslaunch triple_s_util gazebo_spawn_objects_3.launch
```
It is encouraged to try to move the robotic arm around in the RViZ interface and with the command plan and execute, it can move the robot to the requested position in the Gazebo environment. Similarly, the goal position can set to any of the preset values for convinience.

### Try to manipulate the UR5 gripper
For the UR5 robot to execute the operation of identifing the objects and attemp to perform the pick and place, run the two commands in two different terminal windows:
```bash
$ rosrun triple_s_util bin_picking_custom_3.py
$ rosrun triple_s_util find_objects_2.py
```

### To try the functionality of DOPE
Edit the Gazebo environment to add objects and change lighting conditions. With the camera behind the end effector, you can see the image from the camera feed in the Image section (typically found in the bottom left corner) in RViZ. Move the robot to any position you like and try to implement the next few lines of code to get images and see the visuaization of DOPE identified objects. 
```bash
$ rosrun triple_s_util video_to_images.py
```
The above stated command can caputre the video in the time the program runs (stores the images in `POSCAR/triplle_s_util/local_resources/raw_images`). So to get a changing view of the environment, a motion can be executed on the RViZ environment to see different view while the video is being taken.


The next command will input the images taken from the video (all the images stored at `POSCAR/triplle_s_util/local_resources/raw_images`) to the DOPE algorithm, to get images with the cuboids drawn around the objects in the given images.
```bash
$ rosrun triple_s_util images_to_dope.py
```
This command, based on the performance of the Ubuntu machine used, runs the algorithm for a long time. The program can be stopped anytime by pressing Ctrl+C. All the images the program has analized can be accessed from `POSCAR/triplle_s_util/local_resources/dope_images`.
 
 
This repo is for a project for the course Robot Perception, at NYU. 
The repositories used to implement this project are yet to be ackowledged. The author and co-auther of this project do not take complete credit for the work in this repo, since it was built of off a few exsisting Open-source repositories (especially NVIDIA DOPE, and TripleSBin picking). 

Project by:
Tarun Tathvik 
Smrithi Reddy Thudi
