# RoboND-GoChaseIt
This is the code for my Udacity Robotics Software Engineer Nanodegree  - Go Chase It. It creates a simulation world with Gazebo which includes a robot equipped with a camera and lidar sensor that can chase a white ball.

### Output
After launching the world the Building, MobileRobots, the Robot (with camera and lidar), and a white ball are displayed inside a Gazebo World. It should launch as follow:
![alt text](images/output.gif)

### Directory Structure
```
    .RoboND-GoChaseIt                  # main folder 
    ├── images                         # Code output image
    │   ├── output.png
    ├── my_robot                       # my_robot package
    │   ├── launch                     # launch folder for launch files
    │   │   ├── robot_description.launch
    │   │   ├── world.launch
    │   ├── meshes                     # meshes folder for sensors
    │   │   ├── hokuyo.dae
    │   ├── urdf                       # urdf folder for xacro files
    │   │   ├── my_robot.gazebo
    │   │   ├── my_robot.xacro
    │   ├── world                      # world folder for world files
    │   │   ├── nancys_world.world
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    ├── ball_chaser                    # ball_chaser package
    │   ├── launch                     # launch folder for launch files
    │   │   ├── ball_chaser.launch
    │   ├── src                        # source folder for C++ scripts
    │   │   ├── drive_bot.cpp
    │   │   ├── process_images.cpp
    │   ├── srv                        # service folder for ROS services
    │   │   ├── DriveToTarget.srv
    │   ├── CMakeLists.txt             # compiler instructions
    │   ├── package.xml                # package info
    └── README.md
```

### Steps to launch the simulation

#### Clone the repository in tha catkin workspace i.e. /home/workspace/catkin_ws/src
```sh
$ cd /home/workspace/catkin_ws/src
$ git clone https://github.com/nancyreschka/RoboND-GoChaseIt.git
```

#### Compile the code
```sh
$ cd /home/workspace/catkin_ws/
$ catkin_make
```

#### Launch the robot inside the world and RViz
```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

#### Launch the Monte Carlo Localization
In a new terminal:
```sh
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot amcl.launch
```

#### Load the configuration file in RViz
The configuration file RvizSettings.rviz can be found in:
```sh
/home/workspace/catkin_ws/my_robot/launch
```

#### OPTIONAL: Run Teleop Package
If you prefer to control your robot to help it localize itself, you would need to add the teleop node to your package. Thanks to the ROS community, we could use ros-teleop package to send command to the robot using keyboard or controller.
In a new terminal clone the ros-teleop package to your src folder:
```sh
$ cd /home/workspace/catkin_ws/src
$ git clone https://github.com/ros-teleop/teleop_twist_keyboard
```
Build the package, source the setup script, and run the package:
```sh
$ cd ..
$ catkin_make
$ source devel/setup.bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

#### Visualize the robot's camera image in RViz
Setup RViz to visualize the sensor readings. On the left side of RViz, under Displays:

* Select odom for fixed frame
* Click the Add button and
  * add RobotModel and your robot model should load up in RViz.
  * add Camera and select the Image topic "/camera/rgb/image_raw" that was defined in the camera Gazebo plugin
  * add LaserScan and select the topic "/scan" - that was defined in the Hokuyo Gazebo plugin