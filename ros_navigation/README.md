## ROS 2 Challenge

### Part 1: Introduction to ROS2
**Operating System:** Ubuntu 20.04 LTS or compatible.
**ROS Version:** ROS 2 Foxy Fitzroy.

#### Part A
Set up using the following commands:

```bash
cd ~/ros2_ws_intro/

colcon build --packages-select pub_sub_pkg
```
After that, open two terminals for subscriber and publisher and source the ROS 2 workspace in both of them:

```bash
source ~/ros2_ws_intro/install/setup.bash

# for subscriber
ros2 run pub_sub_pkg subscriber

# for publisher
ros2 run pub_sub_pkg publisher
```

You should see the messages being published and received in the terminals like the video below:

[![Alt text](https://img.youtube.com/vi/YUm7KlRHGXg/0.jpg)](https://www.youtube.com/watch?v=YUm7KlRHGXg)

#### Part B

First install the TurtleBot3 simulation package by running the following command:

```bash
sudo apt-get install ros-foxy-turtlebot3*

# source the ROS 2 environment to update the change
source /opt/ros/foxy/setup.bash
```

To establish the simulation environment, first set the environment variable for the turtlebot3 model like this, and then launch simulation:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Open a new terminal and run rviz2 to visualize the laser scan data:

```bash
source /opt/ros/foxy/setup.bash
rviz2
```
To make sure the rviz2 to visualize the laser scan data, add the **LaserScan** plugin and set the topic to **/scan**. And check the input box for **Global Options -> Fixed Frame -> base_scan**.

The laser scan data is shown in the video below:

[![Alt text](https://img.youtube.com/vi/x4-CRRH7gAo/0.jpg)](https://www.youtube.com/watch?v=x4-CRRH7gAo)

### Part 2: ROS2 Navigation2

Just like the step above, to start the simulation, first set the environment variable for the turtlebot3 model, and then start:

```bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

To activate the navigation function, open a new terminal and run the Navigation2 by:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
# this will launch the navigation2 with the turtlebot3 model applied
```

Once the navigation2 is launched, it will automatically start a rviz window, after set the **2D Pose Estimate**, just simply set the goat by **Navigation2 Goal** and the bot will start to navigate to the goal.

The navigation process is shown in the video below:

[![Alt text](https://img.youtube.com/vi/t9nMUsfQ7pc/0.jpg)](https://www.youtube.com/watch?v=t9nMUsfQ7pc)
