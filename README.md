## ROS Noetic package download
```bash
sudo apt-get update
sudo apt-get install ros-noetic-turtlebot3-gazebo
source /opt/ros/noetic/setup.bash
```

```bash
source /opt/ros/noetic/setup.bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

```bash
sudo apt-get update
sudo apt-get install ros-noetic-turtlebot3-slam
source /opt/ros/noetic/setup.bash
```

```bash
sudo apt-get update
sudo apt-get install ros-noetic-turtlebot3-navigation
source /opt/ros/noetic/setup.bash
```

```bash
sudo apt-get update
sudo apt-get install ros-noetic-dwa-local-planner
```


## Demo using Turtlebot3 (Single Robot exploration)

- **Shell #1** : Gazebo

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

- **Shell #2** : SLAM + RViz

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

- **Shell #3** : move_base

```bash
roslaunch turtlebot3_navigation move_base.launch 
```

- **Shell #4** : Frontier exploration

```bash
roslaunch frontier explore.launch
```


https://www.youtube.com/watch?v=HWd1HZNeWmo


## Demo using Turtlebot3 (Two Robot exploration)

- **Shell #1** : Gazebo + RViz

```bash
roslaunch ros_multi_tb3 2_tb3_house.launch
```
- **Shell #2** : tb3_1 exploration

```bash
roslaunch frontier explore_py1.launch
```

- **Shell #3** : tb3_2 exploration

```bash
roslaunch frontier explore_py2.launch
```
https://youtu.be/VKQtYvcHlus
