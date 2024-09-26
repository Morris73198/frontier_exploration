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


## Demo using Turtlebot3 (Two Robot exploration)

- **Shell #2** : Gazebo + RViz

```bash
roslaunch 
```
