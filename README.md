# scout_simulator_with_robosense
simulator for scout

## clone & catkin_make
```sh
mkdir scout_ws && cd scout_ws
mkdir src && cd src
git clone --recursive https://github.com/Steve-Mr/scout_simulator_with_robosense.git .
cd ..
rosdep install --from-paths src --ignore-src -r -y 
catkin_make
```

## launch

```sh
# launch scout v2 robot with rs-16 lidar in gazebo
roslaunch scout_gazebo_sim scout_empty_world.launch

# launch pointcloud_to_laserscan
roslaunch pointcloud_to_laserscan point_to_scan.launch

# launch navigation related nodes
roslaunch navigation move_base.launch
```
