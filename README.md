# AERIAL ROBOTIC

## :package: About

This package contains the code to create and run the HOMEWORK3 of RoboticLab_2025

## :white_check_mark: Usage Homework-Point-1

Point_1a : Add the Custom folder into PX4-Autopilot/Tools/simulation/gz/models  folder;
Point1_b : Add the airframe file 4000_gz_custom anche the new CMakeList into PX4-Autopilot/ROMFS/px4fmu common/init.d-posix/airframes;

After opening QGroundControl run this command in the terminal to takeoff the new drone: 

```
make px4_sitl gz_custom
```

## :white_check_mark: Usage Homework-Point-2

After opening QGroundControl run this command in the terminal to takeoff the new drone :

```
make px4_sitl gz_custom
```
In another terminal run the DDS_run.sh :
```
. DDS_run.sh
```

After run the force_land node use this command : 

```
ros2 run force_land force_land
```

## :white_check_mark: Usage Homework-Point-3

The Point3 is in the folder offboard_rl:

After opening QGroundControl run this command in the terminal to takeoff the new drone :

```
make px4_sitl gz_custom
```
In another terminal run the DDS_run.sh :
```
. DDS_run.sh
```
After run the node go_to_point with trajectory planner using the offboard mode :
```
ros2 run offboard_rl go_to_point
```













