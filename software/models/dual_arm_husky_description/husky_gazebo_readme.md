# Husky Gazebo

## Run
```bash
roslaunch husky_gazebo husky_playpen.launch dual_ur5_enabled:=true robotiq_grippers_enabled:=true laser_enabled:=false
```


## Installation

This assumes you've installed Gazebo 7 and the ``ros-indigo-gazebo*`` and ``ros-indigo-control*`` packages. Make sure to first remove ``ros-indigo-ur-description`` and ``ros-indigo-ur-kinematics`` and ``ros-indigo-flir-description`` if installed.

```bash
sudo apt-get install ros-indigo-pointgrey-camera-description ros-indigo-ekf-localization ros-indigo-twist-mux ros-indigo-robot-localization ros-indigo-interactive-marker-twist-server ros-indigo-ros-controllers ros-indigo-navigation ros-indigo-move-base ros-indigo-soem ros-indigo-serial ros-indigo-rviz-imu-plugin
mkdir -p ~/husky_ws/src
cd ~/husky_ws/src
catkin_init_workspace
git clone https://github.com/husky/husky.git --branch dual_ur5_husky
git clone https://github.com/husky/husky_desktop.git --branch indigo-devel
git clone https://github.com/husky/husky_simulator.git --branch dual_ur5_husky
git clone https://github.com/TheDash/universal_robot.git --branch indigo-devel
git clone https://github.com/TheDash/robotiq --branch ft_300
git clone https://github.com/TheDash/flir_ptu.git --branch fix-gazebo-errors
cd ..
#rosdep install -a # Dont install gazebo2 though, get gazebo7
catkin_make -j9
```

### Environment variables

The following need to be set for the simulation to work:

```
export FLIR_PTU_ENABLED=true
export HUSKY_DUAL_UR5_ENABLED=true
export ROBOTIQ_GRIPPERS_ENABLED=true
export HUSKY_LASER_ENABLED=false
export DUAL_ARM_BULKHEAD=true
export HUSKY_TOP_PLATE_ENABLED=false
export ROBOTIQ_FT_300_ENABLED=true
```