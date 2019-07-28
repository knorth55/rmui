# rmui
Robot Manipulation User Interface (RMUI)

## Installation & Build

```
mkdir rmui_ws/src -p
cd rmui_ws/src
wget https://raw.githubusercontent.com/knorth55/rmui/master/kinetic.rosinstall?token=ACG6QX5SPCICFKYXZK6VR725H2OKI -o .rosinstall
wstool update -j 2
rosdep install --ignore-src --from-path . -y -r -c
cd ..
catkin build
```

## Dependency

- [knorth55/FA-I-Sensor](https://github.com/knorth55/FA-I-sensor/)

## Sensors

### VCNL4040: Proxmity sensor

```bash
rosrun rmui_drivers vcnl4040_node.py
```

**output**

- `~output`: (`force_proximity_ros/ProximityStamped`)

### BNO055: IMU

```bash
rosrun rmui_drivers bno055_node.py
```

**output**

- `~output`: (`sensor_msgs/Imu`)