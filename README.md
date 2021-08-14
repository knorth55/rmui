# rmui

Robot Manipulation User Interface (RMUI)

## Dependency

- [RoboticMaterials/FA-I-Sensor](https://github.com/RoboticMaterials/FA-I-sensor/)

## Installation & Build

### Normal Workspace build

```bash
mkdir rmui_ws/src -p
cd rmui_ws/src
wget https://raw.githubusercontent.com/knorth55/rmui/master/fc.rosinstall -o .rosinstall
wstool update -j 2
rosdep install --ignore-src --from-path . -y -r -c
cd ..
catkin build
```

### Device workspace build

```bash
ssh <your device>
mkdir rmui_ws/src
# for device
wget https://raw.githubusercontent.com/knorth55/rmui/master/device.rosinstall.${ROS_DISTRO} -o .rosinstall
wstool update -j 2
rosdep install --ignore-src --from-path . -y -r -c
cd ..
catkin build
```

### Baxter workspace build

```bash
cd rmui_ws/src
wstool merge knorth55/rmui/baxter.rosinstall
wstool update -j 2
rosdep install --ignore-src --from-path . -y -r -c
cd ..
catkin build
```

## Demo launch

### PR2 + RMUI

```bash
roslaunch rmui_demos pr2_rmui.launch
```

### PR2 + dummy RMUI

```bash
roslaunch rmui_demos pr2_rmui_dummy.launch
```

### Baxter + RMUI

```bash
# for real robot
roslaunch eus_vive baxter_moveit_remote.launch
```

```bash
roslaunch rmui_demos baxter_rmui.launch
```

### Baxter + dummy RMUI

```bash
# for real robot
roslaunch eus_vive baxter_moveit_remote.launch
```

```bash
roslaunch rmui_demos baxter_rmui_dummy.launch
```

## RMUI device

### Miniature Tangible Cube

```bash
rosrun rmui_drivers rmui_node.py
```

#### Output

- `~output/proximities`: (`force_proximity_ros/ProximityArray`)

- `~output/imu`: (`sensor_msgs/Imu`)

## Sensors

### VCNL4040: Proxmity sensor

```bash
rosrun rmui_drivers vcnl4040_node.py
```

#### Output

- `~output`: (`force_proximity_ros/ProximityStamped`)

### BNO055: IMU

```bash
rosrun rmui_drivers bno055_node.py
```

#### Output

- `~output`: (`sensor_msgs/Imu`)

### Multiple VCNL4040 + PCA9547: Proxmity sensors

```bash
rosrun rmui_drivers vcnl4040_multiplexa_node.py
```

#### Output

- `~output`: (`force_proximity_ros/ProximityArray`)

## Actuators

### WX281x & OSTW3535C1A

```bash
rosrun rmui_drivers wx281x_node.py
```

#### Input

- ``~input``: (`rmui_msgs/LED`)
