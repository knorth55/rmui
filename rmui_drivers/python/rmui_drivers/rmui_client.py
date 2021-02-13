try:
    import matplotlib.pyplot as plt
except Exception:
    print('please install matplotlib if you use RMUIClient')

import rospy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class RMUIClient(object):
    sensor_positions = [
        # board 0
        (0.05,  0.0,   -0.1),
        (0.0,   0.05,  -0.1),
        (-0.05, 0.0,   -0.1),
        (0.0,   -0.05, -0.1),
        (0.0,   0.0,   -0.1),
        # board 1
        (0.1,   0.05,  0.0),
        (0.1,   0.0,   -0.05),
        (0.1,   -0.05, 0.0),
        (0.1,   0.0,   0.05),
        (0.1,   0.0,   0.0),
        # board 2
        (-0.05, 0.1,   0.0),
        (0.0,   0.1,   -0.05),
        (0.05,  0.1,   0.0),
        (0.0,   0.1,   0.05),
        (0.0,   0.1,   0.0),
        # board 3
        (-0.1,  -0.05, 0.0),
        (-0.1,  0.0,   -0.05),
        (-0.1,  0.05,  0.0),
        (-0.1,  0.0,   0.05),
        (-0.1,  0.0,   0.0),
        # board 4
        (0.05,  -0.1,  0.0),
        (0.0,   -0.1,  -0.05),
        (-0.05, -0.1,  0.0),
        (0.0,   -0.1,  0.05),
        (0.0,   -0.1,  0.0),
        # board 5
        (0.05,  0.0,   0.1),
        (0.0,   -0.05, 0.1),
        (-0.05, 0.0,   0.1),
        (0.0,   0.05,  0.1),
        (0.0,   0.0,   0.1),
    ]
    sensor_directions = (
        '-z', 'x', 'y', '-x', '-y', 'z',
    )

    def __init__(self, node_id, node_name, sensitivity=500):
        super(RMUIClient, self).__init__()
        self.node_id = node_id
        self.node_name = node_name
        self.sensitivity = sensitivity
        self.sys_calib = False
        self.gyr_calib = False
        self.acc_calib = False
        self.mag_calib = False
        self.pub_prx_markers = rospy.Publisher(
            '{}/output/proximities/markers'.format(self.node_name),
            MarkerArray, queue_size=1)
        self.cm = plt.get_cmap('jet')

    def _cb(self, prx_msg, imu_msg, imu_calib_msg):
        header = prx_msg.header
        prx_markers = MarkerArray()
        for prx_id, (proximity, sensor_pos) in enumerate(
                zip(prx_msg.proximities, self.sensor_positions)):
            if proximity.proximity < self.sensitivity:
                continue
            arrow_marker = Marker()
            arrow_marker.header = header
            arrow_marker.ns = self.node_name
            arrow_marker.id = 2 * prx_id
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.lifetime = rospy.Duration(0.5)
            arrow_marker.pose.orientation.w = 1.0
            arrow_marker.scale.x = 0.02
            arrow_marker.scale.y = 0.03
            arrow_marker.color.a = 1.0
            start_point = Point(
                x=sensor_pos[0],
                y=sensor_pos[1],
                z=sensor_pos[2])
            end_point = Point(
                x=sensor_pos[0],
                y=sensor_pos[1],
                z=sensor_pos[2])
            sensor_direction = self.sensor_directions[prx_id // 5]
            arrow_scale = min(
                2000.0, proximity.proximity - self.sensitivity) / 2000.0
            cm_scale = int((0.4 * (1.0 - arrow_scale) + 0.1) * 255)
            if sensor_direction[0] == '-':
                arrow_scale = arrow_scale * -1
            if sensor_direction[-1] == 'x':
                start_point.x = start_point.x + 0.2 * arrow_scale
            elif sensor_direction[-1] == 'y':
                start_point.y = start_point.y + 0.2 * arrow_scale
            else:
                start_point.z = start_point.z + 0.2 * arrow_scale
            arrow_marker.points = [start_point, end_point]
            arrow_marker.color.b = self.cm(cm_scale)[0]
            arrow_marker.color.g = self.cm(cm_scale)[1]
            arrow_marker.color.r = self.cm(cm_scale)[2]
            prx_markers.markers.append(arrow_marker)

            cube_marker = Marker()
            cube_marker.header = header
            cube_marker.ns = self.node_name
            cube_marker.id = 2 * prx_id + 1
            cube_marker.type = Marker.CUBE
            cube_marker.action = Marker.ADD
            cube_marker.lifetime = rospy.Duration(0.5)
            cube_marker.pose.position.x = sensor_pos[0]
            cube_marker.pose.position.y = sensor_pos[1]
            cube_marker.pose.position.z = sensor_pos[2]
            cube_marker.pose.orientation.x = 0.0
            cube_marker.pose.orientation.y = 0.0
            cube_marker.pose.orientation.z = 0.0
            cube_marker.pose.orientation.w = 1.0
            cube_marker.color.b = self.cm(cm_scale)[0]
            cube_marker.color.g = self.cm(cm_scale)[1]
            cube_marker.color.r = self.cm(cm_scale)[2]
            cube_marker.color.a = 1.0
            if sensor_direction[-1] == 'x':
                cube_marker.scale.x = 0.01
                cube_marker.scale.y = 0.1
                cube_marker.scale.z = 0.1
            elif sensor_direction[-1] == 'y':
                cube_marker.scale.x = 0.1
                cube_marker.scale.y = 0.01
                cube_marker.scale.z = 0.1
            else:
                cube_marker.scale.x = 0.1
                cube_marker.scale.y = 0.1
                cube_marker.scale.z = 0.01
            prx_markers.markers.append(cube_marker)

        self.pub_prx_markers.publish(prx_markers)

        self.sys_calib = (imu_calib_msg.system >= 3)
        self.gyr_calib = (imu_calib_msg.gyroscope >= 3)
        self.acc_calib = (imu_calib_msg.accelerometer >= 3)
        self.mag_calib = (imu_calib_msg.magnetometer >= 3)
        if not self.sys_calib:
            rospy.loginfo_throttle(
                1.0,
                'Device {} is not calibrated: system status {}'.format(
                    self.node_id, imu_calib_msg.system))
            return
        if not self.gyr_calib:
            rospy.loginfo_throttle(
                1.0,
                'Device {} is not calibrated: gyroscope status {}'.format(
                    self.node_id, imu_calib_msg.gyroscope))
            return
