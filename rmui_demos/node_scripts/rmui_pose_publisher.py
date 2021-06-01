#!/usr/bin/env python

import rospy

import tf2_ros

from geometry_msgs.msg import PoseStamped


class RMUIPosePublisher(object):
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.parent_frame_id = rospy.get_param(
            '~parent_frame_id', 'world')
        self.rmui_frame_id = rospy.get_param(
            '~rmui_frame_id', 'rmui_link')
        self.update_rate = rospy.Duration(
            rospy.get_param('~update_rate', 0.1))
        self.timer = rospy.Timer(self.update_rate, self._cb)
        self.pub = rospy.Publisher('~output', PoseStamped, queue_size=1)

    def _cb(self, event):
        if event.last_real:
            timestamp = event.last_real
        else:
            timestamp = event.current_real - self.update_rate
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.parent_frame_id, self.rmui_frame_id, rospy.Time(0))
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr_throttle(
                60.0, 'tf2_ros.ExtrapolationException: {}'.format(e))
            return
        except tf2_ros.ConnectivityException as e:
            rospy.logerr_throttle(
                60.0, 'tf2_ros.ConnectivityException: {}'.format(e))
            return
        except tf2_ros.LookupException as e:
            rospy.logerr_throttle(
                60.0, 'tf2_ros.LookupException: {}'.format(e))
            return

        translation = transform_stamped.transform.translation
        rotation = transform_stamped.transform.rotation
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.parent_frame_id
        pose_stamped.header.stamp = timestamp
        pose_stamped.pose.position.x = translation.x
        pose_stamped.pose.position.y = translation.y
        pose_stamped.pose.position.z = translation.z
        pose_stamped.pose.orientation = rotation
        self.pub.publish(pose_stamped)


if __name__ == '__main__':
    rospy.init_node('rmui_pose_publisher')
    app = RMUIPosePublisher()
    rospy.spin()
