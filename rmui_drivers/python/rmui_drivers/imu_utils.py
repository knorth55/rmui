from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu


def get_imu_msg(q, v, a, q_covar, v_covar, a_covar):
    orientation = Quaternion(
        x=q[0], y=q[1], z=q[2], w=q[3])
    angular_velocity = Vector3(
        x=v[0], y=v[1], z=v[2])
    linear_acceleration = Vector3(
        x=a[0], y=a[1], z=a[2])
    imu_msg = Imu(
        orientation=orientation,
        orientation_covariance=q_covar,
        angular_velocity=angular_velocity,
        angular_velocity_covariance=v_covar,
        linear_acceleration=linear_acceleration,
        linear_acceleration_covariance=a_covar,
    )
    return imu_msg
