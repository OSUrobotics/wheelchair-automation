#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from px_comm.msg import OpticalFlow
from copy import deepcopy
import numpy as np
from tf.transformations import quaternion_about_axis, euler_from_quaternion, quaternion_from_euler
from tf_conversions import fromTf, toTf
import PyKDL


class FlowTransformer(object):
    def __init__(self, base_frame, flow_frame, fixed_frame):
        self.base_frame = base_frame
        self.flow_frame = flow_frame
        self.fixed_frame = fixed_frame
        self.flow_ready = False
        self.last_flow_time = rospy.Time()

        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.transformer = tf.Transformer(interpolating=True)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = fixed_frame

        self.odom_tf = TransformStamped()
        self.odom_tf.header.frame_id = self.fixed_frame
        self.odom_tf.child_frame_id = self.base_frame
        self.odom_tf.transform.rotation.w = 1

        # linear velocity from flow
        self.v_t = np.array((0, 0, 0))
        # angular velocity from IMU
        self.omega_t = np.array((0, 0, 0))

        # x,y,z position state vector
        self.x_t = np.array((0, 0, 0))
        self.theta_t = 0

        print self.base_frame, self.flow_frame
        self.listener.waitForTransform('odom_temp', self.flow_frame, rospy.Time(), rospy.Duration(5))
        offset = self.listener.lookupTransform('odom_temp', self.flow_frame, rospy.Time())[0]
        self.x_t = np.array(offset)

        rospy.Subscriber('flow', OpticalFlow, self.flow_cb)
        rospy.Subscriber('/imu/data', Imu, self.imu_cb)

        rospy.Timer(rospy.Duration(0.1), self.publish_odom)

    def publish_odom(self, _):
        self.odom_tf.header.stamp = rospy.Time.now()

        rot = quaternion_from_euler(0, 0, self.theta_t)
        self.broadcaster.sendTransform(
            self.x_t, rot, rospy.Time.now(), 'flodom', 'odom_temp'
        )

    def imu_cb(self, msg):
        q = PyKDL.Rotation.Quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.theta_t = q.GetRPY()[2]
        self.omega_t = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

    def flow_cb(self, msg):
        dt = (msg.header.stamp - self.last_flow_time).to_sec()
        if self.flow_ready and dt > 0:
            vel = np.array([msg.velocity_x, msg.velocity_y])
            v_body = np.linalg.norm(vel)

            # sign is taken from the sign of the egocentric forward velocity
            sign = -np.sign(vel[0])
            v_body = sign * v_body

            # this is a unit vector pointing in the direction of the robot's orientation
            # in the odom frame
            u_hat_t = np.array([np.cos(self.theta_t), np.sin(self.theta_t), 0])
            v_t1 = v_body * u_hat_t

            x_t1 = self.x_t + 0.5 * dt * (v_t1 + self.v_t)

            self.v_t = v_t1
            self.x_t = x_t1

        self.flow_ready = True
        self.last_flow_time = msg.header.stamp

    def send_transform(self, transform):
        self.broadcaster.sendTransform(
            (transform.transform.translation.x,
             transform.transform.translation.y,
             transform.transform.translation.z),
            (transform.transform.rotation.x,
             transform.transform.rotation.y,
             transform.transform.rotation.z,
             transform.transform.rotation.w),
            rospy.Time.now(),
            transform.child_frame_id,
            transform.header.frame_id
        )


if __name__ == '__main__':
    rospy.init_node('flow_odom')
    base_frame = rospy.get_param('~base_frame', 'base_link')
    flow_frame = rospy.get_param('~flow_frame', 'flow_cam_link')
    fixed_frame = rospy.get_param('~fixed_frame', 'flodom')

    FlowTransformer(base_frame, flow_frame, fixed_frame)

    rospy.spin()
