#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from px_comm.msg import OpticalFlow
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import PyKDL


def pol2cart(theta, rho):
    x = rho * np.cos(theta)
    y = rho * np.sin(theta)
    return x, y

class FlowTransformer(object):
    def __init__(self, base_frame, flow_frame, fixed_frame, flow_to_odom):
        self.listener = tf.TransformListener()
        self.base_frame = base_frame
        self.flow_frame = flow_frame
        self.fixed_frame = fixed_frame
        self.flow_ready = False
        self.last_flow_time = rospy.Time()
        self.last_odom_time = rospy.Time()

        self.flow_to_odom = flow_to_odom

        self.v_t_minus_1 = np.array([0,0,0])

        try:
            self.listener.waitForTransform(
                'odom_temp',
                self.flow_frame,
                rospy.Time(),
                rospy.Duration(5)
            )
            offset = self.listener.lookupTransform('odom_temp', self.flow_frame, rospy.Time())[0]
            base_offset = self.listener.lookupTransform('odom_temp', self.base_frame, rospy.Time())[0]
        except tf.Exception:
            pass


        if not self.flow_to_odom:
            self.x_t = np.array(offset) # use this offset for tracking flow frame from base vels
        else:
            self.x_t = np.array(base_offset) # use this offset for tracking base frame from flow vels

        self.flow_offset = self.listener.lookupTransform(self.flow_frame, self.base_frame, rospy.Time())

        self.r = np.linalg.norm(self.flow_offset[0][:2])
        self.beta = euler_from_quaternion(self.flow_offset[1])[2]
        print self.r

        rospy.Subscriber('flow', OpticalFlow, self.flow_cb)
        rospy.Subscriber('/imu/data', Imu, self.imu_cb)

        self.odom_pub = rospy.Publisher('odom', Odometry)

        self.pose_pub = rospy.Publisher('asdf', PointStamped)

    def imu_cb(self, msg):
        q = PyKDL.Rotation.Quaternion(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.theta_t = q.GetRPY()[2]
        self.omega_t = msg.angular_velocity.z
        self.imu_ang_vel = msg.angular_velocity

    def flow_cb(self, msg):
        now = msg.header.stamp
        dt = (msg.header.stamp - self.last_flow_time).to_sec()
        if self.flow_ready and dt > 0:

            # bind these to local variables because the IMU updates way faster than flow
            theta = self.theta_t
            omega = self.omega_t # multiplying this by 0.6 seems to make things a lot (but not totally) better. Not a clue why.

            A = np.matrix([
                [np.cos(theta), -self.r*np.sin(theta + self.beta)],
                [np.sin(theta),  self.r*np.cos(theta + self.beta)]
            ])
            A_inv = np.matrix([
                [ np.cos(theta + self.beta) / np.cos(self.beta), np.sin(theta + self.beta) / np.cos(self.beta)],
                [-np.sin(theta) / (self.r * np.cos(self.beta)) , np.cos(theta) / (self.r * np.cos(self.beta))]
            ])
            # np.allclose(A.I, A_inv) == True holds, which is good

            if not self.flow_to_odom:
                # this is a weird way to get egocentric velocity, but you can't lookup the egocentric
                # velocity of a frame with reference to itself. It matches the data from odom closely
                base_trans = self.listener.lookupTransformFull('base_footprint', self.last_flow_time, 'base_footprint', now, 'odom_temp')[0]
                base_vel = np.divide(base_trans, dt)

                # The paper doesn't say to, but it looks like it's necessary to have a sign for the normalized velocity,
                # which we can get from the forward component
                # Paper says this is translation speed in the direction of the robot's heading
                # so that should just be the magnitude of the velocity vector, right?
                # since the instantaneous velocity is always tangent to the arc on which
                # the robot is traveling
                V = np.linalg.norm(base_vel[:2]) * np.sign(base_vel[0])

                # Get the front-point's velocity
                vel_dot_f = A * np.matrix([V, omega]).T

                # Integrate to get position
                # CMU paper uses trapezoidal integration - doesn't seem to make a difference here
                # (http://www.ri.cmu.edu/pub_files/2009/7/DilleFSR09.pdf)
                v_t = np.hstack([np.array(vel_dot_f).flatten(), [0]])
                self.x_t = self.x_t + 0.5 * dt * (self.v_t_minus_1 + v_t)

                # Simple numerical integration - doesn't seem any worse than above
                # self.x_t += v_t * dt

                self.v_t_minus_1 = v_t

            else:
                vel = np.array([msg.velocity_x, msg.velocity_y])


                r_omega = self.r * omega
                flow_vel_norm = np.linalg.norm(vel) * np.sign(vel[0])


                # this is just solving for the base vels above...but it doesn't work
                Vx = flow_vel_norm / (np.cos(theta) - r_omega * np.sin(theta + self.beta))
                Vy = flow_vel_norm / (np.sin(theta) + r_omega * np.cos(theta + self.beta))
                # self.x_t += np.array((Vx, Vy, 0)) * dt

                # we can also get V and omega from the matrix inverse
                V = A_inv * np.asmatrix(vel).T
                v_mag = V[0,0] * np.sign(vel[0])
                omega_flow = V[1,0]

                # so now we have linear and angular velocities (v_mag and omega_flow),
                # and a direction should be simple to just push through synchro-drive 
                # forward kinematics which is just a polar to cartesian conversion, 
                # using the IMU's fused orientation estimate

                dx, dy = pol2cart(theta, v_mag * dt)
                # dx, dy = pol2cart(omega_flow * dt, v_mag * dt)

                self.x_t += np.array([dx, dy, 0])

            ps = PointStamped()
            ps.header.frame_id = 'odom_temp'
            ps.header.stamp = now
            ps.point.x, ps.point.y, ps.point.z = self.x_t.tolist()
            self.pose_pub.publish(ps)

        self.flow_ready = True
        self.last_flow_time = msg.header.stamp


if __name__ == '__main__':
    rospy.init_node('flow_odom')
    base_frame = rospy.get_param('~base_frame', 'base_footprint')
    flow_frame = rospy.get_param('~flow_frame', 'flow_cam_link')
    fixed_frame = rospy.get_param('~fixed_frame', 'flow_odom')

    flow_to_odom = rospy.get_param('~flow_to_odom', False)

    FlowTransformer(base_frame, flow_frame, fixed_frame, flow_to_odom)

    rospy.spin()
