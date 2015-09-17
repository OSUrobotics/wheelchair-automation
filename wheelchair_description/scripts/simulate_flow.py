#!/usr/bin/env python

import rospy
import tf
import numpy as np
from px_comm.msg import OpticalFlow

if __name__ == '__main__':
    rospy.init_node('simulate_flow')
    l = tf.TransformListener()
    last_time = rospy.Time()
    r = rospy.Rate(25)
    last_pos = np.array((0,0,0))
    flow_pub = rospy.Publisher('flow', OpticalFlow)
    flow = OpticalFlow()
    flow.header.frame_id = 'flow_sensor'
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            past = now - r.sleep_dur
            l.waitForTransform('flow_sensor', 'odom_temp', now, rospy.Duration(5))
            trans = l.lookupTransformFull('flow_sensor', past, 'flow_sensor', now, 'odom_temp')[0]
            vel = np.asarray(trans) / (now - last_time).to_sec()
            flow.header.stamp = now
            flow.velocity_x = vel[0]
            flow.velocity_y = vel[1]
            # print np.linalg.norm([vel[0], vel[1]])
            flow_pub.publish(flow)

            last_time = now
        except Exception as e:
            print e

        r.sleep()