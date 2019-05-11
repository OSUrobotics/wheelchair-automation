#!/usr/bin/python
import rospy
import tf
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class PCFilter:
    def __init__(self):
        self.nh = rospy.init_node('pointcloud_filter', anonymous = True)
        self.rate = 10
        self.sub_topic = 'merged_pointcloud'
        self.filter_pub = rospy.Publisher('filtered_pointcloud', PointCloud, queue_size = 1)
        self.sub = rospy.Subscriber(self.sub_topic, PointCloud, self.sub_callback)
        self.listen = tf.TransformListener()
        self.original_cloud = PointCloud()
        self.new_cloud = PointCloud()
        self.working_cloud = PointCloud()
        self.params = []
        self.r = rospy.Rate(self.rate)

        self.get_params(0)

        self.loop()


    def get_params(self, failures):
        try:
            params_exist = rospy.has_param("/pointcloud_filters")
        except:
            rospy.logfatal("Unable to check for pointcloud_filters param. Exiting.")
            exit()

        if not params_exist:
            if failures < 10:
                rospy.logerr("Could not find pointcloud_filters param. Retrying.")
                rospy.sleep(rospy.Duration(2.0))
                self.get_params(failures + 1)
            else:
                rospy.logfatal("Unable to locate pointcloud_filters param. Exiting.")
                exit()

        elif failures == 0:
            try:
                self.params = rospy.get_param("/pointcloud_filters")
            except:
                rospy.logfatal("Found pointcloud_filters params but could not load. Exiting.")
                exit()

    def sub_callback(self, msg):
        self.original_cloud = msg

    def box_filter(self, param):
        try:
            frame = param['params']['base_frame']
        except:
            rospy.logerr("Could not find base frame for filter ")
            return
        try:
            (trans,rot) = self.listen.lookupTransform(self.original_cloud.header.frame_id, frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        try:

            x_min = param['params']['x_min'] - trans[0]
            x_max = param['params']['x_max'] - trans[0]
            y_min = param['params']['y_min'] - trans[1]
            y_max = param['params']['y_max'] - trans[1]
            z_min = param['params']['z_min'] - trans[2]
            z_max = param['params']['z_max'] - trans[2]

            proc_cloud = PointCloud()

            for point in self.working_cloud.points:
                try:
                    if point.x >= x_min and point.x <= x_max:
                        if point.y >= y_min and point.y <= y_max:
                            if point.z >= z_min and point.z <= z_max:
                                pass
                            else:
                                proc_cloud.points.append(point)
                        else:
                            proc_cloud.points.append(point)
                    else:
                        proc_cloud.points.append(point)
                except:
                    rospy.logerr('Could not process point ')
                    return

            self.working_cloud = proc_cloud

        except:
            rospy.logerr("At least one xyz min or max param was undefined.")
            return



    def one_dim_filter(self, param):
        try:
            frame = param['params']['base_frame']
        except:
            rospy.logerr("Could not find base frame for filter ")
            return
        try:
            (trans,rot) = self.listen.lookupTransform(self.original_cloud.header.frame_id, frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        try:
            dim = param['params']['dimension']
            min = param['params']['min_val']
            max = param['params']['max_val']

        except:
            rospy.logerr("Could not call dimension params")
            return

        proc_cloud = PointCloud()

        try:
            for point in self.working_cloud.points:
                if dim == 'z':
                    if point.z >= min and point.z <= max:
                        proc_cloud.points.append(point)
                elif dim == 'y':
                    if point.y >= min and point.y <= max:
                        proc_cloud.points.append(point)
                elif dim == 'x':
                    if point.x >= min and point.x <= max:
                        proc_cloud.points.append(point)
                else:
                    rospy.logerr("Given dimension was not recognized")
            self.working_cloud = proc_cloud
        except:
            rospy.logerr("Unable to process pointcloud")
            return


    def loop(self):
        while not rospy.is_shutdown():
            self.working_cloud = PointCloud()
            self.working_cloud = self.original_cloud
            for param in self.params:
                try:
                    if param['type'] == 'BoxFilter':
                        self.box_filter(param)
                    elif param['type'] == 'OneDimensionalFilter':
                        self.one_dim_filter(param)
                except:
                    rospy.logerr("'type' attribute is undefined.")
            self.new_cloud = self.working_cloud
            self.new_cloud.header.stamp = rospy.get_rostime()
            self.new_cloud.header.frame_id = self.original_cloud.header.frame_id
            self.filter_pub.publish(self.new_cloud)
            self.r.sleep()


if __name__ == '__main__':
    filter = PCFilter()
    rospy.spin()
