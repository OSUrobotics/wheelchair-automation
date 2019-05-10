#!/usr/bin/python

import rospy
import math
import tf
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32

class Merger:
    def __init__(self):
        self.nh = rospy.init_node('pointcloud_merger', anonymous = True)
        self.pc_pub = rospy.Publisher('merged_pointcloud', PointCloud, queue_size = 1)
        self.pc_message = PointCloud()
        self.rate = 30
        self.base_frame = 'base_link'
        self.data_raw = []
        self.pc_data = []
        self.pc_merged = []
        self.params = []
        self.subscribers = []
        self.listen = tf.TransformListener()
        self.get_params(0)

    #Creates a subscriber and dictionary entry for each separate topic listed in params
        for param in self.params:
            try:
                if param['type'] == "LaserScan":
                    new_dict = {'frame_id': param['params']['frame_id'], 'type' : param['type'], 'data' : [], 'angle_min' : 0.0, 'angle_increment' : 0.0}
                else:
                    new_dict = {'frame_id': param['params']['frame_id'], 'type' : param['type'], 'data' : []}
            except:
                rospy.logerr("Parameter 'type' is undefined")
                pass
            try:
                self.data_raw.append(new_dict)
            except:
                rospy.logerr("Parameter type '" + param['type'] + "' is invalid.")
                pass
            self.pc_data.append([])
            if param['type'] == "LaserScan":
                self.subscribers.append(rospy.Subscriber(param['params']['topic'], LaserScan, self.laser_callback))
            elif param['type'] == "PointCloud":
                self.subscribers.append(rospy.Subscriber(param['params']['topic'], PointCloud, self.pc_callback))

        self.r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.loop()
            self.r.sleep()

    #Checks for pointcloud_merger params. Retries up to 10 times at 2 second intervals
    #before crashing with fatal error.
    def get_params(self, failures):
        try:
            params_exist = rospy.has_param("/pointcloud_merger")
        except:
            rospy.logfatal("Unable to check for pointcloud_merger param. Exiting.")
            exit()

        if not params_exist:
            if failures < 10:
                rospy.logerr("Could not find pointcloud_merger param. Retrying.")
                rospy.sleep(rospy.Duration(2.0))
                self.get_params(failures + 1)
            else:
                rospy.logfatal("Unable to locate pointcloud_merger param. Exiting.")
                exit()

        elif failures == 0:
            try:
                self.params = rospy.get_param("/pointcloud_merger")
            except:
                rospy.logfatal("Found pointcloud_merger params but could not load. Exiting.")
                exit()

    #Converts laser callback data to points and stores in appropriate dictionary
    def laser_callback(self, msg):
        frame = msg.header.frame_id
        for i in range(len(self.data_raw)):
            try:
                if self.data_raw[i]['frame_id'] == frame:
                    self.data_raw[i]['data'] = msg.ranges
                    self.data_raw[i]['angle_min'] = msg.angle_min
                    self.data_raw[i]['angle_increment'] = msg.angle_increment
            except:
                continue


    def pc_callback(self, msg):
        print msg


    def loop(self):
        for i in range(len(self.data_raw)):
            try:
                frame = self.data_raw[i]['frame_id']
                data = self.data_raw[i]['data']
                (trans,rot) = self.listen.lookupTransform(self.base_frame, frame, rospy.Time(0))
                try:
                    if self.data_raw[i]['type'] == 'LaserScan':
                        angle_increment = self.data_raw[i]['angle_increment']
                        angle_min = self.data_raw[i]['angle_min']
                        for j in range(len(data)):
                            if not math.isnan(data[j]):
                                point = Point32()
                                dist = data[j]
                                e_rot = tf.transformations.euler_from_quaternion(rot)
                                theta = float(j) * angle_increment + angle_min - e_rot[2]
                                point.x = -(dist * math.cos(theta) - trans[0])
                                point.y = -(dist * math.sin(theta) - trans[1])
                                point.z = -trans[2]
                                #Makes sure there are enough spots in the data array
                                if i > len(self.pc_data):
                                    self.pc_data.append([])
                                if j > len(self.pc_data[i]) - 1:
                                    self.pc_data[i].append(point)
                                else:
                                    self.pc_data[i][j] = point

                except:
                    print "Error\n"
                    pass

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        self.pc_merged = []

        for i in range(len(self.pc_data)):
            for j in range(len(self.pc_data[i])):
                self.pc_merged.append(self.pc_data[i][j])

        self.pc_message.header.frame_id = self.base_frame
        self.pc_message.header.stamp = rospy.get_rostime()
        self.pc_message.points = self.pc_merged

        self.pc_pub.publish(self.pc_message)



if __name__ == '__main__':
    merger = Merger()
    rospy.spin()
