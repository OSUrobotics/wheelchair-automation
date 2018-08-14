#include <string>
#include <iostream>
#include <cstdio>
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iterator>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "serial/serial.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <regex>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif


class SubscribeAndPublish{
public:
  SubscribeAndPublish(ros::NodeHandle& n){
    velPub = n.advertise<geometry_msgs::Twist>("input/spacenav/cmd_vel", 10);
    velSub = n.subscribe("spacenav/twist", 10, &SubscribeAndPublish::velCallback,this);
  }


  void velCallback(const geometry_msgs::Twist::ConstPtr& oldVel){
    geometry_msgs::Twist newVel;
    double scale = .69;

    newVel.angular.z = oldVel->angular.z/scale;
    newVel.linear.x = oldVel->linear.x/scale;

    velPub.publish(newVel);

  }
protected:
  ros::Publisher velPub;
  ros::Subscriber velSub;
};

int main(int argc, char **argv){

  ros::init(argc, argv, "scaler");

    ros::NodeHandle n;

  SubscribeAndPublish SAPObject(n);

  ros::Rate r(100);

  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
}
