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
    velPub = n.advertise<geometry_msgs::Twist>("input/nav/cmd_vel", 10);
    velSub = n.subscribe("move_base/cmd_vel", 10, &SubscribeAndPublish::velCallback,this);
  }


  void velCallback(const geometry_msgs::Twist::ConstPtr& oldVel){
    geometry_msgs::Twist newVel;

    // Scale values are based on real world max speeds for x and base_local_planner_params.yaml for z
    double xScale = 2.68224;
    double zScale = 1.75;

    newVel.angular.z = oldVel->angular.z/zScale;
    newVel.linear.x = oldVel->linear.x/xScale;

    velPub.publish(newVel);

  }
protected:
  ros::Publisher velPub;
  ros::Subscriber velSub;
};

int main(int argc, char **argv){

  ros::init(argc, argv, "move_base_scaler");

    ros::NodeHandle n;

  SubscribeAndPublish SAPObject(n);

  ros::Rate r(100);

  while(ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
}
