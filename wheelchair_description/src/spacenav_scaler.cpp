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

// If SPACENAV_CHECK = 0, the spacenav will constantly publish 0's if unplugged or in resting position
#define SPACENAV_CHECK 1

using namespace std;

class SubscribeAndPublish{
public:
  SubscribeAndPublish(ros::NodeHandle& n){
    velPub = n.advertise<geometry_msgs::Twist>("input/spacenav/cmd_vel", 10);
    velSub = n.subscribe("spacenav/twist", 10, &SubscribeAndPublish::velCallback,this);
  }


  void velCallback(const geometry_msgs::Twist::ConstPtr& oldVel){
    geometry_msgs::Twist newVel;
    double scale = .69;

    newVel.angular.z = oldVel->angular.x/scale * -1;
    newVel.linear.x = oldVel->angular.y/scale;

// Loop designed to prevent spacenav's constant publishing from interfering with lower priority devices
    if((newVel.angular.z != velCheck.angular.z || newVel.linear.x != velCheck.linear.x) && SPACENAV_CHECK){
      velCheck = newVel;
      velPub.publish(newVel);
    }

// Loop part to prevent lower devices taking control if the spacenav is stationary, but not at 0
    else if(newVel.angular.z != 0 && newVel.linear.x != 0 && SPACENAV_CHECK){
      velPub.publish(newVel);
    }

    else if(!SPACENAV_CHECK){
      velPub.publish(newVel);
    }

  }
protected:
  ros::Publisher velPub;
  ros::Subscriber velSub;
  geometry_msgs::Twist velCheck;
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
