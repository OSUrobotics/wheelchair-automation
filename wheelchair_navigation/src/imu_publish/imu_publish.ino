#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
ros::NodeHandle nh;

std_msgs::Float32MultiArray IMU;
ros::Publisher imu_pub = ros::Publisher("IMU",&IMU);



Adafruit_BNO055 bno = Adafruit_BNO055();


void setup(void)
{
  nh.initNode();
  IMU.data_length = 4;
  IMU.data= malloc(sizeof(int));
  nh.advertise(imu_pub);
  

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }


}

void loop(void)
{
 
 
  imu::Quaternion quat = bno.getQuat();


  IMU.data[0] = quat.x();
  IMU.data[1] = quat.y();
  IMU.data[2] = quat.z();
  IMU.data[3] = quat.w();
  imu_pub.publish( &IMU);
  nh.spinOnce();


}
