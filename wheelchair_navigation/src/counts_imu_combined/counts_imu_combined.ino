#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <wheelchair_navigation_msgs/imu_enc_merged.h>

ros::NodeHandle nh;

std_msgs::Int32MultiArray counts;
//ros::Publisher counts_pub("counts",&counts);

std_msgs::Float32MultiArray IMU;
//ros::Publisher imu_pub = ros::Publisher("IMU",&IMU);
imu::Quaternion quat;

wheelchair_navigation_msgs::imu_enc_merged merged;

ros::Publisher combined_pub = ros::Publisher("merged_imu_encoder", &imu_enc_merged)

Adafruit_BNO055 bno = Adafruit_BNO055();



volatile long signed int tempL, counterL = 0; //This variable will increase or decrease depending on the rotation of encoder
volatile long signed int tempR, counterR = 0; //This variable will increase or decrease depending on the rotation of encoder

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  counts.data_length = 2;
  IMU.data_length = 4;
  IMU.data = malloc(sizeof(int));
  counts.data = malloc(sizeof(int));
  merged.imu_data = malloc(sizeof(IMU));
  merged.counts = malloc(sizeof(counts));
//  nh.advertise(imu_pub);
//  nh.advertise(counts_pub);

  nh.advertise(combined_pub)

  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2
  pinMode(3, INPUT_PULLUP); // internal pullup input pin 3
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);

  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  attachInterrupt(5, ai2, RISING);
  attachInterrupt(4, ai3, RISING);
}



void loop() {
  quat = bno.getQuat();
  IMU.data[0] = quat.x();
  IMU.data[1] = quat.y();
  IMU.data[2] = quat.z();
  IMU.data[3] = quat.w();

  // Send the value of counter
  if(counterR != tempR || counterL != tempL ){
    tempL = counterL;
    tempR = counterR;
    counterL = counterL + 1;
    counts.data[0]=counterL;
    counts.data[1]=counterR;
  }

  merged.imu_data = IMU;
  merged.counts = counts;

  combined_pub.publish(&merged);

//  imu_pub.publish( &IMU);
//  counts_pub.publish( &counts );

  nh.spinOnce();

}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    counterR++;
  }
  else{
    counterR--;
  }
}

void ai1() {
  // ai1 is activated if DigitalPin nr 3  is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    counterR--;
  }
  else{
    counterR++;
  }
}

void ai2() {
  // ai2 is activated if DigitalPin nr 19 is going from LOW to HIGH
  // Check with pin 19 to determine the direction
  if(digitalRead(19)==LOW) {
    counterL--;
  }
  else{
    counterL++;
  }
}

void ai3() {
  // ai3 is activated if DigitalPin nr 18 is going from LOW to HIGH
  // Check with pin 18 to determine the direction
  if(digitalRead(18)==LOW) {
    counterL++;
  }
  else{
    counterL--;
  }
}
