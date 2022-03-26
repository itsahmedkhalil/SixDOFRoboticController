#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#define VRx A0
#define VRy A1
int xPosition;
int yPosition;
int offsetX;
int offsetY;
int mapX;
int mapY;

ros::NodeHandle  nh;
int value[2]={1,6};
std_msgs::Int16MultiArray str_msg;
ros::Publisher joy_pub("/joy", &str_msg);

void setup()
{
  nh.initNode();
  nh.advertise(joy_pub);
  pinMode(VRx, INPUT);
  pinMode(VRy, INPUT);
  offsetX = analogRead(VRx);
  offsetY = analogRead(VRy);
  
}

void loop()
{
  xPosition = analogRead(VRx);
  yPosition = analogRead(VRy);
  mapX = map(xPosition, 0, 1023, -512, 512);
  mapY = map(yPosition, 0, 1023, -512, 512);
  value[0] = mapX;
  value[1] = mapY;
  str_msg.data = value;
  str_msg.data_length = 2;
  joy_pub.publish( &str_msg );
  nh.spinOnce();
  delay(100);
}
