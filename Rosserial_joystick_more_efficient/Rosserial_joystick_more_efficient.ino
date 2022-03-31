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
  
  //setup multi-array
  str_msg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  str_msg.data_length = 2;
  str_msg.data = (std_msgs::Int16MultiArray::_data_type*) malloc(2 * sizeof(std_msgs::Int16MultiArray::_data_type));
  
}

void loop()
{
  xPosition = analogRead(VRx);
  yPosition = analogRead(VRy);
  mapX = map(xPosition, 0, 1023, -512, 512);
  mapY = map(yPosition, 0, 1023, -512, 512);
  str_msg.data[0] = mapX;
  str_msg.data[1] = mapY;
  joy_pub.publish( &str_msg );
  nh.spinOnce();
  delay(100);
}
