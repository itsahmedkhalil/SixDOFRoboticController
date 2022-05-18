#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#define VRx A0    
#define VRy A1
#define B0 A2
#define B1 10     //Red button connected to D12
#define B2 8     //Blue button connected to D11
int xPosition;
int yPosition;
int joyButton;
int zPos;
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
  pinMode(B0, INPUT);
  pinMode(B1, INPUT);
  digitalWrite(B1, HIGH);
  pinMode(B2, INPUT);
  digitalWrite(B2, HIGH);
  offsetX = analogRead(VRx);
  offsetY = analogRead(VRy);
  
  //setup multi-array
  str_msg.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  str_msg.data_length = 4;
  str_msg.data = (std_msgs::Int16MultiArray::_data_type*) malloc(2 * sizeof(std_msgs::Int16MultiArray::_data_type));
  
}

void loop()
{
  xPosition = analogRead(VRx);
  yPosition = analogRead(VRy);
  joyButton = analogRead(B0);
  if (joyButton < 150){
    joyButton = 1;
    }
//  else if (joyButton < 150 && digitalRead(B2)==LOW){
//    joyButton = -1;
//    }
  else {
    joyButton = 0;
    }
  //if the red button is pressed, go up
  if (digitalRead(B1)==LOW && digitalRead(B2)==HIGH){
    zPos = 1;
    }
  //if the blue button is pressed, go down
  else if (digitalRead(B1)==HIGH && digitalRead(B2)==LOW){
    zPos = -1;
    }
  else{
    zPos = 0;
    }
  mapX = map(xPosition, 0, 1023, -512, 512);
  mapY = map(yPosition, 0, 1023, -512, 512);
  str_msg.data[0] = mapX;
  str_msg.data[1] = mapY;
  str_msg.data[2] = zPos;
  str_msg.data[3] = joyButton;
  joy_pub.publish( &str_msg );
  nh.spinOnce();
  delay(20);
}
