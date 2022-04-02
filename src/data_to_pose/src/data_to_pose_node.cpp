#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Quaternion.h"
//#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TwistStamped.h"
#include "data_to_pose/Controller.h"
using namespace std;

class JoyControl {
    private:
    ros::Subscriber imu_subscriber;
    ros::Subscriber joy_subscriber;
    ros::Subscriber gyr_subscriber;
    //ros::ServiceServer reset_service;
    public:
    float joyX, joyY = 0.0;
    float velX, velY, velZ = 0.0;
    float angleX, angleY, angleZ, angleW = 1.0;
    float angVelX, angVelY, angVelZ = 0.0;
    JoyControl(ros::NodeHandle *nh) {
        
        // imu_subscriber = nh->subscribe("/imu_data", 1000, 
        //      &JoyControl::OrienationCallback, this);
        // gyr_subscriber = nh->subscribe("/gyr_data", 1000, 
        //     &JoyControl::AngVelCallback, this);
        joy_subscriber = nh->subscribe("/joy", 1000, 
            &JoyControl::PositionCallback, this);
        // reset_service = nh->advertiseService("/reset_counter", 
        //     &NumberCounter::callback_reset_counter, this);
    }

    // void AngVelCallback(const  geometry_msgs::Vector3::ConstPtr& msg) {
    //     angVelX = msg->x;
    //     angVelY = msg->y;
    //     angVelZ = msg->z;
    // }
   
    // void OrienationCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
    // {
    //     //geometry_msgs::Quaternion angle;
    //     angleX = msg->x;
    //     angleY = msg->y;
    //     angleZ = msg->z;
    //     angleW = msg->w;
    // }

    void PositionCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
    {
        joyX = (msg->data[0]);
        joyY = (msg->data[1]);
        if (std::abs (joyX) < 15) {
            velX = 0.0;
        } else{
            velX = -joyX * 0.01;
        }
        if (std::abs (joyY) < 15) {
            velY = 0.0;
        } else{
            velY = joyY * 0.01;
        }

        velZ = 0;
        
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_to_pose_node");
    ros::NodeHandle nh;
    JoyControl jc = JoyControl(&nh);
    ros::Publisher twist_publisher = nh.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 1000);
    //ros::Publisher state_publisher = nh.advertise<data_to_pose::Controller>("/position_angle", 1000);
    ros::Rate r(100);     // 100 hz                                                                                                                                        

    while(ros::ok())                                                                                                                                               
    {                                                                                                                                                            
        ros::spinOnce();                                                                                                                                       
        //cout << jc.velX << endl;
        //data_to_pose::Controller state;
        geometry_msgs::TwistStamped twist;
        twist.twist.linear.x = jc.velX;
        twist.twist.linear.y = jc.velY;
        twist.twist.linear.z = 0;//jc.velZ;
        twist.twist.angular.x = 0;//jc.angVelX;
        twist.twist.angular.y = 0;//jc.angVelY;
        twist.twist.angular.z = 0;//jc.angVelZ;
        twist_publisher.publish(twist);                                                                                                 
        r.sleep();                                                                                                                                           
    }              

    //ros::spin();

  return 0;
}