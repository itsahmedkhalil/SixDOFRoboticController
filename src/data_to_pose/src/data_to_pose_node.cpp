#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "data_to_pose/Controller.h"
using namespace std;

class JoyControl {
    private:
    ros::Subscriber imu_subscriber;
    ros::Subscriber joy_subscriber;
    //ros::ServiceServer reset_service;
    public:
    float velX, velY, velZ = 0.0;
    float angleX, angleY, angleZ, angleW = 1.0;
    JoyControl(ros::NodeHandle *nh) {
        
        imu_subscriber = nh->subscribe("/imu_data", 1000, 
            &JoyControl::OrienationCallback, this);
        joy_subscriber = nh->subscribe("/joy", 1000, 
            &JoyControl::PositionCallback, this);
        // reset_service = nh->advertiseService("/reset_counter", 
        //     &NumberCounter::callback_reset_counter, this);
    }

   
    void OrienationCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
    {
        //geometry_msgs::Quaternion angle;
        angleX = msg->x;
        angleY = msg->y;
        angleZ = msg->z;
        angleW = msg->w;
    }

    void PositionCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
    {
        velX = msg->data[0];
        velY = msg->data[1];
        velZ = 0;
    }

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_to_pose_node");
    ros::NodeHandle nh;
    JoyControl jc = JoyControl(&nh);
    //ros::Publisher twist_publisher = nh.advertise<geometry_msgs::Twist>("/position_angle", 1000);
    ros::Publisher state_publisher = nh.advertise<data_to_pose::Controller>("/position_angle", 1000);
    ros::Rate r(100);     // 100 hz                                                                                                                                        

    while(ros::ok())                                                                                                                                               
    {                                                                                                                                                            
        ros::spinOnce();                                                                                                                                       
        //cout << jc.velX << endl;
        data_to_pose::Controller state;
        state.linearvel.x = jc.velX;
        state.linearvel.y = jc.velY;
        state.linearvel.z = jc.velZ;
        state.orientation.x = jc.angleX;
        state.orientation.y = jc.angleY;
        state.orientation.z = jc.angleZ;
        state.orientation.w = jc.angleW;
        state_publisher.publish(state);                                                                                                  
        r.sleep();                                                                                                                                           
    }              

    //ros::spin();

  return 0;
}