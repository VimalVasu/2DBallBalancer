#include <ros.h>
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"

class MotorSpeedsNode
{
public:
    MotorSpeedsNode()
    {
        // Create publishers for the motor speeds
        motor_speedA_pub_ = nh_.advertise<std_msgs::Float32>("motorSpeedA", 10);
        motor_speedB_pub_ = nh_.advertise<std_msgs::Float32>("motorSpeedB", 10);
        motor_speedC_pub_ = nh_.advertise<std_msgs::Float32>("motorSpeedC", 10);

        // Create a subscriber to the point topic
        point_sub_ = nh_.subscribe("green_ball_coords", 10, &MotorSpeedsNode::pointCallback, this);
    }

    void pointCallback(const geometry_msgs::Point::ConstPtr& msg)
    {
        // Extract the x, y, and z values from the Point message
        float x = msg->x;
        float y = msg->y;
        float z = msg->z;
        // -------------Math---------------------
        float thetax;
        float thetay;
        float R = 3.5;
        int spdA;
        int spdB;
        int spdC;
        float kpx = 0.1;
        float kpy = 0.1;
        float kix = 0.1;
        float kiy = 0.1;
        float kdx = 1.6;
        float kdy = 1.6;
        int ballPosX = 100;
        int ballPosY = -100;
        unsigned long lastTime = 0;
        unsigned long timeChange = 0;
        float derivativeX = 0;
        float derivativeY = 0;
        float lastErrorX = 0;
        float lastErrorY = 0;
        float sumErrorX = 0;
        float sumErrorY = 0;
        
        unsigned long now = millis();
        timeChange = now - lastTime;
        derivativeX = (ballPosX - lastErrorX)/timeChange;
        derivativeY = (ballPosY - lastErrorY)/timeChange;

        sumErrorX += lastErrorX;
        sumErrorY += lastErrorY;
	  
        thetax = kpx * ballPosX + kdx * derivativeX + kix * sumErrorX;
        thetay = kpy * ballPosY + kdy * derivativeY + kiy * sumErrorY;
	  
        lastErrorX = ballPosX;
        lastErrorY = ballPosY;

        lastTime = now;
	// -----------End Math-------------------
	
        // Create and publish speed A (e.g., based on x)
        std_msgs::Float32 spdA;
        spdA.data = thetax * R;
        spdA.data = int(spdA.data - (spdA.data % 10));
        spdA.data = 3.0;
        motor_speedA_pub_.publish(spdA);

        // Create and publish speed B (e.g., based on y)
        std_msgs::Float32 spdB;
        spdB.data = (-0.5*thetax + (sqrt(3)/2)*thetay) * R;
        spdB.data = spdB.data - (spdB.data % 10);
        motor_speedB_pub_.publish(spdB);

        // Create and publish speed C (e.g., based on z)
        std_msgs::Float32 spdC;
        spdC.data = (-0.5*thetax - (sqrt(3)/2)*thetay) * R;
        spdC.data = spdC.data - (spdC.data % 10);
        spdC.data = 300;
        motor_speedC_pub_.publish(spdC);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher motor_speedA_pub_;
    ros::Publisher motor_speedB_pub_;
    ros::Publisher motor_speedC_pub_;
    ros::Subscriber point_sub_;
};

int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "motor_speeds_node");

    // Create an instance of MotorSpeedsNode
    MotorSpeedsNode motor_speeds_node;

    // Enter a loop, pumping callbacks
    ros::spin();

    return 0;
}

