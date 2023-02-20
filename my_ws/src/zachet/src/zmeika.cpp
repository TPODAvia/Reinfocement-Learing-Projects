#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/UInt16.h"
#include "snakecontrol.h"

ros::Publisher cmd_pub;

SnakeControl ctrl; //controller

void laserCallback(const sensor_msgs::LaserScan& msg)
{
    ctrl.setLaserData(msg.ranges);
}

void poseCallback(const nav_msgs::Odometry& msg)
{
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;
    double theta = 2*atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
 //   ROS_DEBUG_STREAM("MY POSITION IS X: " << x << " Y: " << y << " theta: " << theta);

    ctrl.setRobotPose(x, y, theta);
}

void timerCallback(const ros::TimerEvent&)
{  
//    ROS_DEBUG_STREAM("on timer "); //mojno udalit

	geometry_msgs::Twist cmd;

    ctrl.getControl(cmd.linear.x, cmd.angular.z);
    
    ROS_DEBUG_STREAM("current command: v = "<<cmd.linear.x<<" w = "<<cmd.angular.z);
    
    cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_node");

    ros::NodeHandle node("~");

    ros::Subscriber laser_sub = node.subscribe("/scan", 1, laserCallback);

    ros::Subscriber pose_sub = node.subscribe("/base_pose_ground_truth", 1, poseCallback);
    
    ros::Timer timer1 = node.createTimer(ros::Duration(0.1), timerCallback);

    cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    ros::spin();

    return 0;
}
