#pragma once
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>

class SnakeControl
{
private:

    double min_range;
    bool right_obst = false; //stena sprava
    bool left_obst = false; //stena sleva
    bool critical_range = false; //stena speredi
    bool nalevo = false; //sobiraus povernut nalevo esli stena speredi
    double max_v;
    double max_w;
    bool change_obst = false; //pora menyat stenu
    bool last_obst_was_left = false; //poslednyaya stena bila sleva
    double wall_curve = 0; //krivizna_steni

public:

    void setLaserData(const std::vector<float>& data);

    void setRobotPose(double x, double y, double theta);

    void getControl(double& v, double& w) ;

    std::string getName() { return "name: SnakeControl"; }

    SnakeControl(double range = 0.6, double maxv = 0.5, double maxw = 0.5):
        min_range(range),
        max_v(maxv),
        max_w(maxw)
    {
        ROS_DEBUG_STREAM("SnakeControl constructor");
    }
};

