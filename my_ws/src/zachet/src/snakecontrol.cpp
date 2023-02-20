#include "snakecontrol.h"

void SnakeControl::setRobotPose(double x, double y, double theta)
{}

void SnakeControl::setLaserData(const std::vector<float> &data)
{
    critical_range = false;

    int k = 50;
    for (int i = k; i<130; i++)
    {
        if ( data[i] < min_range )
        {
            critical_range = true;
            ROS_WARN_STREAM("wall is too close!!");
            break;
        }
    }

    nalevo = false;
    if (data[51] < data[129]) 
    {
        nalevo = true;
    }

    right_obst = false; 
    left_obst = false;
    change_obst = false;
    wall_curve = 0;

    if (!critical_range)
    {
        if (data[0] < min_range) right_obst = true;
        if (data[179] < min_range) left_obst = true;

        double d = 2.5; //ilnur d=1.5

        if ((data[179] < d * min_range) * (data[0] < d * min_range)) change_obst = true;
        
        if (left_obst)        
            wall_curve = (data[80] > d * min_range) * (data[130] - min_range);        
        else if (right_obst)
            wall_curve = (data[100] > d * min_range) * (data[50] - min_range);
    ROS_ERROR_STREAM(data.size()); 
        if (wall_curve > 1) wall_curve = 1;   
        if (wall_curve < -1) wall_curve = -1;  
    }

}

void SnakeControl::getControl(double& v, double& w)
{
    if (critical_range)
    {
        if (left_obst)
        {
            v = 0.1 * max_v;
            w = -1 * max_w;
        }
        else if (right_obst)
        {
            v = 0.1 * max_v;
            w = 1 * max_w;
        }
        else
        {
            if (nalevo == true)
            {
                ROS_WARN_STREAM("want to go to left");
            }
            else ROS_WARN_STREAM("want to go to right");
            v = 0.1 * max_v;
            w = ((nalevo == true) - (nalevo == false)) * max_w;
        }
    }
    else
    {
        if (change_obst)
        {
            if (last_obst_was_left) ROS_WARN_STREAM("have to change wall! left to right");
            else ROS_WARN_STREAM("have to change wall! right to left");
            
            v = 1 * max_v;
            w = ((last_obst_was_left == false) - (last_obst_was_left == true)) * 1 * max_w; //ilnur vernut wall_curve
        }
        else if (left_obst)
        {
            v = 1 * max_v;
            w = 1 * wall_curve * max_w;
            ROS_WARN_STREAM("proceed near the left wall");
            last_obst_was_left = true;
        }
        else if (right_obst)
        {
            v = 1 * max_v;
            w = -1 * wall_curve * max_w;
            ROS_WARN_STREAM("proceed near the right wall");
            last_obst_was_left = false;
        }
        else
        {
            v = max_v;
            w = 0;
        }
        
    }
ROS_WARN_STREAM(min_range); 
}
