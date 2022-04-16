#include "ros/ros.h"
// #include <iostream>
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

class FollowPath
{
public:
    FollowPath()
    {
        this->node = node;

        ROS_INFO("Publishing Follow Path Node Data...");
        this->twistPublisher = node.advertise<geometry_msgs::Twist>("hexapod/teleop/twist", 1);
        this->buttonPublisher = node.advertise<std_msgs::Bool>("hexapod/teleop/button", 1);

        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;
        button_msg.data = false;

        start = ros::Time::now().toSec();
        double publish_rate = 0.1; // 10Hz
        timer = node.createTimer(ros::Duration(publish_rate), boost::bind(&FollowPath::publishPath, this));

        ROS_INFO("Follow Path Node ready.\n");
    }

    ~FollowPath()
    {
        this->node.shutdown();
    }

private:

    ros::NodeHandle node;
    ros::Publisher twistPublisher;
    ros::Publisher buttonPublisher;
    geometry_msgs::Twist twist_msg;
    std_msgs::Bool button_msg;
    ros::Timer timer;
    boost::mutex publish_mutex;
    double start, t, elapsed, last_elapsed, Tc;

    void publishPath()
    {
        boost::mutex::scoped_lock lock(publish_mutex);

        elapsed = ros::Time::now().toSec() - start;
        Tc = elapsed - last_elapsed; // control interval
        last_elapsed = elapsed;

        twist_msg = pathScript(elapsed);
        twistPublisher.publish(twist_msg);
        
        // B button unused for now
        // button_msg.data = false;
        // buttonPublisher.publish(button_msg);

        ROS_INFO("Lx: %f, Ly: %f, Rx: %f, B: %d",
            twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.x, button_msg.data);
    }

    geometry_msgs::Twist pathScript(const double& elapsed)
    {
        geometry_msgs::Twist twist_msg;

        if (elapsed <= 5.0)
        {
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.x = 1.0;
        }
        else if (elapsed <= 6.0)
        {
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.x = 0.0;
        }
        else if (elapsed <= 8.0)
        {
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.x = -1.0;
        }
        else
        {
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.x = 0.0;
        }

        return twist_msg;
    }
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Follow Path Node...");
    ros::init(argc, argv, "follow_path");
    ROS_INFO("Initialized ros...");

    FollowPath follow_path;
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
