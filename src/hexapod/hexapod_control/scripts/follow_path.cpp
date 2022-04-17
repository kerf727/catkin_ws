#include "ros/ros.h"
// #include <iostream>
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "gazebo_msgs/GetLinkState.h"

class FollowPath
{
public:
    FollowPath()
    {
        this->node = node;

        ROS_INFO("Publishing Follow Path Node Data...");
        this->twistPublisher = node.advertise<geometry_msgs::Twist>("hexapod/teleop/twist", 1);
        this->buttonPublisher = node.advertise<std_msgs::Bool>("hexapod/teleop/button", 1);
        
        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

        node.getParam("/hexapod/gait/" + gait_type + "/max_speed", max_speed);
        node.getParam("/hexapod/gait/" + gait_type + "/max_yaw", max_yaw);

        init_pos = getPosition();
        init_heading = getOrientation();
        position_error = 0.0;
        heading_error = 0.0;

        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;
        button_msg.data = false;

        publish_rate_Hz = 10;
        publish_rate_s = 0.1;
        // timer = node.createTimer(ros::Duration(publish_rate), boost::bind(&FollowPath::publishPathPeriodically, this));

        ROS_INFO("Publishing Path Commands.\n");
        publishPath();
    }

    ~FollowPath()
    {
        this->node.shutdown();
    }

private:

    ros::NodeHandle node;
    ros::Publisher twistPublisher;
    ros::Publisher buttonPublisher;
    ros::ServiceClient linkStateClient;
    geometry_msgs::Twist twist_msg;
    std_msgs::Bool button_msg;
    double publish_rate_Hz, publish_rate_s;
    ros::Timer timer;
    boost::mutex publish_mutex;
    std::string gait_type = "tetrapod";
    double max_speed, max_yaw;
    double command_elapsed, command_start;
    geometry_msgs::Point curr_pos, init_pos;
    double curr_heading, init_heading;
    double distance_traveled;
    double position_error, heading_error;
    double position_eps = 1e-2;
    double heading_eps = 1e-2;

    void publishPath()
    {
        // pathCommand(0.0, 0.0, 0.0, 0.0); // Dummy start command
        // pathCommand(0.0, 1.0, 0.0, 3.0);
        // pathCommand(0.0, -1.0, 0.0, 3.0);
        // pathCommand(0.0, 0.0, 1.0, 3.0);

        pathCommand(0.0, 0.0, 0.0, 0.0); // Dummy start command
        pathWaypoint(0.2, 0.3, 0.5, 0.06, 0.15);
    }

    void pathWaypoint(
        const double& target_pos_x, const double& target_pos_y,
        const double& target_heading,
        const double& target_speed, const double& target_yaw
    )
    {
        ros::Rate rate(publish_rate_Hz);

        double error_x, error_y, max_error;
        double speed_ratio, yaw_ratio;
        double target_speed_, target_yaw_;

        target_speed_ = target_speed;
        target_yaw_ = target_yaw;
        if (target_speed_ > max_speed)
        {
            target_speed_ = max_speed;
        }
        if (target_yaw_ > max_yaw)
        {
            target_yaw_ = max_yaw;
        }
        if (target_yaw_ < -max_yaw)
        {
            target_yaw_ = -max_yaw;
        }
        speed_ratio = abs(target_speed/max_speed);
        yaw_ratio = abs(target_yaw/max_yaw);
        
        // Move to correct position
        position_error = sqrt(pow(target_pos_x - curr_pos.x, 2) +
                              pow(target_pos_y - curr_pos.y, 2));
        while (position_error > position_eps)
        {
            // Figure out what path command needs to be sent to get to waypoint
            curr_pos = getPosition();
            error_x = target_pos_x - curr_pos.x;
            error_y = target_pos_y - curr_pos.y;
            position_error = sqrt(pow(error_x, 2) +
                                  pow(error_y, 2));
            max_error = (abs(error_x) > abs(error_y)) ? error_x : error_y;
            twist_msg.linear.x = speed_ratio*error_x/max_error;
            twist_msg.linear.y = speed_ratio*error_y/max_error;
            twist_msg.angular.x = 0.0;
            twistPublisher.publish(twist_msg);

            ROS_INFO("Lx: %.3f, Ly: %.3f, Rx: %.3f",
                twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.x);
            ROS_INFO("curr_pos: (%.3f, %.3f), position_error: %.3f, curr_heading: %.3f, heading_error: %.3f",
                curr_pos.x, curr_pos.y, position_error, curr_heading, heading_error);
            
            rate.sleep();
        }

        // Rotate to correct heading
        heading_error = abs(target_heading - curr_heading);
        while (heading_error > heading_eps)
        {
            // Figure out what path command needs to be sent to get to waypoint
            curr_heading = getOrientation();
            heading_error = abs(target_heading - curr_heading);
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.angular.x = (target_heading > curr_heading) ? yaw_ratio : -yaw_ratio;
            twistPublisher.publish(twist_msg);

            ROS_INFO("Lx: %.3f, Ly: %.3f, Rx: %.3f",
                twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.x);
            ROS_INFO("curr_pos: (%.3f, %.3f), position_error: %.3f, curr_heading: %.3f, heading_error: %.3f",
                curr_pos.x, curr_pos.y, position_error, curr_heading, heading_error);

            rate.sleep();
        }

        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.x = 0.0;
        twistPublisher.publish(twist_msg);

        ROS_INFO("Lx: %.3f, Ly: %.3f, Rx: %.3f",
            twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.x);
        ROS_INFO("curr_pos: (%.3f, %.3f), position_error: %.3f, curr_heading: %.3f, heading_error: %.3f",
            curr_pos.x, curr_pos.y, position_error, curr_heading, heading_error);
    }

    void pathCommand(
        const double& linearX, const double& linearY,
        const double& angular, const double& runtime)
    {
        command_start = ros::Time::now().toSec();
        command_elapsed = 0.0;
        ros::Rate rate(publish_rate_Hz);
        while (command_elapsed <= runtime)
        {
            command_elapsed = ros::Time::now().toSec() - command_start;
            twist_msg.linear.x = linearX;
            twist_msg.linear.y = linearY;
            twist_msg.angular.x = angular;
            twistPublisher.publish(twist_msg);

            ROS_INFO("Lx: %.3f, Ly: %.3f, Rx: %.3f",
                twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.x);

            rate.sleep();
        }
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.x = 0.0;
        twistPublisher.publish(twist_msg);

        ROS_INFO("Lx: %.3f, Ly: %.3f, Rx: %.3f",
            twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.x);
    }

    geometry_msgs::Point getPosition()
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = "robot::dummy_link"; // hexapod/base_link
        linkStateMsg.request.reference_frame = "ground::link"; // world
        this->linkStateClient.call(linkStateMsg);
        return linkStateMsg.response.link_state.pose.position;
    }

    double getOrientation()
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = "robot::dummy_link";
        linkStateMsg.request.reference_frame = "ground::link";
        this->linkStateClient.call(linkStateMsg);
        // negative sign to match yaw convention
        return -linkStateMsg.response.link_state.pose.orientation.z;
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
