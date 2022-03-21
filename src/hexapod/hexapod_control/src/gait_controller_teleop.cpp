#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/GaitAction.h"
#include "hexapod_control/MoveAction.h"
#include "hexapod_control/Pose.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "gazebo_msgs/GetLinkState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

class GaitController
{
public:
    GaitController()
    {
        this->node = node;

        ROS_INFO("Subscribing to Teleop...");
		this->twistSubscriber = node.subscribe("/hexapod/teleop/twist", 10, &GaitController::twistCB, this);
		this->buttonSubscriber = node.subscribe("/hexapod/teleop/button", 10, &GaitController::buttonCB, this);
		
        ROS_INFO("Publishing to Trajectory Action Servers...");
        this->gaitModePublisher = node.advertise<std_msgs::String>("/hexapod/gait/gait_mode", 1);
        this->commandPublisher = node.advertise<geometry_msgs::Vector3>("/hexapod/gait/command", 1);
        
        // Initialize variables
        B_button = false;
        gait_type = "ripple";
        gait_counter = 0;

        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

        ROS_INFO("Gait controller ready.");
    }

    ~GaitController()
    {
        this->node.shutdown();
    }

    void buttonCB(const std_msgs::BoolConstPtr& msg)
	{
        B_button = msg->data;
    }

    void twistCB(const geometry_msgs::TwistConstPtr& msg)
	{
		double linearX = msg->linear.x;
		double linearY = msg->linear.y;
		double angular = msg->angular.x;

        if (B_button)
        {
            gait_counter += 1;
            if (gait_counter > 2)
            {
                gait_counter = 0;
            }
        }

        switch(gait_counter)
        {
            case(0):
                gait_type = "ripple";
                break;
            case(1):
                gait_type = "tripod";
                break;
            case(2):
                gait_type = "wave";
                break;
        }

        // Get parameters from parameter server
        node.getParam("/hexapod/gait/" + gait_type + "/max_speed", max_speed);
        node.getParam("/hexapod/gait/" + gait_type + "/max_yaw", max_yaw);

        // Dead zone
        double deadzone = 0.0; // no deadzone necessary for SN30pro+
        if (abs(linearX) < deadzone)
        {
            linearX = 0.0;
        }
        if (abs(linearY) < deadzone)
        {
            linearY = 0.0;
        }
        if (abs(angular) < deadzone)
        {
            angular = 0.0;
        }

        // Map joystick data to speed, yaw, yaw_angle
        // Strafe (Lx and Ly only)
        if ((linearX != 0.0 || linearY != 0.0) && angular == 0.0)
        {  
            gait_mode = "Strafe";
            // TODO: fix max of map input. can be larger than 1.0 when both contribute
            speed = mapRange(abs(linearX) + abs(linearY), 0.0, 1.0, 0.0, max_speed);
            yaw_angle = atan2(linearY, linearX);
            yaw = eps; //(yaw_angle <= 0.25*M_PI && yaw_angle >= -0.75*M_PI) ? eps : -eps;
            // atan2 range is -pi to +pi; yaw is positive if on bottom half of y = x line
            // TODO: check if above is accurate
        }
        // Rotate (Rx only)
        else if (linearY == 0.0 && angular != 0.0)
        {
            gait_mode = "Rotate";
            speed = 0.0;
            yaw = mapRange(angular, -1.0, 1.0, -max_yaw, max_yaw);
            yaw_angle = 0.0;
        }
        // Steer (Ly and Rx)
        else if (linearY != 0.0 && angular != 0.0)
        {
            // TODO: make this a separate mode controlled by a button? imagine pressing Ly and then suddenly pressing Rx
            gait_mode = "Steer";
            speed = mapRange(abs(linearY), 0.0, 1.0, 0.0, max_speed);
            yaw = mapRange(angular, -1.0, 1.0, -max_yaw, max_yaw);
            yaw_angle = mapRange(abs(angular), 0.0, 1.0, 0.0, M_PI/3.0);
            if (angular > 0.0)
            {
                yaw_angle = M_PI - yaw_angle;
            }
        }
        // Default
        else
        {
            gait_mode = "Default";
            speed = 0.0;
            yaw = eps;
            yaw_angle = 0.0;
        }

        // Cap maximum values
        if (speed > max_speed)
        {
            speed = max_speed;
        }
        if (yaw > max_yaw)
        {
            yaw = max_yaw;
        }

        ROS_INFO("------------------------------------------------------------------------------------------------------");
        ROS_INFO("mode: %s, speed: %f, yaw: %f, yaw_angle: %f\n",
            gait_mode.c_str(), speed, yaw, yaw_angle);

        // Publish gait parameters
        std_msgs::String gaitModeMsg;
        gaitModeMsg.data = gait_mode;
        this->gaitModePublisher.publish(gaitModeMsg);

        geometry_msgs::Vector3 commandMsg;
        commandMsg.x = speed;
        commandMsg.y = yaw;
        commandMsg.z = yaw_angle;
        this->commandPublisher.publish(commandMsg);
    }

    double mapRange(const double& inValue,
                    const double& minInRange, const double& maxInRange,
                    const double& minOutRange, const double& maxOutRange)
    {
        double x = (inValue - minInRange) / (maxInRange - minInRange);
        double result = minOutRange + (maxOutRange - minOutRange) * x;

        return result;
    }

    geometry_msgs::Point GetPosition()
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = "robot::dummy_link"; // hexapod/base_link
        linkStateMsg.request.reference_frame = "ground::link"; // world
        this->linkStateClient.call(linkStateMsg);
        return linkStateMsg.response.link_state.pose.position;
    }

    double GetOrientation()
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = "robot::dummy_link";
        linkStateMsg.request.reference_frame = "ground::link";
        this->linkStateClient.call(linkStateMsg);
        return linkStateMsg.response.link_state.pose.orientation.z;
    }
    
    double calcXYVectorAngle(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
    {
        double a_dot_b = a.x*b.x + a.y*b.y;
        double a_mag = sqrt(pow(a.x, 2) + pow(a.y, 2));
        double b_mag = sqrt(pow(b.x, 2) + pow(b.y, 2));

        return acos(a_dot_b/(a_mag*b_mag));
    }

private:
    ros::NodeHandle node;
    ros::Subscriber twistSubscriber;
    ros::Subscriber buttonSubscriber;
    ros::Publisher gaitModePublisher;
    ros::Publisher commandPublisher;
    ros::ServiceClient linkStateClient;
    std::string gait_mode, gait_type;
    int gait_counter;
    double lin_acc, ang_acc;
    double duty_factor, stride_height;
    double phase_L1, phase_L2, phase_L3, phase_R1, phase_R2, phase_R3;
    double speed, yaw, yaw_angle;
    bool B_button;
    double max_speed, max_yaw;
    double eps = 1e-6; // 1e-6
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Pose Action Server...");
    ros::init(argc, argv, "gait_controller");
    ROS_INFO("Initialized ros...");

    GaitController gait_controller;
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}