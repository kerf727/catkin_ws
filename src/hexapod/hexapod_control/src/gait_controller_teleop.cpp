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
        this->strideTimePublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/stride_time", 1);
        this->strideHeightPublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/stride_height", 1);
        this->strideLengthPublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/stride_length", 1);
        this->dutyFactorPublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/duty_factor", 1);
        this->phaseL1Publisher = node.advertise<std_msgs::Float64>("/hexapod/gait/phase_L1", 1);
        this->phaseL2Publisher = node.advertise<std_msgs::Float64>("/hexapod/gait/phase_L2", 1);
        this->phaseL3Publisher = node.advertise<std_msgs::Float64>("/hexapod/gait/phase_L3", 1);
        this->phaseR1Publisher = node.advertise<std_msgs::Float64>("/hexapod/gait/phase_R1", 1);
        this->phaseR2Publisher = node.advertise<std_msgs::Float64>("/hexapod/gait/phase_R2", 1);
        this->phaseR3Publisher = node.advertise<std_msgs::Float64>("/hexapod/gait/phase_R3", 1);
        this->commandPublisher = node.advertise<geometry_msgs::Vector3>("/hexapod/gait/command", 1);
        
        // Initialize subscribed variables
        // TODO: Remove this section? Does this lead to weird startup behavior?
        // speed = 0.0;
        // yaw = 0.0;
        // yaw_angle = 0.0;

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
        // node.getParam("/hexapod/gait/" + gait_type + "/lin_acc", lin_acc);
        // node.getParam("/hexapod/gait/" + gait_type + "/ang_acc", ang_acc);
        node.getParam("/hexapod/gait/" + gait_type + "/stride_time", stride_time);
        node.getParam("/hexapod/gait/" + gait_type + "/stride_height", stride_height);
        node.getParam("/hexapod/gait/" + gait_type + "/stride_length", stride_length);
        node.getParam("/hexapod/gait/" + gait_type + "/duty_factor", duty_factor);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/L1", phase_L1);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/L2", phase_L2);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/L3", phase_L3);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/R1", phase_R1);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/R2", phase_R2);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/R3", phase_R3);

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
            // TODO: Fix max of map input - can be larger than 1.0 when both contribute
            speed = mapRange(abs(linearX) + abs(linearY), 0.0, 1.0, 0.0, max_speed);
            yaw = eps;
            yaw_angle = atan2(linearY, linearX);
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

        std_msgs::Float64 strideTimeMsg;
        strideTimeMsg.data = stride_time;
        this->strideTimePublisher.publish(strideTimeMsg);

        std_msgs::Float64 strideHeightMsg;
        strideHeightMsg.data = stride_height;
        this->strideHeightPublisher.publish(strideHeightMsg);

        std_msgs::Float64 strideLengthMsg;
        strideLengthMsg.data = stride_length;
        this->strideLengthPublisher.publish(strideLengthMsg);

        std_msgs::Float64 dutyFactorMsg;
        dutyFactorMsg.data = duty_factor;
        this->dutyFactorPublisher.publish(dutyFactorMsg);

        std_msgs::Float64 phaseL1Msg;
        phaseL1Msg.data = phase_L1;
        this->phaseL1Publisher.publish(phaseL1Msg);

        std_msgs::Float64 phaseL2Msg;
        phaseL2Msg.data = phase_L2;
        this->phaseL2Publisher.publish(phaseL2Msg);

        std_msgs::Float64 phaseL3Msg;
        phaseL3Msg.data = phase_L3;
        this->phaseL3Publisher.publish(phaseL3Msg);

        std_msgs::Float64 phaseR1Msg;
        phaseR1Msg.data = phase_R1;
        this->phaseR1Publisher.publish(phaseR1Msg);

        std_msgs::Float64 phaseR2Msg;
        phaseR2Msg.data = phase_R2;
        this->phaseR2Publisher.publish(phaseR2Msg);

        std_msgs::Float64 phaseR3Msg;
        phaseR3Msg.data = phase_R3;
        this->phaseR3Publisher.publish(phaseR3Msg);

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
    ros::Publisher strideTimePublisher;
    ros::Publisher strideHeightPublisher;
    ros::Publisher strideLengthPublisher;
    ros::Publisher dutyFactorPublisher;
    ros::Publisher phaseL1Publisher;
    ros::Publisher phaseL2Publisher;
    ros::Publisher phaseL3Publisher;
    ros::Publisher phaseR1Publisher;
    ros::Publisher phaseR2Publisher;
    ros::Publisher phaseR3Publisher;
    ros::Publisher commandPublisher;
    ros::ServiceClient linkStateClient;
    std::string gait_mode, gait_type;
    int gait_counter;
    double lin_acc, ang_acc;
    double duty_factor, stride_time, stride_height, stride_length;
    double phase_L1, phase_L2, phase_L3, phase_R1, phase_R2, phase_R3;
    double speed, yaw, yaw_angle;
    bool B_button;
    double max_speed, max_yaw;
    double eps = 1e-6;
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