#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/SetPoseAction.h"
#include "hexapod_teleop/TeleopAction.h"
#include "hexapod_control/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/GetLinkState.h"
#include "geometry_msgs/Twist.h"
#include "math.h"

std::string leg_name = "L2";

class SetIKAction
{
public:
    SetIKAction(std::string name):
        server(node, name, boost::bind(&SetIKAction::executeCB, this, _1), false),
        client("leg_" + leg_name + "_pose_action", true),
        actionName(name)
    {
        this->node = node;

        ROS_INFO("Waiting for Pose State Server...");
        this->client.waitForServer(ros::Duration(30));

        ROS_INFO("Subscribing to Gazebo GetLinkState service...");
        this->stopCommandSubscriber = node.subscribe("/hexapod/gait/stop", 10, &SetIKAction::stopCommandCB, this);
		
		ROS_INFO("Starting...");
		server.start();
    }

	~SetIKAction()
	{
		this->node.shutdown();
	}

	void executeCB(const hexapod_teleop::TeleopGoalConstPtr& goal)
	{
		double start = ros::Time::now().toSec();

		double eps = 0.05;
        bool preempted = false;
        stop = false;

        this->movement_mode = goal->movement_mode;
        this->twist.linear.x  = goal->twist.linear.x;
        this->twist.linear.y  = goal->twist.linear.y;
        this->twist.linear.z  = goal->twist.linear.z;
        this->twist.angular.x = goal->twist.angular.x;
        this->twist.angular.y = goal->twist.angular.y;
        this->twist.angular.z = goal->twist.angular.z;

		double elapsed;
		hexapod_control::Pose targetPose;
        geometry_msgs::Point foot_position, hip_position;

		node.getParam("/hexapod/geometry/base/radius", base_radius);
        node.getParam("/hexapod/geometry/base/height", base_height);
        node.getParam("/hexapod/geometry/foot/radius", foot_radius);
        node.getParam("/hexapod/geometry/leg_" + leg_name + "/hip_angle", hip_angle);

        default_hip_x = base_radius*cos(hip_angle*M_PI/180);
        default_hip_y = base_radius*sin(hip_angle*M_PI/180);
        default_hip_z = 0.0;

        default_foot_x = foot_radius*cos(hip_angle*M_PI/180);
        default_foot_y = foot_radius*sin(hip_angle*M_PI/180);
        default_foot_z = -base_height;

		ros::Rate rate(50);
		while (true)
		{
            // Check if preempted or canceled
            if (server.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("IK action preempted, ending...");
                preempted = true;
            }

            elapsed = ros::Time::now().toSec() - start;

            // Calculate Hip and Leg Positions
            hip_position = calcHips(twist);
            foot_position.x = default_foot_x - (hip_position.x - default_hip_x);
            foot_position.y = default_foot_y - (hip_position.y - default_hip_y);
            foot_position.z = default_foot_z - (hip_position.z - default_hip_z);

            if (stop || preempted)
            {
                break;
            }

			// Build message
			targetPose.x = foot_position.x;
			targetPose.y = foot_position.y;
			targetPose.z = foot_position.z;
            targetPose.rotx = std::vector<double>{1.0, 0.0, 0.0};
			targetPose.roty = std::vector<double>{0.0, 1.0, 0.0};
			targetPose.rotz = std::vector<double>{0.0, 0.0, 1.0};

			// Send parameters to pose action client
			hexapod_control::SetPoseGoal poseAction;
			poseAction.goal = targetPose;
			poseAction.eps = eps;
			this->client.sendGoal(poseAction,
				boost::bind(&SetIKAction::publishResult, this, _1, _2),
				boost::bind(&SetIKAction::activeCB, this),
				boost::bind(&SetIKAction::publishFeedback, this, _1));
            
            this->actionFeedback.time = elapsed;
            server.publishFeedback(this->actionFeedback);
            
            rate.sleep();
		}

        // Publish result
        this->actionResult.time = elapsed;
        if (preempted)
        {
            server.setPreempted(this->actionResult);
        }
        else
        {
            server.setSucceeded(this->actionResult);
        }
	}

	void publishFeedback(const hexapod_control::SetPoseFeedback::ConstPtr& poseFeedback)
	{
		currentPose = poseFeedback->currentPose;
	}

	void publishResult(const actionlib::SimpleClientGoalState& state,
		const hexapod_control::SetPoseResult::ConstPtr& poseResult)
	{

	}

	void activeCB()
	{

	}

    void stopCommandCB(const std_msgs::BoolConstPtr& msg)
	{
		this->stop = msg->data;
	}

private:
	std::string actionName;
	ros::NodeHandle node;
	actionlib::SimpleActionServer<hexapod_teleop::TeleopAction> server;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> client;
    hexapod_teleop::TeleopFeedback actionFeedback;
    hexapod_teleop::TeleopResult actionResult;
    ros::Subscriber stopCommandSubscriber;
	hexapod_control::Pose currentPose;
    std::string movement_mode;
    geometry_msgs::Twist twist;
    double base_radius, base_height, foot_radius, hip_angle;
    double default_foot_x, default_foot_y, default_foot_z;
    double default_hip_x, default_hip_y, default_hip_z;
    bool stop = false;
    
    geometry_msgs::Point calcHips(geometry_msgs::Twist twist)
    {
        geometry_msgs::Point result;

        std::vector<std::vector<double>> R = calcRot(twist.angular.x, twist.angular.y, twist.angular.z);
        std::vector<double> s = {base_radius*cos(hip_angle*M_PI/180), base_radius*sin(hip_angle*M_PI/180), 0.0};
        std::vector<double> Rs = multRbyS(R, s);

        result.x = twist.linear.x + Rs[0];
        result.y = twist.linear.y + Rs[1];
        result.z = twist.linear.z + Rs[2];

        return result;
    }

    std::vector<std::vector<double>> calcRot(double a, double b, double c)
    {
        std::vector<std::vector<double>> result;

        double ca = cos(a); double sa = sin(a);
        double cb = cos(b); double sb = sin(b);
        double cc = cos(c); double sc = sin(c);
        result.push_back(std::vector<double>{cb*cc, -cb*sc, sb});
        result.push_back(std::vector<double>{sa*sb*cc + ca*sc, -sa*sb*sc + ca*cc, -sa*cb});
        result.push_back(std::vector<double>{-ca*sb*cc + sa*sc, ca*sb*sc + sa*cc, ca*cb});

        return result;
    }

    std::vector<double> multRbyS(std::vector<std::vector<double>> R, std::vector<double> s)
    {
        std::vector<double> result;
        result.push_back(R[0][0]*s[0] + R[0][1]*s[1] + R[0][2]*s[2]);
        result.push_back(R[1][0]*s[0] + R[1][1]*s[1] + R[1][2]*s[2]);
        result.push_back(R[2][0]*s[0] + R[2][1]*s[1] + R[2][2]*s[2]);

        return result;
    }
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting IK Action Server...");
    ros::init(argc, argv, "leg_" + leg_name + "_IK_action");
    ROS_INFO("Initialized ros...");

    SetIKAction actionServer("leg_" + leg_name + "_IK_action");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
