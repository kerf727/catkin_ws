#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/SetPoseAction.h"
#include "hexapod_control/GaitAction.h"
#include "hexapod_control/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/GetLinkState.h"
#include "geometry_msgs/Twist.h"
#include "math.h"

class SetTrajectoryAction
{
public:
    SetTrajectoryAction(std::string name):
        server(node, name, boost::bind(&SetTrajectoryAction::executeCB, this, _1), false),
        client("leg_L1_pose_action", true),
        actionName(name)
    {
        this->node = node;

        ROS_INFO("Waiting for Pose State Server...");
        this->client.waitForServer(ros::Duration(30));

        ROS_INFO("Subscribing to Teleop...");
        this->teleopSubscriber = node.subscribe("/hexapod/teleop", 10, &SetTrajectoryAction::teleopCB, this);

        ROS_INFO("Subscribing to Gazebo GetLinkState service...");
        this->stopCommandSubscriber = node.subscribe("/hexapod/gait/stop", 10, &SetTrajectoryAction::stopCommandCB, this);
		
		ROS_INFO("Starting...");
		server.start();
    }

	~SetTrajectoryAction()
	{
		this->node.shutdown();
	}

	void executeCB(const hexapod_control::GaitGoalConstPtr& goal)
	{
		double start = ros::Time::now().toSec();

        int mode;
        mode = 0;

		double eps = 0.05;
        bool preempted = false;
        stop = false;
        initialized = false;

		double elapsed;
		hexapod_control::Pose targetPose;

		ros::Rate rate(50);
		while (true)
		{
            // Check if preempted or canceled
            if (server.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Trajectory action preempted, ending trajectory...");
                preempted = true;
            }

            elapsed = ros::Time::now().toSec() - start;

            // Calculate the target position
            geometry_msgs::Point position;

            if (mode == 0) // Stationary
            {
                // Keyboard inputs affect base position/orientation, feet maintain position
                // Calculate leg hip position

                // Position
                position.x = xOffset - twist.linear.x;
                position.y = yOffset - twist.linear.y;
                position.z = zOffset - twist.linear.z;

                // Orientation
            }
            else if (mode == 1) // Moving (walking and rotating)
            {
                // Keyboard inputs affect direction of walking
                // just L or R -> rotate in place
                // L or R and U or D -> move while rotating
                // Maintain position/orientation set in mode 0 while moving
                // position = ;
            }

            if (stop || preempted)
            {
                break;
            }

			// Build message
			targetPose.x = position.x;
			targetPose.y = position.y;
			targetPose.z = position.z;
			targetPose.rotx = std::vector<double>{1.0, 0.0, 0.0};
			targetPose.roty = std::vector<double>{0.0, 1.0, 0.0};
			targetPose.rotz = std::vector<double>{0.0, 0.0, 1.0};

			// Send parameters to pose action client
			hexapod_control::SetPoseGoal poseAction;
			poseAction.goal = targetPose;
			poseAction.eps = eps;
			this->client.sendGoal(poseAction,
				boost::bind(&SetTrajectoryAction::publishResult, this, _1, _2),
				boost::bind(&SetTrajectoryAction::activeCB, this),
				boost::bind(&SetTrajectoryAction::publishFeedback, this, _1));

			rate.sleep();
		}
	}

	void publishFeedback(const hexapod_control::SetPoseFeedback::ConstPtr& poseFeedback)
	{
		
	}

	void publishResult(const actionlib::SimpleClientGoalState& state,
		const hexapod_control::SetPoseResult::ConstPtr& poseResult)
	{

	}

	void activeCB()
	{

	}

    void teleopCB(const geometry_msgs::TwistConstPtr& data)
	{
		this->twist.linear.x  = data->linear.x;
        this->twist.linear.y  = data->linear.y;
        this->twist.linear.z  = data->linear.z;
        this->twist.angular.x = data->angular.x;
        this->twist.angular.y = data->angular.y;
        this->twist.angular.z = data->angular.z;
	}

    void stopCommandCB(const std_msgs::BoolConstPtr& msg)
	{
		this->stop = msg->data;
	}

private:
	std::string actionName;
	ros::NodeHandle node;
	actionlib::SimpleActionServer<hexapod_control::GaitAction> server;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> client;
    ros::Subscriber teleopSubscriber;
    ros::Subscriber stopCommandSubscriber;
    geometry_msgs::Twist twist;
    double xOffset = -0.08232;
    double yOffset = 0.1426;
    double zOffset = -0.03928;
    bool stop = false;
    bool initialized = false;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Trajectory Action Server...");
    ros::init(argc, argv, "leg_L1_trajectory_action");
    ROS_INFO("Initialized ros...");

    SetTrajectoryAction actionServer("leg_L1_trajectory_action");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
