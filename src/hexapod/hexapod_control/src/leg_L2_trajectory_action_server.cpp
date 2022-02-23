#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/SetPoseAction.h"
#include "hexapod_control/GaitAction.h"
#include "hexapod_control/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/GetLinkState.h"
#include "math.h"

class SetTrajectoryAction
{
public:
    SetTrajectoryAction(std::string name):
        server(node, name, boost::bind(&SetTrajectoryAction::executeCB, this, _1), false),
        client("leg_L2_pose_action", true),
        actionName(name)
    {
        this->node = node;

        ROS_INFO("Waiting for Pose State Server...");
        this->client.waitForServer(ros::Duration(30));

		ROS_INFO("Subscribing to Gait Controller...");
		this->dutyFactorSubscriber = node.subscribe("/hexapod/gait/duty_factor", 10, &SetTrajectoryAction::dutyFactorCB, this);
		this->bodyVelocitySubscriber = node.subscribe("/hexapod/gait/body_velocity", 10, &SetTrajectoryAction::bodyVelocityCB, this);

        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
        this->stopCommandSubscriber = node.subscribe("/hexapod/gait/stop", 10, &SetTrajectoryAction::stopCommandCB, this);
		
		ROS_INFO("Starting...");
		server.start();
    }

	~SetTrajectoryAction()
	{
		this->node.shutdown();
	}

	void executeCB(const hexapod_control::GaitGoalConstPtr &goal)
	{
		double start = ros::Time::now().toSec();

		this->initial_phase = goal->initial_phase;
		this->stride_time = goal->stride_time;
		this->stride_height = goal->stride_height;
		double eps = 0.05;
		
		int state = 0;
        bool preempted = false;
        stop = false;
        initialized = false;

		double t, elapsed;
		steps = 0;
		current_phase = this->initial_phase;
		hexapod_control::Pose target_pose;

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
			t = elapsed + this->initial_phase * stride_time; // account for initial_phase

            // Calculate the current phase
			current_phase = fmod(t / this->stride_time, 1.0);
            if (!initialized && current_phase >= dutyFactor)
            {
                initialized = true;
            }

            // Calculate the target position
            geometry_msgs::Point position;
            if (!initialized)
            {
                position = InitialTrajectory(elapsed);
            }
            else
            {
                position = RotateInPlace(current_phase);
            }

            if ((stop || preempted) && stage.compare("Support Phase") == 0)
            {
                break;
            }

			// Build message
			target_pose.x = position.x;
			target_pose.y = position.y;
			target_pose.z = position.z;
			target_pose.rotx = std::vector<double>{1.0, 0.0, 0.0};
			target_pose.roty = std::vector<double>{0.0, 1.0, 0.0};
			target_pose.rotz = std::vector<double>{0.0, 0.0, 1.0};

			// Send parameters to pose action client
			hexapod_control::SetPoseGoal poseAction;
			poseAction.goal = target_pose;
			poseAction.eps = eps;
			this->client.sendGoal(poseAction,
				boost::bind(&SetTrajectoryAction::publishResult, this, _1, _2),
				boost::bind(&SetTrajectoryAction::activeCB, this),
				boost::bind(&SetTrajectoryAction::publishFeedback, this, _1));

			this->actionFeedback.current_phase = current_phase;
			this->actionFeedback.target_pose = target_pose;
			this->actionFeedback.current_pose = current_pose;
            this->actionFeedback.stage = stage;
			server.publishFeedback(this->actionFeedback);

            if (current_phase == initial_phase)
            {
                steps++;
            }

			rate.sleep();
		}

		// Publish result
        this->actionResult.steps_taken = steps;
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
		current_pose = poseFeedback->current_pose;
	}

	void publishResult(const actionlib::SimpleClientGoalState& state,
		const hexapod_control::SetPoseResult::ConstPtr& poseResult)
	{

	}

	void activeCB()
	{

	}

	void dutyFactorCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->dutyFactor = msg->data;
	}

	void bodyVelocityCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->bodyVelocity = msg->data;
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
    ros::ServiceClient linkStateClient;
	hexapod_control::GaitFeedback actionFeedback;
	hexapod_control::GaitResult actionResult;
	ros::Subscriber dutyFactorSubscriber;
	ros::Subscriber bodyVelocitySubscriber;
    ros::Subscriber stopCommandSubscriber;
	hexapod_control::Pose current_pose;
	double initial_phase;
	double current_phase;
	double stride_time;
	double stride_height;
	double dutyFactor;
	double bodyVelocity;
    double xOffset = -0.16464;
    double yOffset = 0.0;
    double zOffset = -0.03928;
    int steps = 0;
    bool stop = false;
    bool initialized = false;
    std::string stage;

    geometry_msgs::Point InitialTrajectory(double elapsed)
    {
        // Position w.r.t. body
        geometry_msgs::Point position;
        double x, y, z;

        // Just move forward until reach initial phase
        x = xOffset - bodyVelocity * elapsed;
        y = yOffset;
        z = zOffset;
        stage = "Initialization";

        position.x = x;
        position.y = y;
        position.z = z;

        return position;
    }

    geometry_msgs::Point SineTrajectory(double phase)
    {
        // Position w.r.t. body
        geometry_msgs::Point position;
        double x, y, z;

        double strideLength = stride_time * bodyVelocity;

        // Support phase
        if (phase < dutyFactor)
        {
            double supportPhase = phase / dutyFactor;
            x = xOffset;
            y = yOffset + (strideLength / 2 - strideLength * supportPhase);
            z = zOffset;
            stage = "Support Phase";
        }
        // Transfer phase
        else
        {
            double transferPhase = (phase - dutyFactor) / (1.0 - dutyFactor);
            x = xOffset;
            y = yOffset + (strideLength * transferPhase - strideLength / 2);
            z = 0.5 * stride_height * cos(2 * M_PI * transferPhase - M_PI) + zOffset + stride_height/2;
            stage = "Transfer Phase";
        }
        
        position.x = x;
        position.y = y;
        position.z = z;

        return position;
    }

    geometry_msgs::Point RotateInPlace(double phase)
    {
        // Position w.r.t. body
        geometry_msgs::Point position;
        double x, y, z;

        double strideLength = stride_time * bodyVelocity;

        // Support phase
        if (phase < dutyFactor)
        {
            double supportPhase = phase / dutyFactor;
            x = xOffset;
            y = yOffset + (strideLength / 2 - strideLength * supportPhase);
            z = zOffset;
            stage = "Support Phase";
        }
        // Transfer phase
        else
        {
            double transferPhase = (phase - dutyFactor) / (1.0 - dutyFactor);
            x = xOffset;
            y = yOffset + (strideLength * transferPhase - strideLength / 2);
            z = 0.5 * stride_height * cos(2 * M_PI * transferPhase - M_PI) + zOffset + stride_height/2;
            stage = "Transfer Phase";
        }
        
        position.x = x;
        position.y = y;
        position.z = z;

        return position;
    }
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Trajectory Action Server...");
    ros::init(argc, argv, "leg_L2_trajectory_action");
    ROS_INFO("Initialized ros...");

    SetTrajectoryAction actionServer("leg_L2_trajectory_action");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
