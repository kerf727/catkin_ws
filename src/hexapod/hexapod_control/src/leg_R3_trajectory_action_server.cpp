#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/SetPoseAction.h"
#include "hexapod_control/GaitAction.h"
#include "hexapod_control/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "math.h"

std::string leg_name = "R3";

class SetTrajectoryAction
{
public:
    SetTrajectoryAction(std::string name):
        server(node, name, boost::bind(&SetTrajectoryAction::executeCB, this, _1), false),
        client("leg_" + leg_name + "_pose_action", true),
        actionName(name)
    {
        this->node = node;

        ROS_INFO("Waiting for Pose State Server...");
        this->client.waitForServer(ros::Duration(30));

		ROS_INFO("Subscribing to Gait Controller and Teleop...");
		this->velocitySubscriber = node.subscribe("/hexapod/gait/velocity", 10, &SetTrajectoryAction::velocityCB, this);
		this->strideTimeSubscriber = node.subscribe("/hexapod/gait/stride_time", 10, &SetTrajectoryAction::strideTimeCB, this);
		this->strideHeightSubscriber = node.subscribe("/hexapod/gait/stride_height", 10, &SetTrajectoryAction::strideHeightCB, this);
		this->strideLengthSubscriber = node.subscribe("/hexapod/gait/stride_length", 10, &SetTrajectoryAction::strideLengthCB, this);
        this->stopCommandSubscriber = node.subscribe("/hexapod/gait/stop", 10, &SetTrajectoryAction::stopCommandCB, this);
		this->hexHeadingSubscriber = node.subscribe("/hexapod/gait/hex_heading", 10, &SetTrajectoryAction::hexHeadingCB, this);
		
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

		this->gait_mode = goal->gait_mode;
		this->duty_factor = goal->duty_factor;
		this->initial_phase = goal->initial_phase;
		double eps = 0.05;
		
		int state = 0;
        bool preempted = false;
        stop = false;
        initialized = false;

		double t, elapsed;
		steps = 0;
		hexapod_control::Pose target_pose;

		current_phase = initial_phase;

        node.getParam("/hexapod/geometry/base/radius", base_radius);
        node.getParam("/hexapod/geometry/base/height", base_height);
        node.getParam("/hexapod/geometry/foot/radius", foot_radius);
        node.getParam("/hexapod/geometry/leg_" + leg_name + "/hip_angle", hip_angle);
        hip_angle = hip_angle*M_PI/180; // convert to radians

        // Set center of foot workspace for this leg
        foot_center.x = foot_radius*cos(hip_angle);
        foot_center.y = foot_radius*sin(hip_angle);
        foot_center.z = -base_height;

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
			t = elapsed + initial_phase*stride_time; // account for initial_phase

            // Calculate the current phase
			current_phase = fmod(t/stride_time, 1.0);
            if (!initialized && current_phase >= duty_factor)
            {
                initialized = true;
            }

            // Calculate the target position
            geometry_msgs::Point pos;
            if (!initialized && gait_mode == "Moving/Position")
            {
                pos = InitialWalk(elapsed);
            }
            else if (initialized && gait_mode == "Moving/Position")
            {
                pos = Walk(current_phase);
            }
            else if (!initialized && gait_mode == "Moving/Orientation")
            {
                pos = InitialRotate(elapsed);
            }
            else if (initialized && gait_mode == "Moving/Orientation")
            {
                pos = Rotate(current_phase);
            }
            else if (gait_mode == "Stationary/Position")
            {
                // TODO: Add Stationary/... options to combine with AIK code
            }
            else if (gait_mode == "Stationary/Orientation")
            {
                // TODO: Add Stationary/... options to combine with AIK code
            }
            else
            {
                pos = foot_center;
            }

            ROS_INFO("stage: %s, phase: %f, vel: %f", stage.c_str(), current_phase, velocity);
            ROS_INFO("pos: (%f, %f, %f)", pos.x, pos.y, pos.z);

            if ((stop || preempted) && stage.compare("Support Phase") == 0)
            {
                break;
            }

			// Build message
			target_pose.x = pos.x;
			target_pose.y = pos.y;
			target_pose.z = pos.z;
			target_pose.rotx = std::vector<double>{1.0, 0.0, 0.0};
			target_pose.roty = std::vector<double>{0.0, 1.0, 0.0};
			target_pose.rotz = std::vector<double>{0.0, 0.0, 1.0};

			// Send goal to pose action client
			hexapod_control::SetPoseGoal poseAction;
			poseAction.goal = target_pose;
			poseAction.eps = eps;
			this->client.sendGoal(poseAction,
				boost::bind(&SetTrajectoryAction::publishResult, this, _1, _2),
				boost::bind(&SetTrajectoryAction::activeCB, this),
				boost::bind(&SetTrajectoryAction::publishFeedback, this, _1));

            // Publish feedback
			this->actionFeedback.current_phase = current_phase;
            this->actionFeedback.stage = stage;
			this->actionFeedback.current_pose = current_pose;
			this->actionFeedback.target_pose = target_pose;
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

	void velocityCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->velocity = msg->data;
	}

    void strideTimeCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->stride_time = msg->data;
	}

    void strideHeightCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->stride_height = msg->data;
	}

    void strideLengthCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->stride_length = msg->data;
	}

    void stopCommandCB(const std_msgs::BoolConstPtr& msg)
	{
		this->stop = msg->data;
	}

    void hexHeadingCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->hex_heading = msg->data*M_PI/180;
	}

    geometry_msgs::Point InitialWalk(double elapsed)
    {
        // Position w.r.t. body
        geometry_msgs::Point pos;
        double offset = velocity*elapsed;

        // Just move forward until reach initial phase
        pos.x = foot_center.x - offset*cos(hex_heading);
        pos.y = foot_center.y - offset*sin(hex_heading);
        pos.z = foot_center.z;
        stage = "Initialization";

        return pos;
    }

    geometry_msgs::Point Walk(double phase)
    {
        // Position w.r.t. body
        geometry_msgs::Point pos;
        double offset, support_phase, transfer_phase;

        // Support phase
        if (phase < duty_factor)
        {
            support_phase = phase/duty_factor;
            offset = stride_length*(support_phase - 0.5);
            pos.x = foot_center.x - offset*cos(hex_heading);
            pos.y = foot_center.y - offset*sin(hex_heading);
            pos.z = foot_center.z;
            stage = "Support Phase";
        }
        // Transfer phase
        else
        {
            transfer_phase = (phase - duty_factor)/(1.0 - duty_factor);
            offset = stride_length*(transfer_phase - 0.5);
            pos.x = foot_center.x + offset*cos(hex_heading);
            pos.y = foot_center.y + offset*sin(hex_heading);
            pos.z = foot_center.z + stride_height*sin(M_PI*transfer_phase);
            stage = "Transfer Phase";
        }

        return pos;
    }

    geometry_msgs::Point InitialRotate(double elapsed)
    {
        // Position w.r.t. body
        geometry_msgs::Point pos;
        double angle_offset = hip_angle + velocity*elapsed;

        // Just move forward until reach initial phase
        pos.x = foot_radius*cos(angle_offset);
        pos.y = foot_radius*sin(angle_offset);
        pos.z = foot_center.z;
        stage = "Initialization";

        return pos;
    }

    geometry_msgs::Point Rotate(double phase)
    {
        // Position w.r.t. body
        geometry_msgs::Point pos;
        double theta, support_phase, transfer_phase, foot_angle;

        theta = acos(1 - 0.125*pow(stride_length/foot_radius, 2));

        // Support phase
        if (phase < duty_factor)
        {
            support_phase = phase/duty_factor;
            if (velocity > 0.0)
            {
                foot_angle = hip_angle + theta*(2*support_phase - 1);
            }
            else
            {
                foot_angle = hip_angle - theta*(2*support_phase - 1);
            }
            pos.x = foot_radius*cos(foot_angle);
            pos.y = foot_radius*sin(foot_angle);
            pos.z = foot_center.z;
            stage = "Support Phase";
        }
        // Transfer phase
        else
        {
            transfer_phase = (phase - duty_factor)/(1.0 - duty_factor);
            if (velocity > 0.0)
            {
                foot_angle = hip_angle - theta*(2*transfer_phase - 1);
            }
            else
            {
                foot_angle = hip_angle + theta*(2*transfer_phase - 1);
            }
            pos.x = foot_radius*cos(foot_angle);
            pos.y = foot_radius*sin(foot_angle);
            pos.z = foot_center.z + stride_height*sin(M_PI*transfer_phase);
            stage = "Transfer Phase";
        }

        return pos;
    }

private:
	std::string actionName;
	ros::NodeHandle node;
	actionlib::SimpleActionServer<hexapod_control::GaitAction> server;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> client;
	hexapod_control::GaitFeedback actionFeedback;
	hexapod_control::GaitResult actionResult;
	ros::Subscriber velocitySubscriber;
	ros::Subscriber strideTimeSubscriber;
    ros::Subscriber strideHeightSubscriber;
    ros::Subscriber strideLengthSubscriber;
    ros::Subscriber hexHeadingSubscriber;
    ros::Subscriber stopCommandSubscriber;
	hexapod_control::Pose current_pose;
    std::string gait_mode;
	double initial_phase, current_phase, stride_time, stride_height, stride_length;
	double duty_factor, velocity, hex_heading;
    geometry_msgs::Point foot_center;
    double base_radius, base_height, foot_radius, hip_angle;
    int steps = 0;
    bool stop = false;
    bool initialized = false;
    std::string stage;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Trajectory Action Server...");
    ros::init(argc, argv, "leg_" + leg_name + "_trajectory_action");
    ROS_INFO("Initialized ros...");

    SetTrajectoryAction actionServer("leg_" + leg_name + "_trajectory_action");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
