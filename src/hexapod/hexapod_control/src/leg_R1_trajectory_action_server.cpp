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

std::string leg_name = "R1";

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

		ROS_INFO("Subscribing to Gait Controller...");
		this->dutyFactorSubscriber = node.subscribe("/hexapod/gait/duty_factor", 10, &SetTrajectoryAction::dutyFactorCB, this);
		this->bodyVelocitySubscriber = node.subscribe("/hexapod/gait/body_velocity", 10, &SetTrajectoryAction::bodyVelocityCB, this);
		this->velocityAngleSubscriber = node.subscribe("/hexapod/gait/velocity_angle", 10, &SetTrajectoryAction::velocityAngleCB, this);

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

		this->gait_mode = goal->gait_mode;
		double eps = 0.05;
		
		int state = 0;
        bool preempted = false;
        stop = false;
        initialized = false;

		double t, elapsed;
		steps = 0;
		hexapod_control::Pose target_pose;

        node.getParam("/hexapod/gait/walking/stride_time", stride_time);
        node.getParam("/hexapod/gait/walking/stride_height", stride_height);
        node.getParam("/hexapod/gait/walking/relative_phase/leg_" + leg_name, initial_phase);

		current_phase = initial_phase;

        node.getParam("/hexapod/geometry/base/radius", base_radius);
        node.getParam("/hexapod/geometry/base/height", base_height);
        node.getParam("/hexapod/geometry/foot/radius", foot_radius);
        node.getParam("/hexapod/geometry/leg_" + leg_name + "/hip_angle", hip_angle);
        hip_angle = hip_angle*M_PI/180; // convert to radians

        // Set center of workspace for this leg
        center.x = foot_radius*cos(hip_angle);
        center.y = foot_radius*sin(hip_angle);
        center.z = -base_height;

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
                pos = InitialWalk(current_phase);
            }
            else if (initialized && gait_mode == "Moving/Position")
            {
                pos = Walk(elapsed);
            }
            else if (!initialized && gait_mode == "Moving/Orientation")
            {
                pos = InitialRotate(current_phase);
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

	void dutyFactorCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->duty_factor = msg->data;
	}

	void bodyVelocityCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->body_velocity = msg->data;
	}

    void velocityAngleCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->velocity_angle = msg->data;
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
	ros::Subscriber velocityAngleSubscriber;
    ros::Subscriber stopCommandSubscriber;
	hexapod_control::Pose current_pose;
    std::string gait_mode;
	double initial_phase, current_phase, stride_time, stride_height;
	double duty_factor, body_velocity, velocity_angle;
    hexapod_control::Pose center;
    double base_radius, base_height, foot_radius, hip_angle;
    int steps = 0;
    bool stop = false;
    bool initialized = false;
    std::string stage;

    geometry_msgs::Point InitialWalk(double elapsed)
    {
        // Position w.r.t. body
        geometry_msgs::Point pos;
        double offset = body_velocity*elapsed;

        // Just move forward until reach initial phase
        pos.x = center.x - offset*cos(velocity_angle);
        pos.y = center.y - offset*sin(velocity_angle);
        pos.z = center.z;
        stage = "Initialization";

        return pos;
    }

    geometry_msgs::Point Walk(double phase)
    {
        // Position w.r.t. body
        geometry_msgs::Point pos;
        double stride_length = stride_time*body_velocity;

        // Support phase
        if (phase < duty_factor)
        {
            double support_phase = phase/duty_factor;
            double offset = stride_length*(support_phase - 1/2);
            pos.x = center.x - offset*cos(velocity_angle);
            pos.y = center.y - offset*sin(velocity_angle);
            pos.z = center.z;
            stage = "Support Phase";
        }
        // Transfer phase
        else
        {
            double transfer_phase = (phase - duty_factor)/(1.0 - duty_factor);
            double offset = stride_length*(transfer_phase - 1/2);
            pos.x = center.x + offset*cos(velocity_angle);
            pos.y = center.y + offset*sin(velocity_angle);
            pos.z = center.z + stride_height*sin(M_PI*transfer_phase);
            stage = "Transfer Phase";
        }
        
        return pos;
    }

    geometry_msgs::Point InitialRotate(double elapsed)
    {
        // Position w.r.t. body
        geometry_msgs::Point pos;
        double offset = hip_angle + body_velocity*elapsed;

        // Just move forward until reach initial phase
        pos.x = center.x - foot_radius*cos(offset);
        pos.y = center.y - foot_radius*sin(offset);
        pos.z = center.z;
        stage = "Initialization";

        return pos;
    }

    geometry_msgs::Point Rotate(double phase)
    {
        // Position w.r.t. body
        geometry_msgs::Point pos;
        double stride_length = stride_time*body_velocity;
        double theta = acos(1 - 1/8*pow(stride_length/foot_radius, 2));

        //TODO: Confirm polarity of support and transfer phases

        // Support phase
        if (phase < duty_factor)
        {
            double support_phase = phase/duty_factor;
            double foot_angle = hip_angle + theta*(2*support_phase - 1);
            pos.x = center.x - foot_radius*cos(foot_angle);
            pos.y = center.y - foot_radius*sin(foot_angle);
            pos.z = center.z;
            stage = "Support Phase";
        }
        // Transfer phase
        else
        {
            double transfer_phase = (phase - duty_factor)/(1.0 - duty_factor);
            double foot_angle = hip_angle + theta*(2*transfer_phase - 1);
            pos.x = center.x + foot_radius*cos(foot_angle);
            pos.y = center.y + foot_radius*sin(foot_angle);
            pos.z = center.z + stride_height*sin(M_PI*transfer_phase);
            stage = "Transfer Phase";
        }
        
        return pos;
    }
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
