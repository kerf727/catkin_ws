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

std::string leg_name = "L1";

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
        Vector3 foot, hip, default_foot, default_hip;

		node.getParam("/hexapod/geometry/base/radius", base_radius);
        node.getParam("/hexapod/geometry/base/height", base_height);
        node.getParam("/hexapod/geometry/foot/radius", foot_radius);
        node.getParam("/hexapod/geometry/leg_" + leg_name + "/hip_angle", hip_angle);
        hip_angle = hip_angle * M_PI / 180; // convert to radians

        default_hip = {base_radius*cos(hip_angle),
                       base_radius*sin(hip_angle),
                       0.0};

        default_foot = {foot_radius*cos(hip_angle),
                        foot_radius*sin(hip_angle),
                        -base_height};

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
            hip = calcHips(twist);
            foot = default_foot - (hip - default_hip);

            if (stop || preempted)
            {
                break;
            }

			// Build message
			targetPose.x = foot.x;
			targetPose.y = foot.y;
			targetPose.z = foot.z;
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

    struct Vector3
        {
            double x, y, z;

            Vector3() :
                x(0.0), y(0.0), z(0.0) {}

            Vector3(double x, double y, double z) :
                x(x), y(y), z(z) {}

            Vector3 operator+(const Vector3& other)
            {
                return Vector3(x + other.x, y + other.y, z + other.z);
            }

            Vector3 operator-(const Vector3& other)
            {
                return Vector3(x - other.x, y - other.y, z - other.z);
            }

            double norm()
            {
                return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
            }
        };

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
    bool stop = false;
    
    Vector3 calcHips(geometry_msgs::Twist twist)
    {
        Vector3 result;

        std::vector<std::vector<double>> R = calcRot(twist.angular.x, twist.angular.y, twist.angular.z);
        Vector3 s = {base_radius*cos(hip_angle),
                     base_radius*sin(hip_angle),
                     0.0};
        Vector3 Rs = multRbyS(R, s);

        result = {twist.linear.x + Rs.x,
                  twist.linear.y + Rs.y,
                  twist.linear.z + Rs.z};

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

    Vector3 multRbyS(std::vector<std::vector<double>> R, Vector3 s)
    {
        Vector3 result;
        result.x = R[0][0]*s.x + R[0][1]*s.y + R[0][2]*s.z;
        result.y = R[1][0]*s.x + R[1][1]*s.y + R[1][2]*s.z;
        result.z = R[2][0]*s.x + R[2][1]*s.y + R[2][2]*s.z;

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
