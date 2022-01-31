#include "actionlib/server/simple_action_server.h"
#include "hexapod_control/SetJointAction.h"
#include "hexapod_control/SolveFKPose.h"
#include "hexapod_control/SolveIKPose.h"
#include "hexapod_control/Pose.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

class SetJointAction
{
public:
    SetJointAction(std::string name) :
        server(node, name, boost::bind(&SetJointAction::executeCB, this, _1), false),
        actionName(name)
    {
        this->node = node;

        ROS_INFO("Subscribing to Joint States...");
        this->jointStateSubscriber = node.subscribe("/hexapod/joint_states", 10, &SetJointAction::jointStatesCB, this);

        ROS_INFO("Publishing to Joint Controllers");
        this->hip1JointPublisher = node.advertise<std_msgs::Float64>("/hexapod/leg_R2/hip1_joint_position_controller/command", 1);
        this->hip2JointPublisher = node.advertise<std_msgs::Float64>("/hexapod/leg_R2/hip2_joint_position_controller/command", 1);
        this->kneeJointPublisher = node.advertise<std_msgs::Float64>("/hexapod/leg_R2/knee_joint_position_controller/command", 1);

        ROS_INFO("Subscribing to FKPoseSolver service...");
        this->fkClient = node.serviceClient<hexapod_control::SolveFKPose>("/hexapod/leg_R2/fk");

        ROS_INFO("Starting...");
        server.start();
    }

    ~SetJointAction()
    {
        this->node.shutdown();
    }

    void executeCB(const hexapod_control::SetJointGoalConstPtr& goal)
    {
        // Set goal
        this->hip1Target = goal->goal[0];
        this->hip2Target = goal->goal[1];
        this->kneeTarget = goal->goal[2];
        this->eps = goal->eps;

        // Command joints
        std_msgs::Float64 hip1JointCommand, hip2JointCommand, kneeJointCommand;
        hip1JointCommand.data = this->hip1Target;
        hip2JointCommand.data = this->hip2Target;
        kneeJointCommand.data = this->kneeTarget;
        hip1JointPublisher.publish(hip1JointCommand);
        hip2JointPublisher.publish(hip2JointCommand);
        kneeJointPublisher.publish(kneeJointCommand);
        this->actionFeedback.targets = goal->goal;

        // Get current time
        double start = ros::Time::now().toSec();

        // Make sure goal state is not current state
        if (calculateJointError() < this->eps)
        {
            this->actionResult.result = currentState.position;
            this->actionResult.error = calculateJointError();
            this->actionResult.time = 0;
            server.setSucceeded(actionResult);
            return;
        }

        // Wait for joints to start moving
        ros::Rate rate(50);
        while(isRobotIdle())
        {
            rate.sleep();
        }

        // Publish feedback
        while (!isRobotIdle() && server.isActive())
        {
            // Check for preemption
            if (server.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Joint action preempted.");
                server.setPreempted();
                return;
            }

            this->actionFeedback.positions = currentState.position;
            this->actionFeedback.error = calculateJointError();
            this->actionFeedback.time = ros::Time::now().toSec() - start;
            server.publishFeedback(this->actionFeedback);

            rate.sleep();
        }

        // Publish result
        this->actionResult.result = currentState.position;
        this->actionResult.error = calculateJointError();
        this->actionResult.time = ros::Time::now().toSec() - start;
        if (this->actionResult.error < eps)
        {
            this->actionResult.errorCode = 0;
        }
        else
        {
            this->actionResult.errorCode = -2;
        }
        
        server.setSucceeded(this->actionResult);
    }

    hexapod_control::SolveFKPoseResponse getCurrentPose()
    {
        // Send FK request to service
        hexapod_control::SolveFKPose fkMsg;
        fkMsg.request.jointPositions = this->currentState.position;
        fkClient.call(fkMsg);

        return fkMsg.response;
    }

    double calculateJointError()
    {
        double hip1Error = this->hip1Target - currentState.position[0];
        double hip2Error = this->hip2Target - currentState.position[1];
        double kneeError = this->kneeTarget - currentState.position[2];
        return sqrt(pow(hip1Error, 2) + pow(hip2Error, 2) + pow(kneeError, 2));
    }

    bool isRobotIdle()
    {
        std::vector<double> velocity = this->currentState.velocity;
        double magnitude = 0;
        for (int i = 0; i < velocity.size(); ++i)
        {
            magnitude += pow(velocity[i], 2);
        }
        double jointError = calculateJointError();
        return magnitude < 0.05 && jointError < eps;
    }

    void jointStatesCB(const sensor_msgs::JointStateConstPtr& msg)
    {
        this->temp = *msg.get();
        int hip1Index, hip2Index, kneeIndex;
        for (int i = 0; i < temp.name.size(); ++i)
        {
            std::string name_i = temp.name[i];
            if (name_i.find("R2") != std::string::npos)
            {
                if (name_i.find("hip1") != std::string::npos)
                {
                    hip1Index = i;
                }
                else if (name_i.find("hip2") != std::string::npos)
                {
                    hip2Index = i;
                }
                else if (name_i.find("knee") != std::string::npos)
                {
                    kneeIndex = i;
                }
            }
        }

        this->currentState.name     = {temp.name[hip1Index],     temp.name[hip2Index],     temp.name[kneeIndex]};
        this->currentState.position = {temp.position[hip1Index], temp.position[hip2Index], temp.position[kneeIndex]};
        this->currentState.velocity = {temp.velocity[hip1Index], temp.velocity[hip2Index], temp.velocity[kneeIndex]};
        this->currentState.effort   = {temp.effort[hip1Index],   temp.effort[hip2Index],   temp.effort[kneeIndex]};
    }

private:
    std::string actionName;
    ros::NodeHandle node;
    actionlib::SimpleActionServer<hexapod_control::SetJointAction> server;
    hexapod_control::SetJointFeedback actionFeedback;
    hexapod_control::SetJointResult actionResult;
    ros::ServiceClient fkClient;
    ros::Subscriber jointStateSubscriber;
    ros::Publisher hip1JointPublisher;
    ros::Publisher hip2JointPublisher;
    ros::Publisher kneeJointPublisher;
    sensor_msgs::JointState currentState;
    sensor_msgs::JointState temp;
    double hip1Target;
    double hip2Target;
    double kneeTarget;
    double eps;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Joint Action Server...");
    ros::init(argc, argv, "leg_R2_joint_action");

    SetJointAction actionServer("leg_R2_joint_action");
    ros::spin();
    return 0;
}