#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/GaitAction.h"
#include "hexapod_control/MoveAction.h"
#include "hexapod_control/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "gazebo_msgs/GetLinkState.h"

class GaitController
{
public:
    GaitController(std::string name) : server(node, name, boost::bind(&GaitController::executeCB, this, _1), false),
                                       L1_client("leg_L1_trajectory_action", true),
                                       R1_client("leg_R1_trajectory_action", true),
                                       L2_client("leg_L2_trajectory_action", true),
                                       R2_client("leg_R2_trajectory_action", true),
                                       L3_client("leg_L3_trajectory_action", true),
                                       R3_client("leg_R3_trajectory_action", true),
                                       actionName(name)
    {
        this->node = node;

        ROS_INFO("Starting gait parameters publishers...");
        this->dutyRatioPublisher = node.advertise<std_msgs::Float64>(
            "/hexapod/gait/duty_factor", 1);
        this->bodyVelocityPublisher = node.advertise<std_msgs::Float64>(
            "/hexapod/gait/body_velocity", 1);
        this->stopCommandPublisher = node.advertise<std_msgs::Bool>(
            "/hexapod/gait/stop", 1);

        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

        ROS_INFO("Waiting for Leg Trajectory Servers...");
        this->L1_client.waitForServer(ros::Duration(30));
        this->R1_client.waitForServer(ros::Duration(30));
        this->L2_client.waitForServer(ros::Duration(30));
        this->R2_client.waitForServer(ros::Duration(30));
        this->L3_client.waitForServer(ros::Duration(30));
        this->R3_client.waitForServer(ros::Duration(30));

        ROS_INFO("Starting Gait Action Server...");
        server.start();

        ROS_INFO("Gait controller ready.");
    }

    ~GaitController()
    {
        this->node.shutdown();
    }

    void executeCB(const hexapod_control::MoveGoalConstPtr &goal)
    {
        // Extract goal
        int gaitType = goal->gaitType;
        double targetDistance = goal->distance;
        double targetTime = goal->time;
        double initialPhase[4];
        ROS_INFO("Gait Controller goal received.");

        // Calculate gait parameters
        int numSteps;
        double bodyVelocity, bodyAcceleration, strideTime, strideHeight, dutyRatio;
        std::vector<double> relative_phases;
        if (gaitType == 0) // Walking
        {
            // Get parameters from parameter server
            node.getParam("/hexapod/gait/walking/velocity", bodyVelocity);
            node.getParam("/hexapod/gait/walking/acceleration", bodyAcceleration);
            node.getParam("/hexapod/gait/walking/duty_factor", dutyRatio);
            node.getParam("/hexapod/gait/walking/relative_phase", relative_phases);
            node.getParam("/hexapod/gait/walking/stride_time", strideTime);
            node.getParam("/hexapod/gait/walking/stride_height", strideHeight);
        }

        // Get initial position
        auto initialPosition = GetPosition();

        // Get current time
        double start = ros::Time::now().toSec();

        // Publish initial gait parameters
        std_msgs::Float64 msg;
        msg.data = dutyRatio;
        this->dutyRatioPublisher.publish(msg);
        
        std_msgs::Bool stopCommand;
        stopCommand.data = true;

        double velocity = 0.0;
        msg.data = velocity;
        this->bodyVelocityPublisher.publish(msg);

        // Send goals to trajectory servers
        ROS_INFO("Initializing leg trajectories...");
        hexapod_control::GaitGoal gaitAction;
        gaitAction.strideTime = strideTime;
        gaitAction.strideHeight = strideHeight;
        gaitAction.initialPhase = relative_phases[0];
        this->L1_client.sendGoal(gaitAction,
                                 boost::bind(&GaitController::L1Result, this, _1, _2),
                                 boost::bind(&GaitController::L1Active, this),
                                 boost::bind(&GaitController::L1Feedback, this, _1));
        gaitAction.initialPhase = relative_phases[1];
        this->R1_client.sendGoal(gaitAction,
                                 boost::bind(&GaitController::R1Result, this, _1, _2),
                                 boost::bind(&GaitController::R1Active, this),
                                 boost::bind(&GaitController::R1Feedback, this, _1));
        gaitAction.initialPhase = relative_phases[2];
        this->L2_client.sendGoal(gaitAction,
                                 boost::bind(&GaitController::L2Result, this, _1, _2),
                                 boost::bind(&GaitController::L2Active, this),
                                 boost::bind(&GaitController::L2Feedback, this, _1));
        gaitAction.initialPhase = relative_phases[3];
        this->R2_client.sendGoal(gaitAction,
                                 boost::bind(&GaitController::R2Result, this, _1, _2),
                                 boost::bind(&GaitController::R2Active, this),
                                 boost::bind(&GaitController::R2Feedback, this, _1));
        gaitAction.initialPhase = relative_phases[4];
        this->L3_client.sendGoal(gaitAction,
                                 boost::bind(&GaitController::L3Result, this, _1, _2),
                                 boost::bind(&GaitController::L3Active, this),
                                 boost::bind(&GaitController::L3Feedback, this, _1));
        gaitAction.initialPhase = relative_phases[5];
        this->R3_client.sendGoal(gaitAction,
                                 boost::bind(&GaitController::R3Result, this, _1, _2),
                                 boost::bind(&GaitController::R3Active, this),
                                 boost::bind(&GaitController::R3Feedback, this, _1));   

        // Start gait loop
        ROS_INFO("Starting gait...");
        ros::Rate rate(50);
        int stage = 0;
        double distanceTraveled, stride, rampTime, rampDistance;
        bool preempted = false;
        bool aborted = false;
        std::string description = "Ramping up.";

        ROS_INFO("Ramp up stage.");
        while (true)
        {
            // Calculate elapsed time
            double elapsed = ros::Time::now().toSec() - start;

            // Calculate distance traveled
            auto currentPosition = GetPosition();
            distanceTraveled = sqrt(pow(currentPosition.x - initialPosition.x, 2) + 
                                    pow(currentPosition.y - initialPosition.y, 2));

            // Check if preempted
            if (server.isPreemptRequested())
            {
                ROS_INFO("Gait action preempted, ending gait...");
                preempted = true;
                stage = 2;
                description = "Slowing down.";
            }
            else if (!ros::ok())
            {
                ROS_INFO("Gait action aborted, ending gait...");
                aborted = true;
                stage = 2;
                description = "Slowing down.";
            }

            // Control gait
            if (stage == 0) // Ramp-up stage
            {
                 // Ramping up
                if (velocity < bodyVelocity)
                {
                    velocity = velocity + elapsed*bodyAcceleration;
                    msg.data = velocity;
                    this->bodyVelocityPublisher.publish(msg);
                }
                // Hit target velocity
                else
                {
                    msg.data = bodyVelocity;
                    this->bodyVelocityPublisher.publish(msg);
                    rampTime = elapsed;
                    rampDistance = distanceTraveled;
                    stage = 1;
                    description = "Constant velocity.";
                    ROS_INFO("Constant velocity stage.");
                }
            }
            else if (stage == 1) // Constant velocity stage
            {
                msg.data = bodyVelocity;
                this->bodyVelocityPublisher.publish(msg);
                if (distanceTraveled >= targetDistance - rampDistance)
                {
                    stage = 2;
                    description = "Slowing down.";
                    ROS_INFO("Slowing down stage.");
                }
            }
            else if (stage == 2) // Slow-down stage
            {
                // Slowing down
                if (velocity > 0)
                {
                    velocity = velocity - elapsed*bodyAcceleration;
                    if (velocity < 0)
                    {
                        velocity = 0;
                    }
                    msg.data = velocity;
                    this->bodyVelocityPublisher.publish(msg);
                }
                // Hit zero velocity
                else
                {
                    this->stopCommandPublisher.publish(stopCommand);
                    stage = 3;
                    description = "Stopping.";
                    ROS_INFO("Stopping stage.");
                }
            }
            else // Stopping stage
            {
                this->stopCommandPublisher.publish(stopCommand);
                if (!L1IsActive && !R1IsActive && !L2IsActive && !R2IsActive && !L3IsActive && !R3IsActive)
                {
                    ROS_INFO("Stopped.");
                    break;
                }
            }
            
            // Publish feedback
            this->actionFeedback.distance = distanceTraveled;
            this->actionFeedback.time = elapsed;
            this->actionFeedback.description = description;
            server.publishFeedback(this->actionFeedback);

            rate.sleep();
        }

        // Publish result
        this->actionResult.distance = distanceTraveled;
        this->actionResult.time = ros::Time::now().toSec() - start;

        if (preempted)
        {
            server.setPreempted(actionResult);
        }
        else if (aborted)
        {
            server.setAborted(actionResult);
        }
        else
        {
            server.setSucceeded(actionResult);
        }
    }

    void L1Active()
    {
        L1IsActive = true;
    }

    void L1Feedback(const hexapod_control::GaitFeedback::ConstPtr &gaitFeedback)
    {
    }

    void L1Result(const actionlib::SimpleClientGoalState &state,
                  const hexapod_control::GaitResult::ConstPtr &gaitResult)
    {
        L1IsActive = false;
    }

    void R1Active()
    {
        R1IsActive = true;
    }

    void R1Feedback(const hexapod_control::GaitFeedback::ConstPtr &gaitFeedback)
    {
    }

    void R1Result(const actionlib::SimpleClientGoalState &state,
                  const hexapod_control::GaitResult::ConstPtr &gaitResult)
    {
        R1IsActive = false;
    }

    void L2Active()
    {
        L2IsActive = true;
    }

    void L2Feedback(const hexapod_control::GaitFeedback::ConstPtr &gaitFeedback)
    {
    }

    void L2Result(const actionlib::SimpleClientGoalState &state,
                  const hexapod_control::GaitResult::ConstPtr &gaitResult)
    {
        L2IsActive = false;
    }

    void R2Active()
    {
        R2IsActive = true;
    }

    void R2Feedback(const hexapod_control::GaitFeedback::ConstPtr &gaitFeedback)
    {
    }

    void R2Result(const actionlib::SimpleClientGoalState &state,
                  const hexapod_control::GaitResult::ConstPtr &gaitResult)
    {
        R2IsActive = false;
    }

    void L3Active()
    {
        L3IsActive = true;
    }

    void L3Feedback(const hexapod_control::GaitFeedback::ConstPtr &gaitFeedback)
    {
    }

    void L3Result(const actionlib::SimpleClientGoalState &state,
                  const hexapod_control::GaitResult::ConstPtr &gaitResult)
    {
        L3IsActive = false;
    }

    void R3Active()
    {
        R3IsActive = true;
    }

    void R3Feedback(const hexapod_control::GaitFeedback::ConstPtr &gaitFeedback)
    {
    }

    void R3Result(const actionlib::SimpleClientGoalState &state,
                  const hexapod_control::GaitResult::ConstPtr &gaitResult)
    {
        R3IsActive = false;
    }

private:

    geometry_msgs::Point GetPosition()
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = "body";
        linkStateMsg.request.reference_frame = "world";
        this->linkStateClient.call(linkStateMsg);
        return linkStateMsg.response.link_state.pose.position;
    }

    std::string actionName;
    ros::NodeHandle node;
    actionlib::SimpleActionServer<hexapod_control::MoveAction> server;
    actionlib::SimpleActionClient<hexapod_control::GaitAction> L1_client;
    actionlib::SimpleActionClient<hexapod_control::GaitAction> R1_client;
    actionlib::SimpleActionClient<hexapod_control::GaitAction> L2_client;
    actionlib::SimpleActionClient<hexapod_control::GaitAction> R2_client;
    actionlib::SimpleActionClient<hexapod_control::GaitAction> L3_client;
    actionlib::SimpleActionClient<hexapod_control::GaitAction> R3_client;
    bool L1IsActive = false;
    bool R1IsActive = false;
    bool L2IsActive = false;
    bool R2IsActive = false;
    bool L3IsActive = false;
    bool R3IsActive = false;
    hexapod_control::MoveFeedback actionFeedback;
    hexapod_control::MoveResult actionResult;
    ros::Publisher dutyRatioPublisher;
    ros::Publisher bodyVelocityPublisher;
    ros::Publisher stopCommandPublisher;
    ros::ServiceClient linkStateClient;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Pose Action Server...");
    ros::init(argc, argv, "gait_controller_node");
    ROS_INFO("Initialized ros...");

    GaitController actionServer("gait_controller");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}