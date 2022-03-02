#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/GaitAction.h"
#include "hexapod_control/MoveAction.h"
#include "hexapod_teleop/TeleopAction.h"
#include "hexapod_control/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "gazebo_msgs/GetLinkState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

class GaitController
{
public:
    GaitController(std::string name):
        server(node, name, boost::bind(&GaitController::executeCB, this, _1), false),
        L1_client("leg_L1_trajectory_action", true),
        L2_client("leg_L2_trajectory_action", true),
        L3_client("leg_L3_trajectory_action", true),
        R1_client("leg_R1_trajectory_action", true),
        R2_client("leg_R2_trajectory_action", true),
        R3_client("leg_R3_trajectory_action", true),
        actionName(name)
    {
        this->node = node;

        ROS_INFO("Starting gait parameters publishers...");
        this->dutyRatioPublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/duty_factor", 1);
        this->bodyVelocityPublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/lin_vel", 1);
        this->velocityAnglePublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/velocity_angle", 1);
        this->stopCommandPublisher = node.advertise<std_msgs::Bool>("/hexapod/gait/stop", 1);

        //TODO: Consider including above variables in Gait Action instead of publishing

        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

        ROS_INFO("Waiting for Leg Trajectory Servers...");
        this->L1_client.waitForServer(ros::Duration(30));
        this->L2_client.waitForServer(ros::Duration(30));
        this->L3_client.waitForServer(ros::Duration(30));
        this->R1_client.waitForServer(ros::Duration(30));
        this->R2_client.waitForServer(ros::Duration(30));
        this->R3_client.waitForServer(ros::Duration(30));

        ROS_INFO("Starting Gait Action Server...");
        server.start();

        ROS_INFO("Gait controller ready.");
    }

    ~GaitController()
    {
        this->node.shutdown();
    }

    void executeCB(const hexapod_teleop::TeleopGoalConstPtr& goal)
    {
        // Extract goal
        this->gait_mode = goal->gait_mode;
        this->base_twist  = goal->base_twist;
        this->hex_pos  = goal->hex_pos;
        this->hex_heading = goal->hex_heading;
        this->hex_rot = goal->hex_rot;
        ROS_INFO("Gait Controller goal received.");

        base_pos.x = base_twist.linear.x;
        base_pos.y = base_twist.linear.y;
        base_pos.z = base_twist.linear.z;
        base_rot.x = base_twist.angular.x;
        base_rot.y = base_twist.angular.y;
        base_rot.z = base_twist.angular.z;

        // Calculate gait parameters
        int numSteps;
        double lin_vel, lin_acc, ang_vel, ang_acc, stride_time, stride_height, duty_ratio;
        std::vector<double> relative_phases;

        // TODO: Add other gait types than walking
        // Get parameters from parameter server
        node.getParam("/hexapod/gait/walking/linear_velocity", lin_vel);
        node.getParam("/hexapod/gait/walking/linear_acceleration", lin_acc);
        node.getParam("/hexapod/gait/walking/angular_velocity", ang_vel);
        node.getParam("/hexapod/gait/walking/angular_acceleration", ang_acc);
        node.getParam("/hexapod/gait/walking/duty_factor", duty_ratio);
        node.getParam("/hexapod/gait/walking/relative_phase", relative_phases);
        node.getParam("/hexapod/gait/walking/stride_time", stride_time);
        node.getParam("/hexapod/gait/walking/stride_height", stride_height);

        // Get initial position
        geometry_msgs::Point initial_pos = GetPosition();
        double initial_angle = GetOrientation();

        // Get current time
        double start = ros::Time::now().toSec();

        // Publish initial gait parameters
        std_msgs::Float64 msg;
        msg.data = duty_ratio;
        this->dutyRatioPublisher.publish(msg);

        double velocity = 0.0;
        msg.data = velocity;
        this->bodyVelocityPublisher.publish(msg);

        msg.data = hex_heading;
        this->velocityAnglePublisher.publish(msg);

        std_msgs::Bool stopCommand;
        stopCommand.data = true;

        // Send goals to trajectory servers
        ROS_INFO("Initializing leg trajectories...");

        hexapod_control::GaitGoal gaitAction;
        gaitAction.gait_mode = gait_mode;
        gaitAction.stride_time = stride_time;
        gaitAction.stride_height = stride_height;
        ROS_INFO("test: %s", gait_mode.c_str());

        gaitAction.initial_phase = relative_phases[0];
        this->L1_client.sendGoal(gaitAction,
            boost::bind(&GaitController::L1Result, this, _1, _2),
            boost::bind(&GaitController::L1Active, this),
            boost::bind(&GaitController::L1Feedback, this, _1));
        gaitAction.initial_phase = relative_phases[1];
        this->R1_client.sendGoal(gaitAction,
            boost::bind(&GaitController::R1Result, this, _1, _2),
            boost::bind(&GaitController::R1Active, this),
            boost::bind(&GaitController::R1Feedback, this, _1));
        gaitAction.initial_phase = relative_phases[2];
        this->L2_client.sendGoal(gaitAction,
            boost::bind(&GaitController::L2Result, this, _1, _2),
            boost::bind(&GaitController::L2Active, this),
            boost::bind(&GaitController::L2Feedback, this, _1));
        gaitAction.initial_phase = relative_phases[3];
        this->R2_client.sendGoal(gaitAction,
            boost::bind(&GaitController::R2Result, this, _1, _2),
            boost::bind(&GaitController::R2Active, this),
            boost::bind(&GaitController::R2Feedback, this, _1));
        gaitAction.initial_phase = relative_phases[4];
        this->L3_client.sendGoal(gaitAction,
            boost::bind(&GaitController::L3Result, this, _1, _2),
            boost::bind(&GaitController::L3Active, this),
            boost::bind(&GaitController::L3Feedback, this, _1));
        gaitAction.initial_phase = relative_phases[5];
        this->R3_client.sendGoal(gaitAction,
            boost::bind(&GaitController::R3Result, this, _1, _2),
            boost::bind(&GaitController::R3Active, this),
            boost::bind(&GaitController::R3Feedback, this, _1));   

        // Start gait loop
        ROS_INFO("Starting gait...");
        ros::Rate rate(50);
        int stage = 0; 
        double traveled, stride, ramp_time;
        bool preempted = false;
        bool aborted = false;
        std::string description = "Ramping up.";

        ROS_INFO("Ramp up stage.");
        while (true)
        {
            // Calculate elapsed time
            double elapsed = ros::Time::now().toSec() - start;

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
            if (gait_mode == "Moving/Position")
            {
                double distance_traveled, ramp_distance;

                // Calculate distance traveled
                geometry_msgs::Point current_pos = GetPosition();
                distance_traveled = sqrt(pow(current_pos.x - initial_pos.x, 2) + 
                                         pow(current_pos.y - initial_pos.y, 2));
                traveled = distance_traveled;   

                if (stage == 0) // Ramp-up stage
                {
                    // Ramping up
                    if (velocity < lin_vel)
                    {
                        velocity = velocity + elapsed*lin_acc;
                        msg.data = velocity;
                        this->bodyVelocityPublisher.publish(msg);
                    }
                    // Hit target velocity
                    else
                    {
                        msg.data = lin_vel;
                        this->bodyVelocityPublisher.publish(msg);
                        ramp_time = elapsed;
                        ramp_distance = distance_traveled;
                        stage = 1;
                        description = "Constant velocity.";
                        ROS_INFO("Constant velocity stage.");
                    }
                }
                else if (stage == 1) // Constant velocity stage
                {
                    msg.data = lin_vel;
                    this->bodyVelocityPublisher.publish(msg);
                    if (distance_traveled >= hex_pos - ramp_distance)
                    {
                        stage = 2;
                        description = "Slowing down.";
                        ROS_INFO("Slowing down stage.");
                    }
                }
                else if (stage == 2) // Slow-down stage
                {
                    // Slowing down
                    if (velocity > 0.0)
                    {
                        velocity = velocity - elapsed*lin_acc;
                        if (velocity < 0.0)
                        {
                            velocity = 0.0;
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
                else if (stage == 3) // Stopping stage
                {
                    this->stopCommandPublisher.publish(stopCommand);
                    if (!L1IsActive && !R1IsActive && !L2IsActive && !R2IsActive && !L3IsActive && !R3IsActive)
                    {
                        ROS_INFO("Stopped.");
                        break;
                    }
                }
            }
            else if (gait_mode == "Moving/Orientation")
            {
                double angle_traveled, ramp_angle;
                
                // Calculate angle traveled
                double current_angle = GetOrientation();
                angle_traveled = abs(current_angle - initial_angle);
                traveled = angle_traveled;

                if (stage == 0) // Ramp-up stage
                {
                    // Ramping up
                    if (velocity < ang_vel)
                    {
                        velocity = velocity + elapsed*ang_acc;
                        msg.data = velocity;
                        this->bodyVelocityPublisher.publish(msg);
                    }
                    // Hit target velocity
                    else
                    {
                        msg.data = ang_vel;
                        this->bodyVelocityPublisher.publish(msg);
                        ramp_time = elapsed;
                        ramp_angle = angle_traveled;
                        stage = 1;
                        description = "Constant velocity.";
                        ROS_INFO("Constant velocity stage.");
                    }
                }
                else if (stage == 1) // Constant velocity stage
                {
                    msg.data = ang_vel;
                    this->bodyVelocityPublisher.publish(msg);
                    if (angle_traveled >= abs(hex_rot) - ramp_angle)
                    {
                        stage = 2;
                        description = "Slowing down.";
                        ROS_INFO("Slowing down stage.");
                    }
                }
                else if (stage == 2) // Slow-down stage
                {
                    // Slowing down
                    if (velocity > 0.0)
                    {
                        velocity = velocity - elapsed*ang_acc;
                        if (velocity < 0.0)
                        {
                            velocity = 0.0;
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
                else if (stage == 3) // Stopping stage
                {
                    this->stopCommandPublisher.publish(stopCommand);
                    if (!L1IsActive && !R1IsActive && !L2IsActive && !R2IsActive && !L3IsActive && !R3IsActive)
                    {
                        ROS_INFO("Stopped.");
                        break;
                    }
                }
            }
            else if (gait_mode == "Stationary/Position")
            {
                // TODO: Add Stationary/... options to combine with AIK code
            }
            else if (gait_mode == "Stationary/Orientation")
            {
                // TODO: Add Stationary/... options to combine with AIK code
            }
            
            // Publish feedback
            this->actionFeedback.distance = traveled;
            this->actionFeedback.time = elapsed;
            this->actionFeedback.description = description;
            server.publishFeedback(this->actionFeedback);

            rate.sleep();
        }

        // Publish result
        this->actionResult.distance = traveled;
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


    // TODO: Replace struct with geometry_msgs::Vector3 ?
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
    geometry_msgs::Point GetPosition()
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = "body";
        linkStateMsg.request.reference_frame = "world";
        this->linkStateClient.call(linkStateMsg);
        return linkStateMsg.response.link_state.pose.position;
    }

    double GetOrientation()
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = "body";
        linkStateMsg.request.reference_frame = "world";
        this->linkStateClient.call(linkStateMsg);
        return linkStateMsg.response.link_state.pose.orientation.z;
    }

    std::string actionName;
    ros::NodeHandle node;
    actionlib::SimpleActionServer<hexapod_teleop::TeleopAction> server;
    actionlib::SimpleActionClient<hexapod_control::GaitAction> L1_client;
    actionlib::SimpleActionClient<hexapod_control::GaitAction> L2_client;
    actionlib::SimpleActionClient<hexapod_control::GaitAction> L3_client;
    actionlib::SimpleActionClient<hexapod_control::GaitAction> R1_client;
    actionlib::SimpleActionClient<hexapod_control::GaitAction> R2_client;
    actionlib::SimpleActionClient<hexapod_control::GaitAction> R3_client;
    bool L1IsActive = false;
    bool L2IsActive = false;
    bool L3IsActive = false;
    bool R1IsActive = false;
    bool R2IsActive = false;
    bool R3IsActive = false;
    hexapod_teleop::TeleopFeedback actionFeedback;
    hexapod_teleop::TeleopResult actionResult;
    ros::Publisher dutyRatioPublisher;
    ros::Publisher bodyVelocityPublisher;
    ros::Publisher velocityAnglePublisher;
    ros::Publisher stopCommandPublisher;
    ros::ServiceClient linkStateClient;
    std::string gait_mode;
    geometry_msgs::Twist base_twist;
    geometry_msgs::Vector3 base_pos, base_rot;
    double hex_pos, hex_heading, hex_rot;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Pose Action Server...");
    ros::init(argc, argv, "gait_controller");
    ROS_INFO("Initialized ros...");

    GaitController actionServer("gait_controller");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}