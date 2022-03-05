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
    GaitController(std::string name):
        L1_client("leg_L1_trajectory_action", true),
        L2_client("leg_L2_trajectory_action", true),
        L3_client("leg_L3_trajectory_action", true),
        R1_client("leg_R1_trajectory_action", true),
        R2_client("leg_R2_trajectory_action", true),
        R3_client("leg_R3_trajectory_action", true)
    {
        this->node = node;

        ROS_INFO("Subscribing to Teleop...");
		this->gaitModeSubscriber = node.subscribe("/hexapod/gait/gait_mode", 10, &GaitController::gaitModeCB, this);
		this->baseTwistSubscriber = node.subscribe("/hexapod/gait/base_twist", 10, &GaitController::baseTwistCB, this);
		this->hexPosSubscriber = node.subscribe("/hexapod/gait/hex_pos", 10, &GaitController::hexPosCB, this);
		this->hexRotSubscriber = node.subscribe("/hexapod/gait/hex_rot", 10, &GaitController::hexRotCB, this);
        
        ROS_INFO("Publishing to Trajectory Action Servers...");
        this->velocityPublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/velocity", 1);
        this->strideTimePublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/stride_time", 1);
        this->strideHeightPublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/stride_height", 1);
        this->strideLengthPublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/stride_length", 1);
        this->stopCommandPublisher = node.advertise<std_msgs::Bool>("/hexapod/gait/stop", 1);
        
        // Initialize subscribed variables
        gait_mode = "Moving/Position";
        base_twist.linear.x = 0.0;
        base_twist.linear.y = 0.0;
        base_twist.linear.z = 0.0;
        base_twist.angular.x = 0.0;
        base_twist.angular.y = 0.0;
        base_twist.angular.z = 0.0;
        hex_pos = 0.04; // TODO: put these two into gait_parameters?
        hex_rot = 10 * M_PI / 180;

        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

        ROS_INFO("Waiting for Leg Trajectory Servers...");
        this->L1_client.waitForServer(ros::Duration(30));
        this->L2_client.waitForServer(ros::Duration(30));
        this->L3_client.waitForServer(ros::Duration(30));
        this->R1_client.waitForServer(ros::Duration(30));
        this->R2_client.waitForServer(ros::Duration(30));
        this->R3_client.waitForServer(ros::Duration(30));

        ROS_INFO("Gait controller ready.");
    }

    ~GaitController()
    {
        this->node.shutdown();
    }

    void publish()
    {
        base_pos.x = base_twist.linear.x;
        base_pos.y = base_twist.linear.y;
        base_pos.z = base_twist.linear.z;
        base_rot.x = base_twist.angular.x;
        base_rot.y = base_twist.angular.y;
        base_rot.z = base_twist.angular.z;

        // Calculate gait parameters
        int numSteps;
        double lin_vel, lin_acc, ang_vel, ang_acc;
        double duty_factor, stride_time, stride_height, stride_length;
        double phase_L1, phase_L2, phase_L3, phase_R1, phase_R2, phase_R3;
        std::vector<double> relative_phases;

        // TODO: control gait type from teleop
        std::string gait_type = "ripple";

        // Get parameters from parameter server
        node.getParam("/hexapod/gait/" + gait_type + "/linear_velocity", lin_vel);
        node.getParam("/hexapod/gait/" + gait_type + "/linear_acceleration", lin_acc);
        node.getParam("/hexapod/gait/" + gait_type + "/angular_velocity", ang_vel);
        node.getParam("/hexapod/gait/" + gait_type + "/angular_acceleration", ang_acc);
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

        // Get initial position
        geometry_msgs::Point initial_pos = GetPosition();
        double initial_angle = GetOrientation();

        // Get current time
        double start = ros::Time::now().toSec();

        // Publish initial gait parameters
        double velocity = 0.0;
        std_msgs::Float64 velocityMsg;
        velocityMsg.data = velocity;
        this->velocityPublisher.publish(velocityMsg);

        std_msgs::Float64 strideTimeMsg;
        strideTimeMsg.data = stride_time;
        this->strideTimePublisher.publish(strideTimeMsg);

        std_msgs::Float64 strideHeightMsg;
        strideHeightMsg.data = stride_height;
        this->strideHeightPublisher.publish(strideHeightMsg);

        std_msgs::Float64 strideLengthMsg;
        strideLengthMsg.data = stride_length;
        this->strideLengthPublisher.publish(strideLengthMsg);

        std_msgs::Bool stopCommand;
        stopCommand.data = true;

        // Send goals to trajectory servers
        ROS_INFO("Sending goal to leg trajectories...");
        hexapod_control::GaitGoal gaitAction;
        gaitAction.gait_mode = gait_mode;
        gaitAction.duty_factor = duty_factor;
        gaitAction.initial_phase = phase_L1;
        this->L1_client.sendGoal(gaitAction,
            boost::bind(&GaitController::L1Result, this, _1, _2),
            boost::bind(&GaitController::L1Active, this),
            boost::bind(&GaitController::L1Feedback, this, _1));
        gaitAction.initial_phase = phase_L2;
        this->L2_client.sendGoal(gaitAction,
            boost::bind(&GaitController::L2Result, this, _1, _2),
            boost::bind(&GaitController::L2Active, this),
            boost::bind(&GaitController::L2Feedback, this, _1));
        gaitAction.initial_phase = phase_L3;
        this->L3_client.sendGoal(gaitAction,
            boost::bind(&GaitController::L3Result, this, _1, _2),
            boost::bind(&GaitController::L3Active, this),
            boost::bind(&GaitController::L3Feedback, this, _1));
        gaitAction.initial_phase = phase_R1;
        this->R1_client.sendGoal(gaitAction,
            boost::bind(&GaitController::R1Result, this, _1, _2),
            boost::bind(&GaitController::R1Active, this),
            boost::bind(&GaitController::R1Feedback, this, _1));
        gaitAction.initial_phase = phase_R2;
        this->R2_client.sendGoal(gaitAction,
            boost::bind(&GaitController::R2Result, this, _1, _2),
            boost::bind(&GaitController::R2Active, this),
            boost::bind(&GaitController::R2Feedback, this, _1));
        gaitAction.initial_phase = phase_R3;
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

            ROS_INFO("traveled: %f, hex_pos: %f", traveled, hex_pos);

            // Check if preempted
            if (!ros::ok())
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
                ROS_INFO("cur: (%f, %f), ini: (%f, %f), dt: %f", current_pos.x, current_pos.y, initial_pos.x, initial_pos.y, distance_traveled);

                if (stage == 0) // Ramp-up stage
                {
                    // Ramping up
                    if (velocity < lin_vel)
                    {
                        velocity = velocity + elapsed*lin_acc;
                        if (velocity > lin_vel)
                        {
                            velocity = lin_vel;
                        }
                        velocityMsg.data = velocity;
                        this->velocityPublisher.publish(velocityMsg);
                    }
                    // Hit target velocity
                    else
                    {
                        velocityMsg.data = lin_vel;
                        this->velocityPublisher.publish(velocityMsg);
                        ramp_time = elapsed;
                        ramp_distance = distance_traveled;
                        ROS_INFO("RAMP DISTANCE: %f", ramp_distance);
                        stage = 1;
                        description = "Constant velocity.";
                        ROS_INFO("Constant velocity stage.");
                    }
                }
                else if (stage == 1) // Constant velocity stage
                {
                    velocityMsg.data = lin_vel;
                    this->velocityPublisher.publish(velocityMsg);
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
                        velocityMsg.data = velocity;
                        this->velocityPublisher.publish(velocityMsg);
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
                    if (abs(velocity) < ang_vel)
                    {
                        if (hex_rot > 0.0)
                        {
                            velocity += elapsed*ang_acc;
                            if (velocity > ang_vel)
                            {
                                velocity = ang_vel;
                            }
                        }
                        else
                        {
                            velocity -= elapsed*ang_acc;
                            if (velocity < -ang_vel)
                            {
                                velocity = -ang_vel;
                            }
                        }
                        velocityMsg.data = velocity;
                        this->velocityPublisher.publish(velocityMsg);
                    }
                    // Hit target velocity
                    else
                    {
                        if (hex_rot > 0.0)
                        {
                            velocityMsg.data = ang_vel;
                        }
                        else
                        {
                            velocityMsg.data = -ang_vel;
                        }
                        this->velocityPublisher.publish(velocityMsg);
                        ramp_time = elapsed;
                        ramp_angle = angle_traveled;
                        stage = 1;
                        description = "Constant velocity.";
                        ROS_INFO("Constant velocity stage.");
                    }
                }
                else if (stage == 1) // Constant velocity stage
                {
                    if (hex_rot > 0.0)
                        {
                            velocityMsg.data = ang_vel;
                        }
                        else
                        {
                            velocityMsg.data = -ang_vel;
                        }
                    this->velocityPublisher.publish(velocityMsg);
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
                    if (abs(velocity) > 0.0)
                    {
                        if (hex_rot > 0.0)
                        {
                            velocity -= elapsed*ang_acc;
                            if (velocity < 0.0)
                            {
                                velocity = 0.0;
                            }
                        }
                        else
                        {
                            velocity += elapsed*ang_acc;
                            if (velocity > 0.0)
                            {
                                velocity = 0.0;
                            }
                        }
                        velocityMsg.data = velocity;
                        this->velocityPublisher.publish(velocityMsg);
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
                // TODO: Add AIK code
            }
            else if (gait_mode == "Stationary/Orientation")
            {
                // TODO: Add AIK code
            }
            else
            {

            }
            
            // ros::spinOnce();
            
            rate.sleep();
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

    void gaitModeCB(const std_msgs::StringConstPtr& msg)
	{
		this->gait_mode = msg->data;
        GaitController::publish();
	}

    void baseTwistCB(const geometry_msgs::TwistConstPtr& msg)
	{
		this->base_twist.linear = msg->linear;
		this->base_twist.angular = msg->angular;
	}

    void hexPosCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->hex_pos = msg->data;
	}

    void hexRotCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->hex_rot = msg->data;
	}

private:
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

    ros::NodeHandle node;
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
    ros::Publisher velocityPublisher;
    ros::Publisher strideTimePublisher;
    ros::Publisher strideHeightPublisher;
    ros::Publisher strideLengthPublisher;
    ros::Publisher stopCommandPublisher;
    ros::Subscriber gaitModeSubscriber;
    ros::Subscriber baseTwistSubscriber;
    ros::Subscriber hexPosSubscriber;
    ros::Subscriber hexHeadingSubscriber;
    ros::Subscriber hexRotSubscriber;
    ros::ServiceClient linkStateClient;
    std::string gait_mode;
    geometry_msgs::Twist base_twist;
    geometry_msgs::Vector3 base_pos, base_rot;
    double hex_pos, hex_rot;
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Pose Action Server...");
    ros::init(argc, argv, "gait_controller");
    ROS_INFO("Initialized ros...");

    GaitController gait_controller("gait_controller");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}