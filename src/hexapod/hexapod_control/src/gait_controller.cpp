#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/GaitAction.h"
#include "hexapod_control/MoveAction.h"
#include "hexapod_control/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "gazebo_msgs/GetLinkState.h"

//TODO: Remove trajectory action server, absorb that into this. Publish directly to pose action servers

class GaitController
{
public:
    GaitController(std::string name):
        server(node, name, boost::bind(&GaitController::executeCB, this, _1), false),
        L1_client("leg_L1_pose_action", true),
        R1_client("leg_R1_pose_action", true),
        L2_client("leg_L2_pose_action", true),
        R2_client("leg_R2_pose_action", true),
        L3_client("leg_L3_pose_action", true),
        R3_client("leg_R3_pose_action", true),
        actionName(name)
    {
        this->node = node;

        //TODO: Remove publishers because these parameters will be used internally
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

        // Send goals to pose action servers
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
        int stage = 0;
        double distanceTraveled, stride, rampTime, rampDistance;
        bool preempted = false;
        bool aborted = false;
        std::string description = "Ramping up.";

        initialized = false;
        initial_phase = relative_phases;
        current_phase = this->initial_phase;
        double t, elapsed;

        ROS_INFO("Ramp up stage.");
        ros::Rate rate(50);
        while (true)
        {
            // Calculate elapsed time
            elapsed = ros::Time::now().toSec() - start;
            t = elapsed + this->initial_phase * stride_time; // account for initial_phase
			current_phase = fmod(t / this->stride_time, 1.0);

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

            // Control gait (low-level)

            if(!initialized)
            {
                initializeHexapod();
                initialized = true;
            }
            else{
                for(int i = 0; i < n_legs; i++)
                {
                    if(current_phase[i] <= duty_factor)
                    {
                        // Stance Phase
                        stancePhase(i);
                    }
                    else
                    {
                        if(current_phase[i] <= duty_factor + 0.2) // need to decide this length
                        {
                            // Lifting Phase
                            liftingPhase(i);
                        }
                        else
                        {
                            // Lowering Phase
                            loweringPhase(i);
                        }
                    }

                    position[i] = hex.Ci[i];

                    // Send new goal position
                }
            }

            // Control gait (high-level)
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
                if (!frIsActive && !flIsActive && !brIsActive && !blIsActive)
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
	hexapod_control::Pose currentPose;
    int n_legs = 6;
	double initial_phase[6];
	double current_phase[6];
	double stride_time;
	double stride_height;
	double duty_factor;
    double Tc = 0.01; // control interval (sec)
    double femur_length = 0.04; // m
    double tibia_length = 0.08; // m
    double x_offset = -0.1695;
    double y_offset = 0.1524;
    double z_offset = -0.2855;
    double z_mh;
    int steps = 0;
    bool stop = false;
    bool initialized = false;
    std::string stage;

    struct _hex // current state of the robot
    {
        double current_phase[6];        // current leg phases
        double Fv;                      // body velocity (m/s)
        double Fw;                      // body angular velocity (rad/s)
        geometry_msgs::Point Cf;        // body position
        geometry_msgs::Point Cm;        // body motion center
        geometry_msgs::Vector3 r_fm;    // body to motion center distance
        double h_gr[6];                 // estimated ground height (m)
        double beta[6];                 // leg to ground angle
        double gamma[6];                // body to hip angle
        double r_ws[6];                 // workspace radius
        geometry_msgs::Point Cw[6];     // foot workspace centers
        geometry_msgs::Point Ci[6];     // foot positions
        double phase[6];                // leg phase
        geometry_msgs::Vector3 r_m[6];  // foot to motion center distance
        double phi_fm;                  // body to motion center angle
        double phi[6];                  // foot to motion center angle
        double phi_plus[6];             // angle to PEP (remaining)
        double phi_minus[6];            // angle to AEP (already travelled)
        double t_Gplus[6];              // time remaining in current gait phase
        double k_plus[6];               // max number of control intervals to reach the next AEP or PEP point
        double alpha[6];                // (swing) angle from foot to AEP point
        double d_AEP[6];                // (swing) distance from foot to AEP point
        geometry_msgs::Point AEP[6];    // anterior extreme position
        geometry_msgs::Point PEP[6];    // posterior extreme position
        
        // Constructor
        _hex():
        {
            Fv = ;
            Fw = ;
            Cf = ;
            Cm = ;
            r_fm = ;
            h_gr = ;
            beta = ;
            gamma = ;
            r_ws = ;
            Cw = ;
            Ci = ;
            phase = ;
            r_m = ;
            phi_fm = ;
            phi = ;
            phi_plus = ;
            phi_minus = ;
            t_Gplus = ;
            k_plus = ;
            alpha = ;
            d_AEP = ;
            AEP = ;
            PEP = ;
        }

        // Destructor
        ~_hex()
        {
            // delete struct data
        }

        void updateAll()
        {
            updateAEPandPEP();
            updatePhase();
            updatePhaseTiming();
            updateWorkspaceCenters();
        }

        // Update AEP and PEP points for each leg
        void updateAEPandPEP()
        {
            // https://stackoverflow.com/questions/3349125/circle-circle-intersection-points
            for(int i = 0; i < n_legs; i++)
            {
                double dx = Cw[i].x - Cm.x;
                double dy = Cw[i].y - Cm.y;
                double d = sqrt(dx*dx + dy*dy);

                if ((d > r_m[i] + r_ws[i]) ||
                    (d < fabs(r_m[i] - r_ws[i])) ||
                    (d == 0 && r_m[i] == r_ws[i]))
                {
                    ROS_ERROR("Cannot calculate distance between motion center and workspace center for leg %d", i);
                    break;
                }

                double a = ((r_m[i]*r_m[i]) - (r_ws[i]*r_ws[i]) + (d*d)) / (2.0*d) ;
                double x2 = Cm.x + (dx * a / d);
                double y2 = Cm.y + (dy * a / d);
                double h = sqrt(r_m[i]*r_m[i] - a*a);
                double rx = -dy * h / d;
                double ry = dx * h / d;
                
                AEP[i].x = x2 + rx;
                AEP[i].y = y2 - ry;
                AEP[i].z = h_gr[i];
                
                PEP[i].x = x2 - rx;
                PEP[i].y = y2 + ry;
                PEP[i].z = h_gr[i];
            }
        }

        // Update phase values for each leg
        void updatePhase()
        {
            for(int i = 0; i < n_legs; i++)
            {
                geometry_msgs::Vector3 vPEP = PEP[i] - Cm;
                geometry_msgs::Vector3 vAEP = AEP[i] - Cm;
                geometry_msgs::Vector3 vCi = Ci[i] - Cm;
                geometry_msgs::Vector3 vCm = [1, 0, 0]; // vector pointing in x-axis direction

                phi[i] = v2Dangle(vCi, vCm);
                phi_plus[i] = v2Dangle(vPEP, vCi);
                phi_minus[i] = v2Dangle(vAEP, vCi);
            }
            geometry_msgs::Vector3 vCf = Cf - Cm;
            phi_fm = v2Dangle(vCf, vCm);
        }

        // Update phase timing values for each leg
        void updatePhaseTiming()
        {
            double delta_phi = Fw * Tc;
            double min_remaining_steps = min(phi_plus[0] / t_Gplus[0],
                                             phi_plus[1] / t_Gplus[1],
                                             phi_plus[2] / t_Gplus[2],
                                             phi_plus[3] / t_Gplus[3],
                                             phi_plus[4] / t_Gplus[4],
                                             phi_plus[5] / t_Gplus[5]);
            double delta_t_G = delta_phi / min_remaining_steps;
            
            for(int i = 0; i < n_legs; i++)
            {
                t_Gplus[i] = phi_plus[i] / (delta_phi / delta_t_G);
                k_plus[i] = round(t_Gplus[i] / delta_t_G);
            }
        }

        // Update workspace centers for each leg
        void updateWorkspaceCenters()
        {
            for(int i = 0; i < n_legs; i++)
            {
                geometry_msgs::Point hip_position = getHipPosition(i);
                double beta[i] = asin(hip_position.z / (femur_length + tibia_length)); // angle from center of leg workspace to hip of leg i
                double r_ws[i] = (femur_length + tibia_length) / 2 * cos(beta[i]); // radius of workspace for leg i

                Cw[i].x = r_ws[i] * cos(gamma[i]) + hip_position.x;
                Cw[i].y = r_ws[i] * sin(gamma[i]) + hip_position.y;
                Cw[i].z = h_gr[i];
            }
        }
        
        // Calculate angle between two 2D vectors
        double v2Dangle(const geometry_msgs::Vector3& v1, const geometry_msgs::Vector3& v2)
        {
            v1dotv2 = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
            v1_length = v1.x*v1.x + v1.y*v1.y + v1.z*v1.z;
            v2_length = v2.x*v2.x + v2.y*v2.y + v2.z*v2.z;
            return acos(v1dotv2 / (v1_length * v2_length));
        }
    }

private:
    std::string getLegName(const int& i)
    {
        std::string legName;
        switch (i)
        {
        case 0:
            legName = "L1";
            break;
        case 1:
            legName = "R1";
            break;
        case 2:
            legName = "L2";
            break;
        case 3:
            legName = "R2";
            break;
        case 4:
            legName = "L3";
            break;
        case 5:
            legName = "R3";
            break;
        
        default:
            ROS_ERROR("Leg %d not defined. Must be between 0 and 5.", i);
            break;
        }

        return legName;
    }

    geometry_msgs::Point getPosition(const std::string linkName)
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = linkName;
        linkStateMsg.request.reference_frame = "world";
        this->linkStateClient.call(linkStateMsg);

        return linkStateMsg.response.link_state.pose.position;
    }

    geometry_msgs::Point getBodyPosition(const int& i)
    {
        return getPosition("body");
    }

    geometry_msgs::Point getHipPosition(const int& i)
    {
        return getPosition("hip_" + getLegName(i));
    }

    geometry_msgs::Point getFootPosition(const int& i)
    {
        return getPosition("foot_" + getLegName(i));
    }

    // 1st order low-pass filter
    double LPF1(const double& goal, const double& z_prev) {
        // https://dsp.stackexchange.com/questions/60277/is-the-typical-implementation-of-low-pass-filter-in-c-code-actually-not-a-typica
        double a = 0.1; // as a approaches 0, the cutoff of the filter decreases
        double z_next = (1 - a) * z_prev + a * goal; // "simplistic" low pass filter
        // double z_next = (1 - a) * z_prev + a * (goal + goal_prev) / 2; // "proper" low pass filter (zero at Nyquist and more attenuation close to Nyquist)

        return z_next;
    }

    void initializeHexapod()
    {
        _hex hex; // create instance of _hex struct
    }

    void stancePhase(const int& i)
    {
        double delta_phi = hex.Fw * Tc;
        hex.r_fm = hex.Fv / hex.Fw; // radius of center of motion
        hex.Cm.x = hex.r_fm * cos(hex.phi_fm - M_PI_2);
        hex.Cm.y = hex.r_fm * sin(hex.phi_fm - M_PI_2);
        hex.Cm.z = hex.h_gr[i];
        hex.r_m[i] = hex.Ci[i] - hex.Cm; // this is being double calculated (also in updatePhase)

        hex.Ci[i].x = hex.r_m[i]*cos(hex.phi[i] - delta_phi) + hex.Cm.x;
        hex.Ci[i].y = hex.r_m[i]*sin(hex.phi[i] - delta_phi) + hex.Cm.y;
        hex.Ci[i].z = hex.h_gr[i];
    }

    void liftingPhase(const int& i)
    {
        hex.alpha[i] = atan2(hex.AEP[i].y - hex.Ci[i].y, hex.AEP[i].x - hex.Ci[i].x);
        hex.d_AEP[i] = sqrt(pow(hex.AEP[i].x - hex.Ci[i].x, 2) + pow(hex.AEP[i].y - hex.Ci[i].y, 2));
        
        hex.Ci[i].x += (hex.d_AEP[i] + z_mh) * cos(hex.alpha[i]) / hex.k_plus[i];
        hex.Ci[i].y += (hex.d_AEP[i] + z_mh) * sin(hex.alpha[i]) / hex.k_plus[i];
        hex.Ci[i].z = LPF1(z_mh, hex.Ci[i].z);
    }

    void loweringPhase(const int& i)
    {
        hex.Ci[i].z += (hex.h_gr[i] - hex.Ci[i].z) / hex.k_plus[i];
    }
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
