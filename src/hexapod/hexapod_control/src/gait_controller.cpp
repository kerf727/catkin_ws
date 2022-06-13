#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/SetPoseAction.h"
#include "hexapod_control/Pose.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "gait_controller_library.h"

class GaitController
{
public:
    GaitController() :
        client_L1("leg_L1_pose_action", true),
        client_L2("leg_L2_pose_action", true),
        client_L3("leg_L3_pose_action", true),
        client_R1("leg_R1_pose_action", true),
        client_R2("leg_R2_pose_action", true),
        client_R3("leg_R3_pose_action", true)
    {
        this->node = node;

        ROS_INFO("Subscribing to Teleop...");
		this->twistSubscriber = node.subscribe("/hexapod/teleop/twist", 10, &GaitController::twistCB, this);
		this->buttonSubscriber = node.subscribe("/hexapod/teleop/button", 10, &GaitController::buttonCB, this);
		
        ROS_INFO("Waiting for Pose State Servers...");
        this->client_L1.waitForServer(ros::Duration(30));
        this->client_L2.waitForServer(ros::Duration(30));
        this->client_L3.waitForServer(ros::Duration(30));
        this->client_R1.waitForServer(ros::Duration(30));
        this->client_R2.waitForServer(ros::Duration(30));
        this->client_R3.waitForServer(ros::Duration(30));

        ROS_INFO("Initializing constants...");
        node.getParam("/hexapod/geometry/coxa_length", coxa_length);
        node.getParam("/hexapod/geometry/femur_length", femur_length);
        node.getParam("/hexapod/geometry/tibia_length", tibia_length);
        node.getParam("/hexapod/geometry/base/radius", base_radius);
        node.getParam("/hexapod/geometry/base/height", base_height);
        node.getParam("/hexapod/geometry/leg_L1/hip_angle", hip_angle_L1);
        node.getParam("/hexapod/geometry/leg_L2/hip_angle", hip_angle_L2);
        node.getParam("/hexapod/geometry/leg_L3/hip_angle", hip_angle_L3);
        node.getParam("/hexapod/geometry/leg_R1/hip_angle", hip_angle_R1);
        node.getParam("/hexapod/geometry/leg_R2/hip_angle", hip_angle_R2);
        node.getParam("/hexapod/geometry/leg_R3/hip_angle", hip_angle_R3);
        
        // Convert to radians
        hip_angle_L1 = hip_angle_L1*M_PI/180.0;
        hip_angle_L2 = hip_angle_L2*M_PI/180.0;
        hip_angle_L3 = hip_angle_L3*M_PI/180.0;
        hip_angle_R1 = hip_angle_R1*M_PI/180.0;
        hip_angle_R2 = hip_angle_R2*M_PI/180.0;
        hip_angle_R3 = hip_angle_R3*M_PI/180.0;

        start = ros::Time::now().toSec();
        timer = node.createTimer(ros::Duration(publish_rate), boost::bind(&GaitController::updateAllLegs, this));

        std::tie(dcw, dwi) = calcStepRadius(base_height, coxa_length, femur_length, tibia_length);
        cw_L1 = calcCw(base_height, base_radius, hip_angle_L1, dcw);
        cw_L2 = calcCw(base_height, base_radius, hip_angle_L2, dcw);
        cw_L3 = calcCw(base_height, base_radius, hip_angle_L3, dcw);
        cw_R1 = calcCw(base_height, base_radius, hip_angle_R1, dcw);
        cw_R2 = calcCw(base_height, base_radius, hip_angle_R2, dcw);
        cw_R3 = calcCw(base_height, base_radius, hip_angle_R3, dcw);
        
        // Initialize legs to center of workspace
        ci_L1 = cw_L1;
        ci_L2 = cw_L2;
        ci_L3 = cw_L3;
        ci_R1 = cw_R1;
        ci_R2 = cw_R2;
        ci_R3 = cw_R3;

        node.getParam("/hexapod/gait/" + gait_type + "/max_speed", max_speed);
        node.getParam("/hexapod/gait/" + gait_type + "/max_yaw", max_yaw);
        node.getParam("/hexapod/gait/" + gait_type + "/duty_factor", duty_factor);
        node.getParam("/hexapod/gait/" + gait_type + "/stride_height", stride_height);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/L1", init_phase_L1);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/L2", init_phase_L2);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/L3", init_phase_L3);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/R1", init_phase_R1);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/R2", init_phase_R2);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/R3", init_phase_R3);

        gait_mode = "Default";
        last_gait_mode = gait_mode;
        speed = 0.0;
        yaw = yaw_eps;
        yaw_angle = 0.0;
        last_yaw = yaw;
        last_yaw_angle = yaw_angle;
        
        lowering_point = 0.9;
        verbose_global = 2;

        ROS_INFO("Gait controller ready.");
    }

    ~GaitController()
    {
        this->node.shutdown();
    }

private:
    ros::NodeHandle node;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> client_L1;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> client_L2;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> client_L3;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> client_R1;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> client_R2;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> client_R3;
    ros::Subscriber twistSubscriber;
    ros::Subscriber buttonSubscriber;
    ros::ServiceClient linkStateClient;
    hexapod_control::Pose current_pose;
    double publish_rate = 0.02; // 50Hz
    ros::Timer timer;
    boost::mutex publish_mutex;
    double start, t, elapsed, last_elapsed, Tc, delta_phi;
    double femur_length, tibia_length, coxa_length, base_radius, base_height;
    double hip_angle_L1, hip_angle_L2, hip_angle_L3, hip_angle_R1, hip_angle_R2, hip_angle_R3;
    double init_phase_L1, init_phase_L2, init_phase_L3, init_phase_R1, init_phase_R2, init_phase_R3;
    double leg_speed_L1, leg_speed_L2, leg_speed_L3, leg_speed_R1, leg_speed_R2, leg_speed_R3;
	double stride_height, duty_factor, lowering_point;
    double dwi, dcw;
    geometry_msgs::Point cm;
    geometry_msgs::Point ci_L1, ci_L2, ci_L3, ci_R1, ci_R2, ci_R3;
    geometry_msgs::Point cw_L1, cw_L2, cw_L3, cw_R1, cw_R2, cw_R3;
    int p_L1 = 1;
    int p_L2 = 1;
    int p_L3 = 1;
    int p_R1 = 1;
    int p_R2 = 1;
    int p_R3 = 1;
    bool init_L1 = false;
    bool init_L2 = false;
    bool init_L3 = false;
    bool init_R1 = false;
    bool init_R2 = false;
    bool init_R3 = false;
    WalkState s_L1 = WalkState::Idle;
    WalkState s_L2 = WalkState::Idle;
    WalkState s_L3 = WalkState::Idle;
    WalkState s_R1 = WalkState::Idle;
    WalkState s_R2 = WalkState::Idle;
    WalkState s_R3 = WalkState::Idle;
    std::string gait_mode, last_gait_mode;
    double speed, yaw, yaw_angle;
    double last_yaw, last_yaw_angle;
    bool new_command_flag = false;
    double yaw_eps = 1e-6;
    double max_speed, max_yaw;
    double rm, dir;
    double min_next_step;
    bool B_button = false;
    int gait_counter = 0;
    std::string gait_type = "tetrapod";
    double eps = 0.05;
    int verbose_global = 3;

    void buttonCB(const std_msgs::BoolConstPtr& msg)
    {
        B_button = msg->data;

        if (B_button)
        {
            gait_counter += 1;
            if (gait_counter > 2)
            {
                gait_counter = 0;
            }

            switch(gait_counter)
            {
                case(0):
                    gait_type = "tetrapod";
                    break;
                case(1):
                    gait_type = "tripod";
                    break;
                case(2):
                    gait_type = "wave";
                    break;
            }

            node.getParam("/hexapod/gait/" + gait_type + "/max_speed", max_speed);
            node.getParam("/hexapod/gait/" + gait_type + "/max_yaw", max_yaw);
            node.getParam("/hexapod/gait/" + gait_type + "/duty_factor", duty_factor);
            node.getParam("/hexapod/gait/" + gait_type + "/stride_height", stride_height);
            node.getParam("/hexapod/gait/" + gait_type + "/phase/L1", init_phase_L1);
            node.getParam("/hexapod/gait/" + gait_type + "/phase/L2", init_phase_L2);
            node.getParam("/hexapod/gait/" + gait_type + "/phase/L3", init_phase_L3);
            node.getParam("/hexapod/gait/" + gait_type + "/phase/R1", init_phase_R1);
            node.getParam("/hexapod/gait/" + gait_type + "/phase/R2", init_phase_R2);
            node.getParam("/hexapod/gait/" + gait_type + "/phase/R3", init_phase_R3);
        }
    }

    void twistCB(const geometry_msgs::TwistConstPtr& msg)
    {
        double linearX = msg->linear.x;
        double linearY = msg->linear.y;
        double angular = msg->angular.x;

        // Dead zone
        double deadzone = 0.0; // placeholder if deadzone is necessary for a controller
        if (abs(linearX) < deadzone)
        {
            linearX = 0.0;
        }
        if (abs(linearY) < deadzone)
        {
            linearY = 0.0;
        }
        if (abs(angular) < deadzone)
        {
            angular = 0.0;
        }

        // Map joystick data to speed, yaw, yaw_angle
        // Strafe (Lx and Ly only)
        if ((linearX != 0.0 || linearY != 0.0) && angular == 0.0)
        {  
            gait_mode = "Strafe";
            speed = mapRange(abs(linearX) + abs(linearY), 0.0, 1.0, 0.0, max_speed);
            yaw_angle = atan2(linearY, linearX);
            yaw = yaw_eps;
            dir = 1.0;
        }
        // Rotate (Rx only)
        else if (linearY == 0.0 && angular != 0.0)
        {
            gait_mode = "Rotate";
            speed = 0.0;
            yaw = mapRange(angular, -1.0, 1.0, -max_yaw, max_yaw);
            yaw_angle = 0.0;
            dir = (yaw > 0.0) ? 1.0 : -1.0;
        }
        // Steer (Ly and Rx) -- Currently not working --
        // else if (linearY != 0.0 && angular != 0.0)
        // {
            // TODO: make this a separate mode controlled by a button? imagine pressing Ly and then suddenly pressing Rx
            // gait_mode = "Steer";
            // speed = mapRange(abs(linearY), 0.0, 1.0, 0.0, max_speed);
            // yaw = mapRange(angular, -1.0, 1.0, -max_yaw, max_yaw);
            // yaw_angle = mapRange(abs(angular), 0.0, 1.0, 0.0, M_PI/3.0);
            // if (angular > 0.0)
            // {
            //     yaw_angle = M_PI - yaw_angle;
            // }
            // dir = (yaw > 0.0) ? 1.0 : -1.0;
        // }
        // Default
        else
        {
            gait_mode = "Default";
            speed = 0.0;
            yaw = yaw_eps;
            yaw_angle = 0.0;
            dir = 1.0;
        }

        // Clamp max/min values
        std::clamp(speed, 0.0, max_speed);
        std::clamp(yaw, -max_yaw, max_yaw);

        // Prevent yaw from being zero
        if (yaw == 0.0)
        {
            yaw = yaw_eps;
        }

        if (verbose_global >= 2)
        {
            ROS_INFO("--------------------------------------------------------------------------------");
            ROS_INFO("mode: %s, speed: %f, yaw: %f, yaw_angle: %f\n",
                gait_mode.c_str(), speed, yaw, yaw_angle);
        }
    }

    void updateAllLegs()
    {
        boost::mutex::scoped_lock lock(publish_mutex);

        elapsed = ros::Time::now().toSec() - start;
        Tc = 0.0; // don't allow time to pass if not moving
        if (gait_mode != "Default") // if moving
        {
            Tc = elapsed - last_elapsed; // control interval
        }
        last_elapsed = elapsed;
        delta_phi = yaw*Tc;

        // Detect if a new command has been sent
        new_command_flag = false;
        if (gait_mode != last_gait_mode || yaw != last_yaw || yaw_angle != last_yaw_angle)
        {
            new_command_flag = true;
        }
        
        rm = speed/yaw; // distance from body center to motion center
        cm = calcCm(yaw_angle, rm, base_height); // motion center

        min_next_step = 1e6; // reset minimum before next round

        updateOneLeg(ci_L1, cw_L1, s_L1, p_L1, init_phase_L1, init_L1, leg_speed_L1, 0);
        updateOneLeg(ci_L2, cw_L2, s_L2, p_L2, init_phase_L2, init_L2, leg_speed_L2, 2);
        updateOneLeg(ci_L3, cw_L3, s_L3, p_L3, init_phase_L3, init_L3, leg_speed_L3, 0);
        updateOneLeg(ci_R1, cw_R1, s_R1, p_R1, init_phase_R1, init_R1, leg_speed_R1, 0);
        updateOneLeg(ci_R2, cw_R2, s_R2, p_R2, init_phase_R2, init_R2, leg_speed_R2, 0);
        updateOneLeg(ci_R3, cw_R3, s_R3, p_R3, init_phase_R3, init_R3, leg_speed_R3, 0);
 
        sendPoseGoal(client_L1, ci_L1, eps);
        sendPoseGoal(client_L2, ci_L2, eps);
        sendPoseGoal(client_L3, ci_L3, eps);
        sendPoseGoal(client_R1, ci_R1, eps);
        sendPoseGoal(client_R2, ci_R2, eps);
        sendPoseGoal(client_R3, ci_R3, eps);

        last_gait_mode = gait_mode;
        last_yaw = yaw;
        last_yaw_angle = yaw_angle;

        if (verbose_global >= 3)
        {
            ROS_INFO("state: %d, L1: (%f, %f, %f)", p_L1, ci_L1.x, ci_L1.y, ci_L1.z);
            ROS_INFO("state: %d, L2: (%f, %f, %f)", p_L2, ci_L2.x, ci_L2.y, ci_L2.z);
            ROS_INFO("state: %d, L3: (%f, %f, %f)", p_L3, ci_L3.x, ci_L3.y, ci_L3.z);
            ROS_INFO("state: %d, R1: (%f, %f, %f)", p_R1, ci_R1.x, ci_R1.y, ci_R1.z);
            ROS_INFO("state: %d, R2: (%f, %f, %f)", p_R2, ci_R2.x, ci_R2.y, ci_R2.z);
            ROS_INFO("state: %d, R3: (%f, %f, %f)", p_R3, ci_R3.x, ci_R3.y, ci_R3.z);
        }
    }

    void updateOneLeg(
        geometry_msgs::Point& ci, const geometry_msgs::Point& cw,
        WalkState& state, int& gait_phase, const double& init_phase, bool& initialized,
        double& leg_speed, const int& verbose)
    {
        geometry_msgs::Point AEP, PEP, cm_to_AEP, cm_to_PEP;
        double phi_i, phi_w, phi_AEP, phi_PEP;
        double rmw, theta;

        phi_i = calcPhiI(cm, ci);
        std::tie(phi_w, rmw) = calcPhiW(cm, cw);

        theta = calcTheta(rmw, dwi);
        std::tie(AEP, PEP) = calcAEPPEP(cm, rmw, phi_w, dir, theta, base_height);
        std::tie(phi_i, phi_AEP, phi_PEP) = calcPhiAEPPEP(cm, AEP, PEP, phi_i);

        switch(state)
        {
            case WalkState::Idle :
                if (verbose >= 1)
                {
                    ROS_INFO("Idle State");
                }

                leg_speed = Idle(ci, cw, PEP, gait_phase, init_phase, theta, verbose);

                if (!initialized && gait_mode != "Default")
                {
                    toggleState(state);
                }
                break;
            
            case WalkState::Initialize :
                if (verbose >= 1)
                {
                    ROS_INFO("Initialize State");
                }

                Initialize(ci, gait_phase, init_phase, initialized, leg_speed, phi_i, PEP, phi_PEP, verbose);

                if (initialized)
                {
                    toggleState(state);
                }
                break;

            case WalkState::Move :
                if (verbose >= 1)
                {
                    ROS_INFO("Move State, gait_phase: %d", gait_phase);
                }

                Move(ci, gait_phase, rmw, phi_i, AEP, PEP, phi_AEP, phi_PEP, verbose);

                if (initialized && gait_mode == "Default")
                {
                    toggleState(state);
                }
                break;

            case WalkState::Uninitialize :
                if (verbose >= 1)
                {
                    ROS_INFO("Uninitialize State");
                }

                Uninitialize(ci, gait_phase, init_phase, initialized);

                if (!initialized)
                {
                    toggleState(state);
                }
                break;
        };

        if (verbose >= 2)
        {
            if (gait_mode != "Default")
            {
                ROS_INFO("dir: %.3f, rmw: %.3f, theta: %4.3e, Tc: %.3f", dir, rmw, theta, Tc);
                ROS_INFO("AEP: (%.3f, %.3f, %.3f); PEP: (%.3f, %.3f, %.3f)", AEP.x, AEP.y, AEP.z, PEP.x, PEP.y, PEP.z);
                ROS_INFO("cm: (%.3f, %.3f);    ci: (%.3f, %.3f, %.3f)\n", cm.x, cm.y, ci.x, ci.y, ci.z);
            }
            else
            {
                ROS_INFO("Default. ci: (%.3f, %.3f, %.3f)\n", ci.x, ci.y, ci.z);
            }
        }
    }

    double Idle(
        geometry_msgs::Point& ci, const geometry_msgs::Point& cw,
        const geometry_msgs::Point& PEP, int& gait_phase,
        const double& init_phase, const double& theta, const int& verbose)
    {
        double stride_time, support_time, support_phase;
        double remaining_support_time, leg_speed;
        double dist_ci_PEP;

        ci = cw; // constant for idle state
        gait_phase = 1;
        
        // TODO: update to only run below code once?
        stride_time = abs(2.0*theta/yaw);
        support_time = stride_time*duty_factor;
        if (init_phase == 0.0)
        {
            support_phase = 0.0;
        }
        else
        {
            support_phase = (1.0 - init_phase)/duty_factor;
        }
        remaining_support_time = (1.0 - support_phase)*support_time;
        dist_ci_PEP = sqrt(pow(PEP.x - ci.x, 2) + pow(PEP.y - ci.y, 2));
        leg_speed = dist_ci_PEP/remaining_support_time;
        
        if (verbose >= 2)
        {
            ROS_INFO("stride_time: %.3f, support_phase: %.3f", stride_time, support_phase);
            // ROS_INFO("remaining_support_time: %.3f, dist_ci_PEP: %.3f, leg_speed: %.3f", remaining_support_time, dist_ci_PEP, leg_speed);
        }

        return leg_speed;
    }

    void Initialize(
        geometry_msgs::Point& ci, int& gait_phase,
        const double& init_phase, bool& initialized,
        const double& leg_speed, const double& phi_i,
        const geometry_msgs::Point& PEP, const double& phi_PEP,
        const int& verbose)
    {
        // Assumes that robot legs will only ever start in support phase due to designed phase overlap

        double phi_minus, leg_pos;

        gait_phase = 1;
        phi_minus = dir*(phi_PEP - phi_i);
        if (verbose >= 2)
        {
            ROS_INFO("phi_minus: %4.3e", phi_minus);
        }

        // Reached PEP, set to initialized
        if (phi_minus <= 0.0)
        {
            ci = PEP;
            gait_phase = 2;
            initialized = true;
            if (verbose >= 2)
            {
                ROS_INFO("Initialized");
            }
            return;
        }

        leg_pos = leg_speed*Tc;
        ci.x += leg_pos*cos(yaw_angle + M_PI);
        ci.y += leg_pos*sin(yaw_angle + M_PI);
        ci.z = -base_height;

        if (verbose >= 2)
        {
            ROS_INFO("init_phase: %.3f, leg_speed: %.3f, leg_pos: %.4f", init_phase, leg_speed, leg_pos);
        }
    }

    void Move(
        geometry_msgs::Point& ci, int& gait_phase,
        const double& rmw, const double& phi_i,
        const geometry_msgs::Point& AEP, const geometry_msgs::Point& PEP,
        const double& phi_AEP, const double& phi_PEP,
        const int& verbose)
    {
        double alpha, d_AEP, d_PEP, PEP_to_AEP, phi_plus, phi_minus;
        double tG_plus, next_step, delta_tG, k_plus;

        alpha = atan2(AEP.y - ci.y, AEP.x - ci.x);
        d_AEP = sqrt(pow(AEP.x - ci.x, 2) + pow(AEP.y - ci.y, 2));
        PEP_to_AEP = sqrt(pow(AEP.x - PEP.x, 2) + pow(AEP.y - PEP.y, 2));
        phi_plus = dir*(phi_PEP - phi_i); // remaining support angle
        phi_minus = dir*(phi_i - phi_AEP); // remaining transfer angle

        if (gait_phase == 1)
        {
            tG_plus = phi_plus/yaw; // time left until end of current gait phase for each leg on ground
            next_step = phi_plus/tG_plus;

            if (next_step < min_next_step)
            {
                min_next_step = next_step;
            }
            ROS_INFO("next_step: %4.3e", next_step);
        }
        else
        {
            tG_plus = phi_minus/yaw; // this never happens so far
        }
        
        // TODO: doesn't wait to calculate for all legs in new loop - could unintentionally use old min
        // if (min_next_step > 0.0)
        // {
            delta_tG = abs(delta_phi/min_next_step);
            k_plus = round(tG_plus/delta_tG);
        // }
        // else
        // {
        //     delta_tG = 0.0;
        //     k_plus = 0.0;
        // }

        // delta_tG = abs(delta_phi/next_step);
        // k_plus = round(tG_plus/delta_tG);

        // Support Phase
        if (gait_phase == 1)
        {
            ci.x = rmw*cos(phi_i + delta_phi) + cm.x;
            ci.y = rmw*sin(phi_i + delta_phi) + cm.y;
            ci.z = -base_height;
            
            if (phi_minus <= 0.0)
            {
                ci = PEP;
                gait_phase = 2;
            }
        }
        // Transfer Phase - Lifting
        else if (gait_phase == 2)
        {
            // TODO: figure out how this can be synced to desired duty_factor
            ci.x = ci.x + (d_AEP + stride_height)*cos(alpha)/k_plus;
            ci.y = ci.y + (d_AEP + stride_height)*sin(alpha)/k_plus;
            ci.z = LPF1(-stride_height, ci.z, 0.2); // TODO: make dependent on k_plus?
            
            if (d_AEP < 0.005)
            {
                ci.x = AEP.x;
                ci.y = AEP.y;
                ci.z = -base_height + stride_height;
                gait_phase = 3;
            }
        }
        // Transfer Phase - Lowering
        else if (gait_phase == 3)
        {
            // TODO: figure out how this can be synced to desired duty_factor
            ci.x = AEP.x;
            ci.y = AEP.y;
            ci.z -= (stride_height + ci.z)/k_plus;

            if (ci.z <= -base_height)
            {
                ci = AEP;
                gait_phase = 1;
            }
        }

        if (verbose >= 2)
        {
            ROS_INFO("phi_plus: %4.3e, phi_minus: %4.3e, d_AEP: %.3f, alpha: %.3f", phi_plus, phi_minus, d_AEP, alpha);
            ROS_INFO("tG+: %.3f, delta_tG: %4.3e, next_step: %4.3e, min_n_s: %4.3e, k+: %.0f", tG_plus, delta_tG, next_step, min_next_step, k_plus);
        }
    }

    void Uninitialize(
        geometry_msgs::Point& ci, int& gait_phase,
        const double& init_phase, bool& initialized)
    {
        // Support Phase
        if (gait_phase == 1)
        {
            // lift up and place back down at cw location
            // timing to lift up determined by initial phase
        }
        // Transfer Phase
        else if (gait_phase == 2 || gait_phase == 3)
        {
            // place back down at cw location
        }

        // once done with above procedure
        initialized = false;
    }

    void toggleState(WalkState& current_state)
    {
        current_state = stateTransitions[current_state];
    }

    void sendPoseGoal(
        actionlib::SimpleActionClient<hexapod_control::SetPoseAction>& client,
        const geometry_msgs::Point& ci, const double& eps)
    {
        // Build message
        hexapod_control::Pose target_pose;
        target_pose.x = ci.x;
        target_pose.y = ci.y;
        target_pose.z = ci.z;
        target_pose.rotx = std::vector<double>{1.0, 0.0, 0.0};
        target_pose.roty = std::vector<double>{0.0, 1.0, 0.0};
        target_pose.rotz = std::vector<double>{0.0, 0.0, 1.0};

        // Send goal to pose action client
        hexapod_control::SetPoseGoal poseAction;
        poseAction.eps = eps;
        poseAction.goal = target_pose;

        client.sendGoal(poseAction,
            boost::bind(&GaitController::publishResult, this, _1, _2),
            boost::bind(&GaitController::activeCB, this),
            boost::bind(&GaitController::publishFeedback, this, _1));
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
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Pose Action Server...");
    ros::init(argc, argv, "gait_controller");
    ROS_INFO("Initialized ros...");

    GaitController gait_controller;
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}