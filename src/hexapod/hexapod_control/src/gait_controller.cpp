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
        publish_rate = 0.02; // 50Hz
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
        p_L1 = init_phase_L1;
        p_L2 = init_phase_L2;
        p_L3 = init_phase_L3;
        p_R1 = init_phase_R1;
        p_R2 = init_phase_R2;
        p_R3 = init_phase_R3;

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
    double publish_rate;
    ros::Timer timer;
    boost::mutex publish_mutex;
    double start, t, elapsed, last_elapsed, Tc;
    double femur_length, tibia_length, coxa_length, base_radius, base_height;
    double hip_angle_L1, hip_angle_L2, hip_angle_L3, hip_angle_R1, hip_angle_R2, hip_angle_R3;
    double init_phase_L1, init_phase_L2, init_phase_L3, init_phase_R1, init_phase_R2, init_phase_R3;
	double stride_height, duty_factor, lowering_point;
    double dwi, dcw;
    geometry_msgs::Point cm;
    geometry_msgs::Point ci_L1, ci_L2, ci_L3, ci_R1, ci_R2, ci_R3;
    geometry_msgs::Point cw_L1, cw_L2, cw_L3, cw_R1, cw_R2, cw_R3;
    double p_L1, p_L2, p_L3, p_R1, p_R2, p_R3;
    bool init_L1 = false;
    bool init_L2 = false;
    bool init_L3 = false;
    bool init_R1 = false;
    bool init_R2 = false;
    bool init_R3 = false;
    std::string gait_mode, last_gait_mode;
    double speed, yaw, yaw_angle;
    double last_yaw, last_yaw_angle;
    bool new_command_flag = false;
    double yaw_eps = 1e-6;
    double max_speed, max_yaw;
    double rm, dir;
    bool B_button = false;
    int gait_counter = 0;
    std::string gait_type = "tetrapod";
    double eps = 0.05;
    int verbose_global = 0;

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
            p_L1 = init_phase_L1;
            p_L2 = init_phase_L2;
            p_L3 = init_phase_L3;
            p_R1 = init_phase_R1;
            p_R2 = init_phase_R2;
            p_R3 = init_phase_R3;
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

        // Cap max/min values
        if (speed > max_speed)
        {
            speed = max_speed;
        }
        if (yaw > max_yaw)
        {
            yaw = max_yaw;
        }
        if (yaw < -max_yaw)
        {
            yaw = -max_yaw;
        }

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

        // Detect if a new command has been sent
        new_command_flag = false;
        if (gait_mode != last_gait_mode || yaw != last_yaw || yaw_angle != last_yaw_angle)
        {
            new_command_flag = true;
        }
        
        rm = speed/yaw; // distance from body center to motion center
        cm = calcCm(yaw_angle, rm, base_height); // motion center

        double sp_L1, sp_L2, sp_L3, sp_R1, sp_R2, sp_R3; // support phase
        double tp_L1, tp_L2, tp_L3, tp_R1, tp_R2, tp_R3; // transfer phase

        std::tie(ci_L1, p_L1, sp_L1, tp_L1) = updateOneLeg(ci_L1, cw_L1, p_L1, init_phase_L1, init_L1, 0);
        std::tie(ci_L2, p_L2, sp_L2, tp_L2) = updateOneLeg(ci_L2, cw_L2, p_L2, init_phase_L2, init_L2, 0);
        std::tie(ci_L3, p_L3, sp_L3, tp_L3) = updateOneLeg(ci_L3, cw_L3, p_L3, init_phase_L3, init_L3, 0);
        std::tie(ci_R1, p_R1, sp_R1, tp_R1) = updateOneLeg(ci_R1, cw_R1, p_R1, init_phase_R1, init_R1, 0);
        std::tie(ci_R2, p_R2, sp_R2, tp_R2) = updateOneLeg(ci_R2, cw_R2, p_R2, init_phase_R2, init_R2, 0);
        std::tie(ci_R3, p_R3, sp_R3, tp_R3) = updateOneLeg(ci_R3, cw_R3, p_R3, init_phase_R3, init_R3, 0);
 
        sendL1PoseGoal(ci_L1);
        sendL2PoseGoal(ci_L2);
        sendL3PoseGoal(ci_L3);
        sendR1PoseGoal(ci_R1);
        sendR2PoseGoal(ci_R2);
        sendR3PoseGoal(ci_R3);

        last_gait_mode = gait_mode;
        last_yaw = yaw;
        last_yaw_angle = yaw_angle;

        if (verbose_global >= 2)
        {
            ROS_INFO("mode: %s, pL1: %.3f, pL2: %.3f, pL3: %.3f, pR1: %.3f, pR2: %.3f, pR3: %.3f",
                gait_mode.c_str(), p_L1, p_L2, p_L3, p_R1, p_R2, p_R3);
        }

        if (verbose_global >= 3)
        {
            ROS_INFO("p: %f, sp: %f, tp: %f, L1: (%f, %f, %f)", p_L1, sp_L1, tp_L1, ci_L1.x, ci_L1.y, ci_L1.z);
            ROS_INFO("p: %f, sp: %f, tp: %f, L2: (%f, %f, %f)", p_L2, sp_L2, tp_L2, ci_L2.x, ci_L2.y, ci_L2.z);
            ROS_INFO("p: %f, sp: %f, tp: %f, L3: (%f, %f, %f)", p_L3, sp_L3, tp_L3, ci_L3.x, ci_L3.y, ci_L3.z);
            ROS_INFO("p: %f, sp: %f, tp: %f, R1: (%f, %f, %f)", p_R1, sp_R1, tp_R1, ci_R1.x, ci_R1.y, ci_R1.z);
            ROS_INFO("p: %f, sp: %f, tp: %f, R2: (%f, %f, %f)", p_R2, sp_R2, tp_R2, ci_R2.x, ci_R2.y, ci_R2.z);
            ROS_INFO("p: %f, sp: %f, tp: %f, R3: (%f, %f, %f)", p_R3, sp_R3, tp_R3, ci_R3.x, ci_R3.y, ci_R3.z);
        }
    }

    std::tuple<geometry_msgs::Point, double, double, double> updateOneLeg(
        const geometry_msgs::Point& ci_old, const geometry_msgs::Point& cw,
        double& current_phase, const double& init_phase, bool& initialized,
        const int& verbose)
    {
        geometry_msgs::Point ci_new, AEP, PEP, cm_to_AEP, cm_to_PEP;
        double phi_w, rmw, theta, stride_time, diff_phase;
        double support_phase, transfer_phase;
        std::string stage;

        std::tie(phi_w, rmw) = calcPhiW(cm, cw);
        theta = calcTheta(rmw, dwi);
        stride_time = abs(2.0*theta/yaw);
        diff_phase = Tc/stride_time;
        std::tie(AEP, PEP) = calcAEPPEP(cm, rmw, phi_w, dir, theta, base_height);

        // Not moving
        if (gait_mode == "Default")
        {
            if (verbose >= 1)
            {
                ROS_INFO("Not moving.");
            }
            // TODO: Decide if leg should hold position while uninitialized or go to cw. Two spots like this
            ci_new = cw; // cw or ci_old
        }
        // New command sent
        else if (new_command_flag)
        {
            if (verbose >= 1)
            {
                ROS_INFO("New comand sent.");
            }
            ci_new = cw; // cw or ci_old
            // current_phase = init_phase;

            std::tie(ci_new, support_phase, transfer_phase, stage) = 
                movementRoutine(current_phase, phi_w, theta, rmw, AEP, PEP, ci_old);
        }
        // Moving, same command as before
        else if (!new_command_flag)
        {
            if (verbose >= 1)
            {
                ROS_INFO("Moving, same command as before.");
            }

            current_phase = updatePhase(current_phase, ci_old, AEP, PEP, theta, diff_phase, verbose);
            std::tie(ci_new, support_phase, transfer_phase, stage) = 
                movementRoutine(current_phase, phi_w, theta, rmw, AEP, PEP, ci_old);

            // Reset phase if reached 1.0
            if (current_phase >= 1.0)
            {
                current_phase = 0.0;
            }
        }
        // Do nothing
        else
        {
            if (verbose >= 1)
            {
                ROS_INFO("Do nothing (else case).");
            }
            ci_new = ci_old;
        }

        if (verbose >= 2)
        {
            if (gait_mode != "Default")
            {
                ROS_INFO("stage: %s, current phase: %.3f, support phase: %.3f, transfer phase: %.3f",
                    stage.c_str(), current_phase, support_phase, transfer_phase);
                ROS_INFO("diff_phase: %.3f, stride time: %.3f, stride height: %.3f", diff_phase, stride_time, stride_height);
                ROS_INFO("dir: %.3f, rmw: %.3f, theta: %f", dir, rmw, theta);
                ROS_INFO("AEP: (%.3f, %.3f, %.3f); PEP: (%.3f, %.3f, %.3f)", AEP.x, AEP.y, AEP.z, PEP.x, PEP.y, PEP.z);
                ROS_INFO("cm: (%.3f, %.3f);    ci: (%.3f, %.3f, %.3f)\n",
                    cm.x, cm.y, ci_new.x, ci_new.y, ci_new.z);
            }
            else
            {
                ROS_INFO("Default. ci: (%.3f, %.3f, %.3f)\n", ci_new.x, ci_new.y, ci_new.z);
            }
        }

        std::tuple<geometry_msgs::Point, double, double, double> result(ci_new, current_phase, support_phase, transfer_phase);
        
        return result; 
    }

    double updatePhase(
        const double& current_phase,
        const geometry_msgs::Point& ci_old,
        const geometry_msgs::Point& AEP, const geometry_msgs::Point& PEP,
        const double& theta, const double& diff_phase, const int& verbose)
    {
        double updated_current_phase;

        // Support Phase
        if (current_phase < duty_factor)
        {
            double phi_i, phi_AEP, phi_PEP, support_phase;
            phi_i = calcPhiI(cm, ci_old);
            std::tie(phi_i, phi_AEP, phi_PEP) = calcPhiAEPPEP(cm, AEP, PEP, phi_i);

            support_phase = abs(phi_i - phi_AEP)/(2.0 * theta);
            updated_current_phase = support_phase*duty_factor;

            if (verbose >= 2)
            {
                ROS_INFO("recalculated phase: %.3f, next phase: %.3f", updated_current_phase, updated_current_phase + diff_phase);
                ROS_INFO("before routine: sp: %.3f, phi_i-phi_AEP: %.3f, 2*theta: %.3f", support_phase, phi_i-phi_AEP, 2.0*theta);
            }

            // Increment current phase
            updated_current_phase += diff_phase;
            
            // Cap at duty_factor
            if (updated_current_phase >= duty_factor)
            {
                updated_current_phase = duty_factor;
            }
        }
        // Transfer Phase
        else
        {
            double d_PEP, PEP_to_AEP, transfer_phase;
            d_PEP = sqrt(pow(PEP.x - ci_old.x, 2.0) + pow(PEP.y - ci_old.y, 2.0));
            PEP_to_AEP = sqrt(pow(AEP.x - PEP.x, 2.0) + pow(AEP.y - PEP.y, 2.0));
            
            transfer_phase = d_PEP/PEP_to_AEP;
            updated_current_phase = transfer_phase*(1.0 - duty_factor) + duty_factor;

            if (verbose >= 2)
            {
                ROS_INFO("recalculated phase: %.3f, next phase: %.3f", updated_current_phase, updated_current_phase + diff_phase);
                ROS_INFO("before routine: tp: %.3f, d_PEP: %.3f, PEP_to_AEP: %.3f", transfer_phase, d_PEP, PEP_to_AEP);
            }

            // Increment current phase
            updated_current_phase += diff_phase;

            // Cap at 1.0
            if (updated_current_phase >= 1.0)
            {
                updated_current_phase = 1.0;
            }
        }

        return updated_current_phase;
    }

    std::tuple<geometry_msgs::Point, double, double, std::string> movementRoutine(
        const double& current_phase, const double& phi_w,
        const double& theta, const double& rmw,
        const geometry_msgs::Point& AEP, const geometry_msgs::Point& PEP,
        const geometry_msgs::Point& ci_old)
    {
        geometry_msgs::Point ci_new;
        double support_phase, transfer_phase;
        std::string stage;

        // Support Phase
        if (current_phase < duty_factor)
        {
            stage = "Support Phase";
            support_phase = current_phase/duty_factor;
            transfer_phase = 0.0;

            double angle = phi_w + dir*(2.0*support_phase - 1.0)*theta;
            ci_new.x = rmw*cos(angle) + cm.x;
            ci_new.y = rmw*sin(angle) + cm.y;
            ci_new.z = -base_height;
        }
        // Transfer Phase
        else
        {
            stage = "Transfer Phase";
            support_phase = 0.0;
            transfer_phase = (current_phase - duty_factor)/(1.0 - duty_factor);

            ci_new.x = AEP.x*(transfer_phase) + PEP.x*(1.0 - transfer_phase);
            ci_new.y = AEP.y*(transfer_phase) + PEP.y*(1.0 - transfer_phase);

            // Lifting
            if (transfer_phase < lowering_point)
            {
                ci_new.z = LPF1(-stride_height, ci_old.z, 0.1);
            }
            // Lowering
            else
            {
                double lowering_phase = (transfer_phase - lowering_point)/(1.0 - lowering_point);
                ci_new.z = -base_height + stride_height*(1.0 - lowering_phase);
            }
        }

        std::tuple<geometry_msgs::Point, double, double, std::string> result(
            ci_new, support_phase, transfer_phase, stage);
        
        return result;
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

    void sendL1PoseGoal(const geometry_msgs::Point& ci)
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
        client_L1.sendGoal(poseAction,
            boost::bind(&GaitController::publishResult, this, _1, _2),
            boost::bind(&GaitController::activeCB, this),
            boost::bind(&GaitController::publishFeedback, this, _1));
    }

    void sendL2PoseGoal(const geometry_msgs::Point& ci)
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
        client_L2.sendGoal(poseAction,
            boost::bind(&GaitController::publishResult, this, _1, _2),
            boost::bind(&GaitController::activeCB, this),
            boost::bind(&GaitController::publishFeedback, this, _1));
    }

    void sendL3PoseGoal(const geometry_msgs::Point& ci)
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
        client_L3.sendGoal(poseAction,
            boost::bind(&GaitController::publishResult, this, _1, _2),
            boost::bind(&GaitController::activeCB, this),
            boost::bind(&GaitController::publishFeedback, this, _1));
    }

    void sendR1PoseGoal(const geometry_msgs::Point& ci)
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
        client_R1.sendGoal(poseAction,
            boost::bind(&GaitController::publishResult, this, _1, _2),
            boost::bind(&GaitController::activeCB, this),
            boost::bind(&GaitController::publishFeedback, this, _1));
    }

    void sendR2PoseGoal(const geometry_msgs::Point& ci)
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
        client_R2.sendGoal(poseAction,
            boost::bind(&GaitController::publishResult, this, _1, _2),
            boost::bind(&GaitController::activeCB, this),
            boost::bind(&GaitController::publishFeedback, this, _1));
    }

    void sendR3PoseGoal(const geometry_msgs::Point& ci)
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
        client_R3.sendGoal(poseAction,
            boost::bind(&GaitController::publishResult, this, _1, _2),
            boost::bind(&GaitController::activeCB, this),
            boost::bind(&GaitController::publishFeedback, this, _1));
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