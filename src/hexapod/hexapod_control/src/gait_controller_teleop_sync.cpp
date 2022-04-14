#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/SetPoseAction.h"
#include "hexapod_control/Pose.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "math.h"
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

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
        hip_angle_L1 = hip_angle_L1*M_PI/180.0; // convert to radians
        hip_angle_L2 = hip_angle_L2*M_PI/180.0; // convert to radians
        hip_angle_L3 = hip_angle_L3*M_PI/180.0; // convert to radians
        hip_angle_R1 = hip_angle_R1*M_PI/180.0; // convert to radians
        hip_angle_R2 = hip_angle_R2*M_PI/180.0; // convert to radians
        hip_angle_R3 = hip_angle_R3*M_PI/180.0; // convert to radians

        start = ros::Time::now().toSec();
        publish_rate = 0.02; // 50Hz
        timer = node.createTimer(ros::Duration(publish_rate), boost::bind(&GaitController::updateAllLegs, this));

        std::tie(dcw, dwi) = calcStepRadius(base_height); // TODO: make base_height variable
        cw_L1 = calcCw(base_height, base_radius, hip_angle_L1);
        cw_L2 = calcCw(base_height, base_radius, hip_angle_L2);
        cw_L3 = calcCw(base_height, base_radius, hip_angle_L3);
        cw_R1 = calcCw(base_height, base_radius, hip_angle_R1);
        cw_R2 = calcCw(base_height, base_radius, hip_angle_R2);
        cw_R3 = calcCw(base_height, base_radius, hip_angle_R3);

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

        speed = 0.0;
        yaw = yaw_eps;
        yaw_angle = 0.0;
        lowering_point = 0.9;
        stability_factor = 0.05;
        verbose = 2;

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
    double start, t, elapsed, last_elapsed, Tc;
    double femur_length, tibia_length, coxa_length, base_radius, base_height;
    double hip_angle_L1, hip_angle_L2, hip_angle_L3, hip_angle_R1, hip_angle_R2, hip_angle_R3;
    double init_phase_L1, init_phase_L2, init_phase_L3, init_phase_R1, init_phase_R2, init_phase_R3;
    double p_L1, p_L2, p_L3, p_R1, p_R2, p_R3;
    std::string gait_mode = "Default";
    std::string last_gait_mode = "Default";
    double speed, yaw, yaw_angle;
    double max_speed, max_yaw;
    double rm, dir;
	double stride_time, stride_height, duty_factor;
    geometry_msgs::Point ci_L1, ci_L2, ci_L3, ci_R1, ci_R2, ci_R3;
    geometry_msgs::Point cw_L1, cw_L2, cw_L3, cw_R1, cw_R2, cw_R3;
    geometry_msgs::Point cm;
    double lowering_point;
    double dwi, dcw;
    bool new_command = false;
    bool init_L1 = false;
    bool init_L2 = false;
    bool init_L3 = false;
    bool init_R1 = false;
    bool init_R2 = false;
    bool init_R3 = false;
    bool B_button = false;
    int gait_counter = 0;
    std::string gait_type = "ripple";
    double stability_factor;
    double eps = 0.05;
    double yaw_eps = 1e-6;
    double publish_rate;
    ros::Timer timer;
    boost::mutex publish_mutex;
    int verbose = 0;

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
                    gait_type = "ripple";
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
        double deadzone = 0.0; // no deadzone necessary for SN30pro+
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

        // TODO: Should mapRange() be nonlinear so that small deflections move the robot more?

        dir = 1.0; // default

        // Map joystick data to speed, yaw, yaw_angle
        // Strafe (Lx and Ly only)
        if ((linearX != 0.0 || linearY != 0.0) && angular == 0.0)
        {  
            gait_mode = "Strafe";
            speed = mapRange(abs(linearX) + abs(linearY), 0.0, 1.0, 0.0, max_speed);
            yaw_angle = atan2(linearY, linearX);
            yaw = yaw_eps;
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
        // Steer (Ly and Rx)
        else if (linearY != 0.0 && angular != 0.0)
        {
            // TODO: make this a separate mode controlled by a button? imagine pressing Ly and then suddenly pressing Rx
            gait_mode = "Steer";
            speed = mapRange(abs(linearY), 0.0, 1.0, 0.0, max_speed);
            yaw = mapRange(angular, -1.0, 1.0, -max_yaw, max_yaw);
            yaw_angle = mapRange(abs(angular), 0.0, 1.0, 0.0, M_PI/3.0);
            if (angular > 0.0)
            {
                yaw_angle = M_PI - yaw_angle;
            }
            dir = (yaw > 0.0) ? 1.0 : -1.0;
        }
        // Default
        else
        {
            gait_mode = "Default";
            speed = 0.0;
            yaw = yaw_eps;
            yaw_angle = 0.0;
            dir = 1.0;
        }

        // Cap maximum values
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

        if (verbose >= 2)
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
        if (gait_mode != "Default")
        {
            Tc = elapsed - last_elapsed; // control interval
        }
        else
        {
            Tc = 0.0; // don't allow time to pass if not moving
        }
        last_elapsed = elapsed;
        
        rm = speed/yaw; // distance from body center to motion center
        cm = calcCm(yaw_angle, rm, base_height); // motion center

        double sp_L1, sp_L2, sp_L3, sp_R1, sp_R2, sp_R3; // support phase
        double tp_L1, tp_L2, tp_L3, tp_R1, tp_R2, tp_R3; // transfer phase

        // TODO: get rid of new command and do this every time regardless of change in command
        if (gait_mode != "Default" && last_gait_mode == "Default")
        {
            new_command = true;
        }
        else
        {
            new_command = false;
        }

        std::tie(ci_L1, p_L1, sp_L1, tp_L1) = updateOneLeg(ci_L1, cw_L1, p_L1, init_L1, 2);
        std::tie(ci_L2, p_L2, sp_L2, tp_L2) = updateOneLeg(ci_L2, cw_L2, p_L2, init_L2, 0);
        std::tie(ci_L3, p_L3, sp_L3, tp_L3) = updateOneLeg(ci_L3, cw_L3, p_L3, init_L3, 0);
        std::tie(ci_R1, p_R1, sp_R1, tp_R1) = updateOneLeg(ci_R1, cw_R1, p_R1, init_R1, 0);
        std::tie(ci_R2, p_R2, sp_R2, tp_R2) = updateOneLeg(ci_R2, cw_R2, p_R2, init_R2, 0);
        std::tie(ci_R3, p_R3, sp_R3, tp_R3) = updateOneLeg(ci_R3, cw_R3, p_R3, init_R3, 0);

        last_gait_mode = gait_mode;

        sendL1PoseGoal(ci_L1);
        sendL2PoseGoal(ci_L2);
        sendL3PoseGoal(ci_L3);
        sendR1PoseGoal(ci_R1);
        sendR2PoseGoal(ci_R2);
        sendR3PoseGoal(ci_R3);

        if (verbose >= 2)
        {
            ROS_INFO("mode: %s, pL1: %.3f, pL2: %.3f, pL3: %.3f, pR1: %.3f, pR2: %.3f, pR3: %.3f",
                gait_mode.c_str(), p_L1, p_L2, p_L3, p_R1, p_R2, p_R3);
        }

        if (verbose >= 3)
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
        double& current_phase, bool& initialized,
        const int& verbose)
    {
        geometry_msgs::Point ci_new, AEP, PEP, cm_to_AEP, cm_to_PEP;
        double phi_w, rmw, theta;
        double support_phase, transfer_phase;
        std::string stage;

        std::tie(phi_w, rmw) = calcPhiW(cm, cw);
        theta = calcTheta(rmw, dwi);
        stride_time = abs(2.0*theta/yaw);
        std::tie(AEP, PEP) = calcAEPPEP(rmw, phi_w, dir, theta, base_height);

        // Increment current phase
        current_phase += Tc/stride_time;
        if (current_phase > 1.0)
        {
            current_phase -= 1.0;
        }

        // Not moving and not initialized
        if (!initialized && gait_mode == "Default")
        {
            ci_new = cw;
            if (verbose >= 1)
            {
                ROS_INFO("Not initialized.");
            }
        }
        // About to move and not initialized
        else if (!initialized && gait_mode != "Default")
        {
            initialized = true;
            if (verbose >= 1)
            {
                ROS_INFO("Initialized.");
            }
            // First update ci based on current phase in case coming from neutral cw state
            if (current_phase < duty_factor)
            {
                std::tie(ci_new, support_phase) = 
                    support_phase_routine(current_phase, phi_w, theta, rmw);
            }
            else
            {
                std::tie(ci_new, transfer_phase) = 
                    transfer_phase_routine(current_phase, AEP, PEP, ci_old);
            }
        }
        // Moving and initialized
        else if (initialized && gait_mode != "Default")
        {
            // Support Phase
            if (current_phase < duty_factor)
            {
                // Update current phase based on current location if command changed
                if (new_command)
                {
                    double phi_i = calcPhiI(cm, ci_old);
                    double phi_AEP, phi_PEP;
                    std::tie(phi_AEP, phi_PEP) = calcPhiAEPPEP(cm, AEP, PEP);

                    // Fix for L2 angle pi -> -pi crossover
                    if (phi_PEP > M_PI_2 && phi_AEP < -M_PI_2)
                    {
                        phi_AEP += 2.0*M_PI;
                        phi_i += (phi_i < 0.0) ? 2.0*M_PI : 0.0;
                    }
                    else if (phi_AEP > M_PI_2 && phi_PEP < -M_PI_2)
                    {
                        phi_PEP += 2.0*M_PI;
                        phi_i += (phi_i < 0.0) ? 2.0*M_PI : 0.0;
                    }

                    support_phase = abs(phi_i - phi_AEP)/(2.0 * theta);
                    transfer_phase = 0.0;
                    current_phase = support_phase*duty_factor;

                    if (verbose >= 2)
                    {
                        ROS_INFO("phi_i: %.3f, phi_AEP: %.3f, phi_PEP: %.3f", phi_i, phi_AEP, phi_PEP);
                        ROS_INFO("phase updated to: %.3f, sp: %.3f", current_phase, support_phase);
                    }
                }

                stage = "Support Phase";
                std::tie(ci_new, support_phase) = 
                    support_phase_routine(current_phase, phi_w, theta, rmw);
                transfer_phase = 0.0;
            }
            // Transfer Phase
            else
            {
                // Update current phase based on current location if command changed
                if (new_command)
                {
                    double alpha = atan2(AEP.y - ci_old.y, AEP.x - ci_old.x);
                    double d_AEP = sqrt(pow(AEP.x - ci_old.x, 2) + pow(AEP.y - ci_old.y, 2));
                    double PEP_to_AEP = sqrt(pow(AEP.x - PEP.x, 2) + pow(AEP.y - PEP.y, 2));
                    double d_PEP = PEP_to_AEP - d_AEP;
                    double transfer_distance = PEP_to_AEP + stride_height;
                    
                    transfer_phase = d_PEP/transfer_distance;
                    support_phase = 0.0;
                    current_phase = transfer_phase*(1.0 - duty_factor) + duty_factor;

                    if (verbose >= 2)
                    {
                        ROS_INFO("d_PEP: %.3f, transfer_distance: %.3f", d_PEP, transfer_distance);
                        ROS_INFO("phase updated to: %.3f, tp: %.3f", current_phase, transfer_phase);
                    }
                };

                stage = "Transfer Phase";
                std::tie(ci_new, transfer_phase) = 
                    transfer_phase_routine(current_phase, AEP, PEP, ci_old);
                support_phase = 0.0;
            }
        }
        else // Do nothing
        {
            ci_new = ci_old;
        }

        if (verbose >= 2)
        {
            if (gait_mode != "Default")
            {
                ROS_INFO("stage: %s, current phase: %.3f, support phase: %.3f, transfer phase: %.3f",
                    stage.c_str(), current_phase, support_phase, transfer_phase);
                ROS_INFO("stride time: %.3f, stride height: %.3f", stride_time, stride_height);
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

    std::tuple<geometry_msgs::Point, double> support_phase_routine(
        const double& current_phase, const double& phi_w,
        const double& theta, const double& rmw
    )
    {
        geometry_msgs::Point ci_new;
        double support_phase, angle;

        support_phase = current_phase/duty_factor;
        angle = phi_w + dir*(2.0*support_phase - 1.0)*theta; // TODO: don't make this dependent on dir
        ci_new.x = rmw*cos(angle) + cm.x;
        ci_new.y = rmw*sin(angle) + cm.y;
        ci_new.z = -base_height;

        std::tuple<geometry_msgs::Point, double> result(ci_new, support_phase);
        
        return result;
    }

    std::tuple<geometry_msgs::Point, double> transfer_phase_routine(
        const double& current_phase,
        const geometry_msgs::Point& AEP, const geometry_msgs::Point& PEP,
        const geometry_msgs::Point& ci_old
    )
    {
        geometry_msgs::Point ci_new;
        double transfer_phase = (current_phase - duty_factor)/(1.0 - duty_factor);

        // Lifting
        if (transfer_phase < lowering_point)
        {
            double lifting_phase = transfer_phase/lowering_point;
            // if (dir < 0.0)
            // {
            //     lifting_phase = 1.0 - lifting_phase;
            // }
            ci_new.x = AEP.x*(lifting_phase) + PEP.x*(1.0 - lifting_phase);
            ci_new.y = AEP.y*(lifting_phase) + PEP.y*(1.0 - lifting_phase);
            ci_new.z = LPF1(-stride_height, ci_old.z, 0.1);
        }
        // Lowering
        else
        {
            ci_new.x = AEP.x;
            ci_new.y = AEP.y;
            double lowering_phase = (transfer_phase - lowering_point)/(1.0 - lowering_point);
            // if (dir < 0.0)
            // {
            //     lowering_phase = 1.0 - lowering_phase;
            // }
            ci_new.z = -base_height + stride_height*(1.0 - lowering_phase);

            if(verbose >= 2)
            {
                ROS_INFO("Lowering");
            }
        }

        std::tuple<geometry_msgs::Point, double> result(ci_new, transfer_phase);
        
        return result;
    }

    geometry_msgs::Point calcCm(
        const double& yaw_angle, const double& rm, const double& base_height)
    {
        // phi: current angle of the line connecting motion center and body center
        // double phi = atan2(ci.y, ci.x);
        double phi = yaw_angle - M_PI_2;

        geometry_msgs::Point cm;
        cm.x = rm*cos(phi);
        cm.y = rm*sin(phi);
        cm.z = -base_height;

        return cm;
    }

    geometry_msgs::Point calcCw(
        const double& base_height, const double& base_radius,
        const double& hip_angle)
    {
        double foot_radius = base_radius + dcw;
        geometry_msgs::Point cw;
        cw.x = foot_radius*cos(hip_angle);
        cw.y = foot_radius*sin(hip_angle);
        cw.z = -base_height;

        return cw;
    }

    double calcPhiI(
        const geometry_msgs::Point& cm, const geometry_msgs::Point& ci)
    {
        // phi_i: current angle of the line connecting motion center and foot position
        geometry_msgs::Point cm_to_ci;
        cm_to_ci.x = ci.x - cm.x;
        cm_to_ci.y = ci.y - cm.y;

        return atan2(cm_to_ci.y, cm_to_ci.x);
    }

    std::tuple<double, double> calcPhiW(
        const geometry_msgs::Point& cm, const geometry_msgs::Point& cw)
    {
        geometry_msgs::Point cm_to_cw;
        cm_to_cw.x = cw.x - cm.x;
        cm_to_cw.y = cw.y - cm.y;
        double phi_w = atan2(cm_to_cw.y, cm_to_cw.x);
        double rmw = sqrt(pow(cm_to_cw.x, 2) + pow(cm_to_cw.y, 2));
        std::tuple<double, double> result(phi_w, rmw);
        
        return result;
    }

    double calcTheta(const double& rmw, const double& dwi)
    {
        // theta: angle from rotation center to edge of foot WS wrt cm
        // https://mathworld.wolfram.com/Circle-CircleIntersection.html
        double x = (pow(rmw, 2) - pow(dwi, 2) + pow(rmw, 2))/(2.0*rmw);
        double y = sqrt(pow(rmw, 2) - pow(x, 2));
        
        return asin(y/rmw); // y = rmw*sin(theta)
    }

    std::tuple<geometry_msgs::Point, geometry_msgs::Point> calcAEPPEP(
        const double& rmw, const double& phi_w, const double& dir,
        const double& theta, const double& base_height)
    {
        geometry_msgs::Point AEP;
        AEP.x = rmw*cos(phi_w - dir*theta) + cm.x;
        AEP.y = rmw*sin(phi_w - dir*theta) + cm.y;
        AEP.z = -base_height;

        geometry_msgs::Point PEP;
        PEP.x = rmw*cos(phi_w + dir*theta) + cm.x;
        PEP.y = rmw*sin(phi_w + dir*theta) + cm.y;
        PEP.z = -base_height;

        std::tuple<geometry_msgs::Point, geometry_msgs::Point> result(AEP, PEP);

        return result;
    }

    std::tuple<double, double> calcPhiAEPPEP(
        const geometry_msgs::Point& cm, const geometry_msgs::Point& AEP,
        const geometry_msgs::Point& PEP)
    {
        geometry_msgs::Point cm_to_AEP, cm_to_PEP;
        cm_to_AEP.x = AEP.x - cm.x;
        cm_to_AEP.y = AEP.y - cm.y;
        cm_to_PEP.x = PEP.x - cm.x;
        cm_to_PEP.y = PEP.y - cm.y;
        
        double phi_AEP = atan2(cm_to_AEP.y, cm_to_AEP.x);
        double phi_PEP = atan2(cm_to_PEP.y, cm_to_PEP.x);
        std::tuple<double, double> result(phi_AEP, phi_PEP);

        return result;
    }


    std::tuple<double, double> calcStepRadius(const double& base_height)
    {
        double beta;
        beta = asin(base_height/(femur_length + tibia_length)); // full stretch leg angle
        dwi = (femur_length + tibia_length)*cos(beta)/2.0; // workspace radius
        dcw = coxa_length + dwi; // radius from hip to workspace center

        // Limit dwi to use smaller workspace radius without changing cw workspace center
        dwi *= 0.5;

        std::tuple<double, double> result(dcw, dwi);

        return result;
    }

    double LPF1(const double& goal, const double& z_prev, const double& rate)
    {
        // 1st order low-pass filter
        // https://dsp.stackexchange.com/questions/60277/is-the-typical-implementation-of-low-pass-filter-in-c-code-actually-not-a-typica
        // rate must be between 0.0 and 1.0. as rate approaches 0, the cutoff of the filter decreases
        double z_next = (1 - rate) * z_prev + rate * goal; // "simplistic" low pass filter

        return z_next;
    }

    double mapRange(const double& inValue,
                    const double& minInRange, const double& maxInRange,
                    const double& minOutRange, const double& maxOutRange)
    {
        double x = (inValue - minInRange) / (maxInRange - minInRange);
        double result = minOutRange + (maxOutRange - minOutRange) * x;

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