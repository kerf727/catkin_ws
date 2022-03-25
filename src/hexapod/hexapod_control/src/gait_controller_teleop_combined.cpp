#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/SetPoseAction.h"
#include "hexapod_control/Pose.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "math.h"

class GaitController
{
public:
    GaitController() :
        L1_client("leg_L1_pose_action", true)//,
        // L2_client("leg_L2_pose_action", true),
        // L3_client("leg_L3_pose_action", true),
        // R1_client("leg_R1_pose_action", true),
        // R2_client("leg_R2_pose_action", true),
        // R3_client("leg_R3_pose_action", true)
    {
        this->node = node;

        ROS_INFO("Subscribing to Teleop...");
		this->twistSubscriber = node.subscribe("/hexapod/teleop/twist", 10, &GaitController::twistCB, this);
		this->buttonSubscriber = node.subscribe("/hexapod/teleop/button", 10, &GaitController::buttonCB, this);
		
        ROS_INFO("Waiting for Pose State Server...");
        this->L1_client.waitForServer(ros::Duration(30));
        // this->L2_client.waitForServer(ros::Duration(30));
        // this->L3_client.waitForServer(ros::Duration(30));
        // this->R1_client.waitForServer(ros::Duration(30));
        // this->R2_client.waitForServer(ros::Duration(30));
        // this->R3_client.waitForServer(ros::Duration(30));

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

        ux.x = 1.0;
        ux.y = 0.0;

        std::tie(dcw, dwi) = calcStepRadius(base_height); // TODO: make base_height variable

        node.getParam("/hexapod/gait/" + gait_type + "/max_speed", max_speed);
        node.getParam("/hexapod/gait/" + gait_type + "/max_yaw", max_yaw);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/L1", phase_L1);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/L2", phase_L2);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/L3", phase_L3);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/R1", phase_R1);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/R2", phase_R2);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/R3", phase_R3);
        node.getParam("/hexapod/gait/" + gait_type + "/duty_factor", duty_factor);
        node.getParam("/hexapod/gait/" + gait_type + "/stride_height", stride_height);

        // TODO: repeat for 6 legs
        for (int i = 0; i < num_legs; i++)
        {
            leg_name = "L1";
            
            cw = calcCw(base_height, base_radius, hip_angle_L1);
            ci = cw; // set ci to be workspace center to start

            if (phase_L1 < 1.0 - duty_factor)
            {
                stage = "Transfer Phase";
                support_phase = 0.0;
                transfer_phase = phase_L1/(1.0 - duty_factor);
            }
            else
            {
                stage = "Support Phase";
                support_phase = (duty_factor - phase_L1)/duty_factor;
                transfer_phase = 0.0;
            }
        }

        ROS_INFO("Gait controller ready.");
        ROS_INFO("initial_phase: %f, stage: %s, support_phase: %f, transfer_phase: %f", initial_phase, stage.c_str(), support_phase, transfer_phase);
        ROS_INFO("initial ci: (%f, %f, %f); dwi: %f, dcw: %f", ci.x, ci.y, ci.z, dwi, dcw);
    }

    ~GaitController()
    {
        this->node.shutdown();
    }

private:
    std::string leg_name;
    double num_legs = 1;
    ros::NodeHandle node;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> L1_client;
	// actionlib::SimpleActionClient<hexapod_control::SetPoseAction> L2_client;
	// actionlib::SimpleActionClient<hexapod_control::SetPoseAction> L3_client;
	// actionlib::SimpleActionClient<hexapod_control::SetPoseAction> R1_client;
	// actionlib::SimpleActionClient<hexapod_control::SetPoseAction> R2_client;
	// actionlib::SimpleActionClient<hexapod_control::SetPoseAction> R3_client;
    ros::Subscriber twistSubscriber;
    ros::Subscriber buttonSubscriber;
    ros::ServiceClient linkStateClient;
    hexapod_control::Pose current_pose;
    double start, t, elapsed, last_elapsed, Tc;
    double femur_length, tibia_length, coxa_length, base_radius, base_height;
    double hip_angle_L1, hip_angle_L2, hip_angle_L3, hip_angle_R1, hip_angle_R2, hip_angle_R3;
	double initial_phase;
    double phase_L1, phase_L2, phase_L3, phase_R1, phase_R2, phase_R3;
    double speed, yaw, yaw_angle;
    double max_speed, max_yaw;
    std::string gait_mode;
	double stride_time, stride_height, duty_factor;
    geometry_msgs::Point ci, cm, cw, hip, ux;
    double support_phase, transfer_phase;
    double dwi, dcw;
    double last_transfer_phase, phase_diff, lowering_rate;
    bool initialized = false;
    bool B_button = false;
    int gait_counter = 0;
    std::string gait_type = "ripple";
    std::string stage;
    double eps = 0.05;
    double yaw_eps = 1e-6; // 1e-6

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
        node.getParam("/hexapod/gait/" + gait_type + "/phase/L1", phase_L1);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/L2", phase_L2);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/L3", phase_L3);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/R1", phase_R1);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/R2", phase_R2);
        node.getParam("/hexapod/gait/" + gait_type + "/phase/R3", phase_R3);
        node.getParam("/hexapod/gait/" + gait_type + "/duty_factor", duty_factor);
        node.getParam("/hexapod/gait/" + gait_type + "/stride_height", stride_height);
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

        // Map joystick data to speed, yaw, yaw_angle
        // Strafe (Lx and Ly only)
        if ((linearX != 0.0 || linearY != 0.0) && angular == 0.0)
        {  
            gait_mode = "Strafe";
            // TODO: fix max of map input. can be larger than 1.0 when both contribute
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
        }
        // Default
        else
        {
            gait_mode = "Default";
            speed = 0.0;
            yaw = yaw_eps;
            yaw_angle = 0.0;
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

        ROS_INFO("--------------------------------------------------------------------------------");
        ROS_INFO("mode: %s, speed: %f, yaw: %f, yaw_angle: %f\n",
            gait_mode.c_str(), speed, yaw, yaw_angle);

        updateLegs(speed, yaw, yaw_angle, gait_mode);
    }

    void updateLegs(
        const double& speed, const double& yaw, const double& yaw_angle, const std::string& gait_mode)
    {
        geometry_msgs::Point AEP, PEP, cm_to_AEP, cm_to_PEP;
        double alpha, d_AEP, d_PEP, PEP_to_AEP, phi_plus, delta_phi;
        double phi_i, phi_w, phi_AEP, phi_PEP;
        double rm, rmw, theta, dir;
        double transfer_time, tG_plus, delta_tG, k_plus, delta_x, delta_y;
        double current_height, transfer_distance;
        
        elapsed = ros::Time::now().toSec() - start;

        Tc = elapsed - last_elapsed; // control interval
        last_elapsed = elapsed;
        delta_phi = yaw*Tc;

        try
        {
            rm = speed/yaw; // distance from body center to motion center
        }
        catch(const std::exception& e)
        {
            rm = 0.0;
            ROS_WARN("Divide by zero error. Yaw cannot be zero. %s", e.what());
        }

        dir = (yaw > 0) ? 1.0 : -1.0;

        cm = calcCm(yaw_angle, rm, base_height); // motion center

        // TODO: repeat for 6 legs
        for (int i = 0; i < num_legs; i++)
        {
            leg_name = "L1";

            cw = calcCw(base_height, base_radius, hip_angle_L1); // center of foot workspace

            phi_i = calcPhiI(cm, ci);
            std::tie(phi_w, rmw) = calcPhiW(cm, cw);

            theta = calcTheta(rmw, dwi);
            stride_time = abs(2.0*theta/yaw); // not infinity because yaw is nonzero

            std::tie(AEP, PEP) = calcAEPPEP(rmw, phi_w, dir, theta, base_height);
            std::tie(phi_AEP, phi_PEP) = calcPhiAEPPEP(cm, AEP, PEP);

            if (!initialized && gait_mode != "Default")
            {
                ci = (stage == "Support Phase") ? AEP : PEP; // place at initial_phase position
                initialized = true;
            }

            if (stage == "Support Phase" && gait_mode != "Default" && initialized)
            {
                support_phase = abs(phi_i - phi_AEP)/(2.0 * theta);
                if (support_phase >= 1.0)
                {
                    stage = "Transfer Phase";
                    support_phase = 0.0;
                    transfer_phase = 0.0;
                    phi_i = phi_PEP;
                    phi_plus = phi_PEP - phi_AEP;
                    ci = PEP;
                }
                else
                {
                    ci.x = rmw*cos(phi_i + delta_phi) + cm.x;
                    ci.y = rmw*sin(phi_i + delta_phi) + cm.y;
                    ci.z = -base_height;
                }
            }
            else if (stage == "Transfer Phase" && gait_mode != "Default" && initialized)
            {
                alpha = atan2(AEP.y - ci.y, AEP.x - ci.x);
                d_AEP = sqrt(pow(AEP.x - ci.x, 2) + pow(AEP.y - ci.y, 2));
                PEP_to_AEP = sqrt(pow(AEP.x - PEP.x, 2) + pow(AEP.y - PEP.y, 2));
                d_PEP = PEP_to_AEP - d_AEP;
                phi_plus = dir*(phi_i - phi_AEP);
                current_height = ci.z + base_height;
                transfer_distance = PEP_to_AEP + stride_height;

                if (phi_plus > 0.0) // lifting
                {
                    // transfer_phase = abs((phi_i - phi_PEP)/(2.0 * theta));
                    transfer_phase = d_PEP/transfer_distance;
                    phase_diff = transfer_phase - last_transfer_phase;
                    last_transfer_phase = transfer_phase;
                    lowering_rate = stride_height/(1.0 - transfer_phase);

                    transfer_time = stride_time*(1.0 - duty_factor);
                    tG_plus = (1.0 - transfer_phase)*transfer_time;
                    delta_tG = abs(delta_phi)/(phi_plus/tG_plus); 
                    k_plus = round(tG_plus/delta_tG);

                    ci.x += (d_AEP + stride_height)*cos(alpha)/k_plus;
                    ci.y += (d_AEP + stride_height)*sin(alpha)/k_plus;
                    ci.z = LPF1(-stride_height, ci.z, 0.1);
                }
                else // lowering
                {
                    transfer_phase += phase_diff;
                    if (transfer_phase >= 1.0)
                    {
                        stage = "Support Phase";
                        support_phase = 0.0;
                        transfer_phase = 0.0;
                        phi_i = phi_AEP;
                        ci = AEP;
                    }

                    ci.x = AEP.x;
                    ci.y = AEP.y;
                    ci.z -= lowering_rate*phase_diff;
                    if (ci.z < -base_height)
                    {
                        ci.z = -base_height;
                    }

                    ROS_INFO("Lowering");
                }
            }
            else // Do nothing
            {
                support_phase = 0.0;
                transfer_phase = 0.0;
            }
        }

        if (gait_mode != "Default")
        {
            ROS_INFO("stage: %s, support phase: %f, transfer phase: %f", stage.c_str(), support_phase, transfer_phase);
            ROS_INFO("stride time: %f, stride height: %f", stride_time, stride_height);
            ROS_INFO("rmw: %f, theta: %f, delta_phi: %f", rmw, theta, delta_phi);
            ROS_INFO("AEP: (%f, %f, %f); PEP: (%f, %f, %f)", AEP.x, AEP.y, AEP.z, PEP.x, PEP.y, PEP.z);
            ROS_INFO("phi_i: %f, phi_AEP: %f, phi_PEP = %f", phi_i, phi_AEP, phi_PEP);
            if (stage == "Transfer Phase")
            {
                ROS_INFO("d_AEP: %f, PEP_to_AEP: %f, alpha: %f, k_plus: %f", d_AEP, PEP_to_AEP, alpha, k_plus);
                ROS_INFO("transfer time: %f, phi_plus: %f, tG_plus: %f, delta_tG: %f",
                    transfer_time, phi_plus, tG_plus, delta_tG);
                ROS_INFO("lowering_rate: %f, phase_diff: %f", lowering_rate, phase_diff);
            }
            ROS_INFO("cm: (%f, %f);   ci: (%f, %f, %f)\n",
                cm.x, cm.y, ci.x, ci.y, ci.z);
        }

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
        poseAction.goal = target_pose;
        poseAction.eps = eps;
        this->L1_client.sendGoal(poseAction,
            boost::bind(&GaitController::publishResult, this, _1, _2),
            boost::bind(&GaitController::activeCB, this),
            boost::bind(&GaitController::publishFeedback, this, _1));
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
        dwi *= 0.7;

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