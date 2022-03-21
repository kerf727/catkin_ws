#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/SetPoseAction.h"
#include "hexapod_control/GaitAction.h"
#include "hexapod_control/Pose.h"
#include "hexapod_control/SolveFKPose.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "gazebo_msgs/GetLinkState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"
#include "math.h"

std::string leg_name = "L1";

class SetTrajectoryAction
{
public:
    SetTrajectoryAction() :
        client("leg_" + leg_name + "_pose_action", true)
    {
        this->node = node;

		ROS_INFO("Subscribing to Gait Controller and Teleop...");
		this->gaitModeSubscriber = node.subscribe("/hexapod/gait/gait_mode", 10, &SetTrajectoryAction::gaitModeCB, this);
		this->strideHeightSubscriber = node.subscribe("/hexapod/gait/stride_height", 10, &SetTrajectoryAction::strideHeightCB, this);
		this->dutyFactorSubscriber = node.subscribe("/hexapod/gait/duty_factor", 10, &SetTrajectoryAction::dutyFactorCB, this);
		this->phaseSubscriber = node.subscribe("/hexapod/gait/phase_" + leg_name, 10, &SetTrajectoryAction::phaseCB, this);
		this->commandSubscriber = node.subscribe("/hexapod/gait/command", 10, &SetTrajectoryAction::commandCB, this);

        ROS_INFO("Waiting for Pose State Server...");
        this->client.waitForServer(ros::Duration(30));

        ROS_INFO("Initializing constants...");
        node.getParam("/hexapod/geometry/coxa_length", coxa_length);
        node.getParam("/hexapod/geometry/femur_length", femur_length);
        node.getParam("/hexapod/geometry/tibia_length", tibia_length);
        node.getParam("/hexapod/geometry/base/radius", base_radius);
        node.getParam("/hexapod/geometry/base/height", base_height);
        node.getParam("/hexapod/geometry/leg_" + leg_name + "/hip_angle", hip_angle);
        hip_angle = hip_angle*M_PI/180.0; // convert to radians

        start = ros::Time::now().toSec();

        std::tie(dwi, dcw) = calcStepRadius(base_height); // TODO: make base_height variable
        foot_radius = base_radius + dcw;

        ci = initialFootPos(base_height); // initialize with current foot position
        ROS_INFO("initial ci: (%f, %f, %f); dwi: %f, dcw: %f", ci.x, ci.y, ci.z, dwi, dcw);

        current_phase = initial_phase;
        if (initial_phase < duty_factor)
        {
            stage = "Support Phase";
        }
        else
        {
            stage = "Transfer Phase";
        }
     
        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

        ROS_INFO("Subscribing to Joint States...");
        this->jointStateSubscriber = node.subscribe("/hexapod/joint_states", 10, &SetTrajectoryAction::jointStatesCB, this);
        
        ROS_INFO("Subscribing to FKPoseSolver service...");
        this->fkClient = node.serviceClient<hexapod_control::SolveFKPose>("/hexapod/leg_L1/fk");

        ROS_INFO("Leg %s trajectory action server ready.\n", leg_name.c_str());
    }

	void updateTrajectory();

	~SetTrajectoryAction()
	{
		this->node.shutdown();
	}

private:
    void publishFeedback(const hexapod_control::SetPoseFeedback::ConstPtr& poseFeedback);
	void publishResult(const actionlib::SimpleClientGoalState& state,
		const hexapod_control::SetPoseResult::ConstPtr& poseResult);
	void activeCB();
    void gaitModeCB(const std_msgs::StringConstPtr& msg);
    void strideHeightCB(const std_msgs::Float64ConstPtr& msg);
    void dutyFactorCB(const std_msgs::Float64ConstPtr& msg);
    void phaseCB(const std_msgs::Float64ConstPtr& msg);
    void commandCB(const geometry_msgs::Vector3ConstPtr& msg);
    geometry_msgs::Point initialFootPos(const double& base_height);
    geometry_msgs::Point getHipPos();
    std::tuple<double, double> calcStepRadius(const double& base_height);
    double calcXYAngle(const geometry_msgs::Point& from, const geometry_msgs::Point& to);
    double LPF1(const double& goal, const double& z_prev, const double& rate);
    void jointStatesCB(const sensor_msgs::JointStateConstPtr& msg);
    ros::NodeHandle node;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> client;
    ros::Subscriber jointStateSubscriber;
	ros::Subscriber gaitModeSubscriber;
    ros::Subscriber strideTimeSubscriber;
    ros::Subscriber strideHeightSubscriber;
    ros::Subscriber strideLengthSubscriber;
    ros::Subscriber dutyFactorSubscriber;
    ros::Subscriber phaseSubscriber;
    ros::Subscriber commandSubscriber;
    ros::ServiceClient linkStateClient;
    ros::ServiceClient fkClient;
    hexapod_control::Pose current_pose;
    double start, t, elapsed, last_elapsed, Tc;
	double initial_phase, current_phase;
    double speed, yaw, yaw_angle;
    std::string gait_mode;
	double stride_time, stride_height, duty_factor;
    sensor_msgs::JointState current_state;
    geometry_msgs::Point ci;
    double base_radius, base_height, hip_angle, foot_radius;
    double femur_length, tibia_length, coxa_length;
    double dwi, dcw;
    // bool initialized = false;
    std::string stage;
    double eps = 0.05;
};

void SetTrajectoryAction::updateTrajectory() // TODO: combine with commandCB?
{
    hexapod_control::Pose target_pose;

    // Calculate distance from body center to motion center
    double rm;
    try
    {
        rm = speed/yaw;
    }
    catch(const std::exception& e)
    {
        rm = 0.0;
        ROS_WARN("Divide by zero error. Yaw cannot be zero. %s", e.what());
    }

    // phi: current angle of the line connecting motion center and body center
    // geometry_msgs::Point ux; // unit vector in body CF X-axis
    // ux.x = 1.0; ux.y = 0.0; ux.z = 0.0;
    // double phi = atan2(ci.y, ci.x);
    double phi = M_PI_2 - yaw_angle;

    // Calculate motion center
    geometry_msgs::Point cm;
    cm.x = rm*cos(phi);
    cm.y = rm*sin(phi);
    cm.z = -base_height;

    // Calculate center of foot workspace
    geometry_msgs::Point hip, cw;
    hip = getHipPos();
    cw.x = dcw*cos(hip_angle) + hip.x;
    cw.y = dcw*sin(hip_angle) + hip.y;
    cw.z = -base_height;

    // Calculate current foot center
    // phi_i: current angle of the line connecting motion center and foot position
    geometry_msgs::Point ux; // unit vector in body CF X-axis
    ux.x = 1.0;
    ux.y = 0.0;
    geometry_msgs::Point cm_to_ci;
    cm_to_ci.x = ci.x - cm.x;
    cm_to_ci.y = ci.y - cm.y;
    double phi_i = atan2(cm_to_ci.y, cm_to_ci.x);
    double rmi = sqrt(pow(cm_to_ci.x, 2) + pow(cm_to_ci.y, 2));

    geometry_msgs::Point cm_to_cw;
    cm_to_cw.x = cw.x - cm.x;
    cm_to_cw.y = cw.y - cm.y;
    double phi_w = atan2(cm_to_cw.y, cm_to_cw.x);
    double rmw = sqrt(pow(cm_to_cw.x, 2) + pow(cm_to_cw.y, 2));
    
    // theta: angle from rotation center to edge of foot WS wrt cm
    // https://mathworld.wolfram.com/Circle-CircleIntersection.html
    double x = (pow(rmw, 2) - pow(dwi, 2) + pow(rmw, 2))/(2.0*rmw);
    double y = sqrt(pow(rmw, 2) - pow(x, 2));
    double theta = asin(y/rmw); // y = rmw*sin(theta)

    double dir = (yaw > 0.0) ? 1.0 : -1.0; // sign of yaw movement

    geometry_msgs::Point AEP;
    AEP.x = rmw*cos(phi_w + dir*theta) + cm.x;
    AEP.y = rmw*sin(phi_w + dir*theta) + cm.y;
    AEP.z = -base_height;

    geometry_msgs::Point cm_to_AEP;
    cm_to_AEP.x = AEP.x - cm.x;
    cm_to_AEP.y = AEP.y - cm.y;
    double phi_AEP = atan2(cm_to_AEP.y, cm_to_AEP.x);

    geometry_msgs::Point PEP;
    PEP.x = rmw*cos(phi_w - dir*theta) + cm.x;
    PEP.y = rmw*sin(phi_w - dir*theta) + cm.y;
    PEP.z = -base_height;

    geometry_msgs::Point cm_to_PEP;
    cm_to_PEP.x = PEP.x - cm.x;
    cm_to_PEP.y = PEP.y - cm.y;
    double phi_PEP = atan2(cm_to_PEP.y, cm_to_PEP.x);

    elapsed = ros::Time::now().toSec() - start;

    double support_phase, transfer_phase, transfer_time;
    double alpha, d_AEP, d_transfer, z_mh, phi_plus, delta_phi;
    double tG_plus, delta_tG, k_plus, delta_x, delta_y;

    stride_time = abs(2.0*theta/yaw); // not infinity because yaw is nonzero
    // TODO: make sure this doesn't lead to weird slow movement...

    Tc = elapsed - last_elapsed; // control interval
    last_elapsed = elapsed;
    delta_phi = yaw*Tc;

    if (gait_mode == "Default")
    {
        // Do nothing
        support_phase = 0.0;
        transfer_phase = 0.0;
    }
    else if (stage == "Support Phase")
    {
        ci.x = rmi*cos(phi_i + delta_phi) + cm.x;
        ci.y = rmi*sin(phi_i + delta_phi) + cm.y;
        ci.z = -base_height;

        support_phase = abs((phi_i - phi_AEP)/(2.0 * theta)); // support phase is angular
        current_phase = support_phase*duty_factor;
        transfer_phase = 0.0;

        if (support_phase >= 1.0)
        {
            stage = "Transfer Phase";
            support_phase = 0.0;
            current_phase = duty_factor;
        }
    }
    else if (stage == "Transfer Phase")
    {
        alpha = atan2(AEP.y - ci.y, AEP.x - ci.x);
        d_AEP = sqrt(pow(AEP.x - ci.x, 2) + pow(AEP.y - ci.y, 2));
        d_transfer = sqrt(pow(AEP.x - PEP.x, 2) + pow(AEP.y - PEP.y, 2));
        
        // transfer_phase = (phi_i - phi_PEP)/(2.0 * theta);
        transfer_phase = 1.0 - d_AEP/d_transfer; // transfer phase is linear
        current_phase = duty_factor + transfer_phase*(1.0 - duty_factor);
        support_phase = 0.0;

        transfer_time = stride_time*(1.0 - duty_factor);
        phi_plus = (1.0 - transfer_phase)*(2.0*theta);
        tG_plus = (1.0 - transfer_phase)*transfer_time;
        delta_tG = delta_phi/(phi_plus/tG_plus); 
        k_plus = round(tG_plus/delta_tG);

        z_mh = stride_height;
        delta_x = (d_AEP + z_mh)*cos(alpha)/k_plus;
        delta_y = (d_AEP + z_mh)*sin(alpha)/k_plus;

        ci.x += delta_x;
        ci.y += delta_y;

        double epsilon = 0.01;
        if (d_AEP > epsilon) // Check if above final XY location
        {
            ci.z = LPF1(-z_mh, ci.z, 0.1); // lifting
        }
        else
        {
            ci.z += (-z_mh - ci.z)/k_plus; // lowering
        }

        if (transfer_phase >= 1.0)
        {
            stage = "Support Phase";
            transfer_phase = 0.0;
            current_phase = 0.0;
        }
    }
    else
    {
        // Do nothing
        support_phase = 0.0;
        transfer_phase = 0.0;
    }

    ROS_INFO("stage: %s, support phase: %f, transfer phase: %f, stride time: %f", stage.c_str(), support_phase, transfer_phase, stride_time);
    ROS_INFO("theta: %f, x: %f, y: %f", theta, x, y);
    ROS_INFO("rmi: %f, phi_i: %f, delta_phi: %f", rmi, phi_i, delta_phi);
    ROS_INFO("AEP: (%f, %f, %f); PEP: (%f, %f, %f)", AEP.x, AEP.y, AEP.z, PEP.x, PEP.y, PEP.z);
    ROS_INFO("phi_AEP: %f, phi_PEP: %f, phi_w: %f, rmw: %f", phi_AEP, phi_PEP, phi_w, rmw);
    if (stage == "Transfer Phase")
    {
        ROS_INFO("d_AEP: %f, d_transfer: %f, alpha: %f, k_plus: %f", d_AEP, d_transfer, alpha, k_plus);
        ROS_INFO("transfer time: %f, phi_plus: %f, tG_plus: %f, delta_tG: %f",
            transfer_time, phi_plus, tG_plus, delta_tG);
        ROS_INFO("delta_x, delta_y = (%f, %f)", delta_x, delta_y);
    }
    ROS_INFO("cm: (%f, %f);   cw: (%f, %f);   ci: (%f, %f, %f)\n",
        cm.x, cm.y, cw.x, cw.y, ci.x, ci.y, ci.z);

    // Build message
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
    this->client.sendGoal(poseAction,
        boost::bind(&SetTrajectoryAction::publishResult, this, _1, _2),
        boost::bind(&SetTrajectoryAction::activeCB, this),
        boost::bind(&SetTrajectoryAction::publishFeedback, this, _1));
}

void SetTrajectoryAction::publishFeedback(const hexapod_control::SetPoseFeedback::ConstPtr& poseFeedback)
{
    current_pose = poseFeedback->current_pose;
}

void SetTrajectoryAction::publishResult(const actionlib::SimpleClientGoalState& state,
    const hexapod_control::SetPoseResult::ConstPtr& poseResult)
{

}

void SetTrajectoryAction::activeCB()
{

}

void SetTrajectoryAction::gaitModeCB(const std_msgs::StringConstPtr& msg)
{
    this->gait_mode = msg->data;
}

void SetTrajectoryAction::strideHeightCB(const std_msgs::Float64ConstPtr& msg)
{
    this->stride_height = msg->data;
}

void SetTrajectoryAction::dutyFactorCB(const std_msgs::Float64ConstPtr& msg)
{
    this->duty_factor = msg->data;
}

void SetTrajectoryAction::phaseCB(const std_msgs::Float64ConstPtr& msg)
{
    this->initial_phase = msg->data;
}

void SetTrajectoryAction::commandCB(const geometry_msgs::Vector3ConstPtr& msg)
{
    this->speed = msg->x;
    this->yaw = msg->y;
    this->yaw_angle = msg->z;

    SetTrajectoryAction::updateTrajectory();
}

geometry_msgs::Point SetTrajectoryAction::initialFootPos(const double& base_height)
{
    geometry_msgs::Point foot;
    foot.x = foot_radius*cos(hip_angle);
    foot.y = foot_radius*sin(hip_angle);
    foot.z = -base_height;

    return foot;
}

geometry_msgs::Point SetTrajectoryAction::getHipPos()
{
    // TODO: make hip location a gazebo look-up so it's more accurate?
    geometry_msgs::Point hip;
    hip.x = base_radius*cos(hip_angle);
    hip.y = base_radius*sin(hip_angle);
    hip.z = 0.0;

    return hip;
}

std::tuple<double, double> SetTrajectoryAction::calcStepRadius(const double& base_height)
{
    double beta, dwi, dcw;
    beta = asin(base_height/(femur_length + tibia_length)); // full stretch leg angle
    dwi = (femur_length + tibia_length)*cos(beta)/2.0; // workspace radius
    dcw = coxa_length + dwi; // radius from hip to workspace center

    return std::make_tuple(dwi, dcw);
}

double SetTrajectoryAction::calcXYAngle(const geometry_msgs::Point& from, const geometry_msgs::Point& to)
{
    double from_cross_to = to.x*from.y - to.y*from.x;
    double from_mag = sqrt(pow(from.x, 2) + pow(from.y, 2));
    double to_mag = sqrt(pow(to.x, 2) + pow(to.y, 2));
    ROS_INFO("cross, from, to, angle: %f, %f, %f, %f", from_cross_to, from_mag, to_mag, from_cross_to/(from_mag*to_mag));

    double result = asin(from_cross_to/(from_mag*to_mag)) - M_PI; // TODO: WRONG for cases when to.y < 0
    return result;
}

// 1st order low-pass filter
double SetTrajectoryAction::LPF1(const double& goal, const double& z_prev, const double& rate)
{
    // https://dsp.stackexchange.com/questions/60277/is-the-typical-implementation-of-low-pass-filter-in-c-code-actually-not-a-typica
    // rate must be between 0.0 and 1.0
    // as rate approaches 0, the cutoff of the filter decreases
    double z_next = (1 - rate) * z_prev + rate * goal; // "simplistic" low pass filter

    return z_next;
}

void SetTrajectoryAction::jointStatesCB(const sensor_msgs::JointStateConstPtr& msg)
{
    sensor_msgs::JointState temp = *msg.get();
    int hip_index, knee_index, ankle_index;
    for (int i = 0; i < temp.name.size(); ++i)
    {
        std::string name_i = temp.name[i];
        if (name_i.find(leg_name) != std::string::npos)
        {
            if (name_i.find("hip") != std::string::npos)
            {
                hip_index = i;
            }
            else if (name_i.find("knee") != std::string::npos)
            {
                knee_index = i;
            }
            else if (name_i.find("ankle") != std::string::npos)
            {
                ankle_index = i;
            }
        }
    }

    this->current_state.name     = {temp.name[hip_index],     temp.name[knee_index],     temp.name[ankle_index]};
    this->current_state.position = {temp.position[hip_index], temp.position[knee_index], temp.position[ankle_index]};
    this->current_state.velocity = {temp.velocity[hip_index], temp.velocity[knee_index], temp.velocity[ankle_index]};
    this->current_state.effort   = {temp.effort[hip_index],   temp.effort[knee_index],   temp.effort[ankle_index]};
}

int main(int argc, char **argv)
{
    ROS_INFO("Starting Trajectory Action Server...");
    ros::init(argc, argv, "leg_" + leg_name + "_trajectory_action");
    ROS_INFO("Initialized ros...");

    SetTrajectoryAction trajectoryActionServer;
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
