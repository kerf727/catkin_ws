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

std::string leg_name = "L3";

class SetTrajectoryAction
{
public:
    SetTrajectoryAction() :
        client("leg_" + leg_name + "_pose_action", true)
    {
        this->node = node;

		ROS_INFO("Subscribing to Gait Controller and Teleop...");
		this->gaitModeSubscriber = node.subscribe("/hexapod/gait/gait_mode", 10, &SetTrajectoryAction::gaitModeCB, this);
		this->commandSubscriber = node.subscribe("/hexapod/gait/command", 10, &SetTrajectoryAction::commandCB, this);
		this->buttonSubscriber = node.subscribe("/hexapod/teleop/button", 10, &SetTrajectoryAction::buttonCB, this);

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

        ux.x = 1.0;
        ux.y = 0.0;

        calcStepRadius(base_height); // TODO: make base_height variable

        updateCw(base_height);
        ci = cw; // set ci to be workspace center to start

        ROS_INFO("initial ci: (%f, %f, %f); dwi: %f, dcw: %f", ci.x, ci.y, ci.z, dwi, dcw);

        gait_type = "ripple";
        node.getParam("/hexapod/gait/" + gait_type + "/phase/" + leg_name, initial_phase);
        node.getParam("/hexapod/gait/" + gait_type + "/duty_factor", duty_factor);
        node.getParam("/hexapod/gait/" + gait_type + "/stride_height", stride_height);

        if (initial_phase < 1.0 - duty_factor)
        {
            stage = "Transfer Phase";
            support_phase = 0.0;
            transfer_phase = initial_phase/(1.0 - duty_factor);
        }
        else
        {
            stage = "Support Phase";
            support_phase = (duty_factor - initial_phase)/duty_factor;
            transfer_phase = 0.0;
        }
        ROS_INFO("initial_phase: %f, stage: %s, support_phase: %f, transfer_phase: %f", initial_phase, stage.c_str(), support_phase, transfer_phase);

        initialized = false;
     
        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");

        ROS_INFO("Subscribing to Joint States...");
        this->jointStateSubscriber = node.subscribe("/hexapod/joint_states", 10, &SetTrajectoryAction::jointStatesCB, this);
        
        ROS_INFO("Subscribing to FKPoseSolver service...");
        this->fkClient = node.serviceClient<hexapod_control::SolveFKPose>("/hexapod/leg_L1/fk");

        ROS_INFO("Leg %s trajectory action server ready.\n", leg_name.c_str());
    }

	~SetTrajectoryAction()
	{
		this->node.shutdown();
	}

private:
    
    void commandCB(const geometry_msgs::Vector3ConstPtr& msg);
    void supportPhase();
    void transferPhase();
    void updateCm(const double& yaw_angle, const double& base_height);
    void updateCw(const double& base_height);
    void updatePhiIW();
    void updateTheta();
    void updateAEPPEP();
    void calcStepRadius(const double& base_height);
    double calcXYAngle(const geometry_msgs::Point& from, const geometry_msgs::Point& to);
    double LPF1(const double& goal, const double& z_prev, const double& rate);
    void jointStatesCB(const sensor_msgs::JointStateConstPtr& msg);
    void publishFeedback(const hexapod_control::SetPoseFeedback::ConstPtr& poseFeedback);
	void publishResult(const actionlib::SimpleClientGoalState& state,
		const hexapod_control::SetPoseResult::ConstPtr& poseResult);
	void activeCB();
    void gaitModeCB(const std_msgs::StringConstPtr& msg);
    void buttonCB(const std_msgs::BoolConstPtr& msg);
    ros::NodeHandle node;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> client;
    ros::Subscriber jointStateSubscriber;
	ros::Subscriber gaitModeSubscriber;
    ros::Subscriber commandSubscriber;
    ros::Subscriber buttonSubscriber;
    ros::ServiceClient linkStateClient;
    ros::ServiceClient fkClient;
    hexapod_control::Pose current_pose;
    double start, t, elapsed, last_elapsed, Tc;
	double initial_phase;
    double speed, yaw, yaw_angle;
    std::string gait_mode;
	double stride_time, stride_height, duty_factor;
    sensor_msgs::JointState current_state;
    geometry_msgs::Point ci, cm, cw, cm_to_ci, cm_to_cw, hip, ux;
    geometry_msgs::Point AEP, PEP, cm_to_AEP, cm_to_PEP;
    double base_radius, base_height, hip_angle, foot_radius;
    double femur_length, tibia_length, coxa_length;
    double dwi, dcw;
    double support_phase, transfer_phase, last_transfer_phase, phase_diff, lowering_rate;
    double alpha, d_AEP, d_PEP, PEP_to_AEP, phi_plus, delta_phi;
    double phi_i, phi_w, phi_AEP, phi_PEP;
    double rm, rmw, theta, dir;
    double transfer_time, tG_plus, delta_tG, k_plus, delta_x, delta_y;
    bool initialized = false;
    bool B_button = false;
    std::string gait_type = "ripple";
    int gait_counter = 0;
    std::string stage;
    double eps = 0.05;
    double d_epsilon = 0.003;
};

void SetTrajectoryAction::commandCB(const geometry_msgs::Vector3ConstPtr& msg)
{
    this->speed = msg->x;
    this->yaw = msg->y;
    this->yaw_angle = msg->z;

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

    updateCm(yaw_angle, base_height); // motion center
    
    updateCw(base_height); // center of foot workspace

    updatePhiIW();

    updateTheta();

    stride_time = abs(2.0*theta/yaw); // not infinity because yaw is nonzero

    updateAEPPEP();

    // TODO: place ci at initial_phase position
    if (!initialized && gait_mode != "Default")
    {
        ci = (stage == "Support Phase") ? AEP : PEP; // place at initial_phase position
        initialized = true;
    }

    if (stage == "Support Phase" && gait_mode != "Default" && initialized)
    {
        supportPhase();
    }
    else if (stage == "Transfer Phase" && gait_mode != "Default" && initialized)
    {
        transferPhase();
    }
    else // Do nothing
    {
        support_phase = 0.0;
        transfer_phase = 0.0;
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
            ROS_INFO("delta_x, delta_y = (%f, %f)", delta_x, delta_y);
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
    this->client.sendGoal(poseAction,
        boost::bind(&SetTrajectoryAction::publishResult, this, _1, _2),
        boost::bind(&SetTrajectoryAction::activeCB, this),
        boost::bind(&SetTrajectoryAction::publishFeedback, this, _1));
}

void SetTrajectoryAction::supportPhase()
{
    support_phase = abs(phi_i - phi_AEP)/(2.0 * theta);

    if (support_phase >= 1.0)
    {
        ROS_INFO("                        Switch to Transfer Phase                        \n");
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

void SetTrajectoryAction::transferPhase()
{
    alpha = atan2(AEP.y - ci.y, AEP.x - ci.x);
    d_AEP = sqrt(pow(AEP.x - ci.x, 2) + pow(AEP.y - ci.y, 2));
    PEP_to_AEP = sqrt(pow(AEP.x - PEP.x, 2) + pow(AEP.y - PEP.y, 2));
    d_PEP = PEP_to_AEP - d_AEP;
    phi_plus = dir*(phi_i - phi_AEP);
    
    double current_height = ci.z + base_height;
    double transfer_distance = PEP_to_AEP + stride_height;

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

        delta_x = (d_AEP + stride_height)*cos(alpha)/k_plus;
        delta_y = (d_AEP + stride_height)*sin(alpha)/k_plus;

        ci.x += delta_x;
        ci.y += delta_y;
        ci.z = LPF1(-stride_height, ci.z, 0.1);
    }
    else // lowering
    {
        transfer_phase += phase_diff;
        if (transfer_phase >= 1.0)
        {
            ROS_INFO("                        Switch to Support Phase                        \n");
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

void SetTrajectoryAction::updateCm(const double& yaw_angle, const double& base_height)
{
    // phi: current angle of the line connecting motion center and body center
    // geometry_msgs::Point ux; // unit vector in body CF X-axis
    // ux.x = 1.0; ux.y = 0.0; ux.z = 0.0;
    // double phi = atan2(ci.y, ci.x);
    double phi = yaw_angle - M_PI_2;

    cm.x = rm*cos(phi);
    cm.y = rm*sin(phi);
    cm.z = -base_height;
}

void SetTrajectoryAction::updateCw(const double& base_height)
{
    double foot_radius = base_radius + dcw;
    cw.x = foot_radius*cos(hip_angle);
    cw.y = foot_radius*sin(hip_angle);
    cw.z = -base_height;
}

void SetTrajectoryAction::updatePhiIW()
{
    // phi_i: current angle of the line connecting motion center and foot position
    cm_to_ci.x = ci.x - cm.x;
    cm_to_ci.y = ci.y - cm.y;
    phi_i = atan2(cm_to_ci.y, cm_to_ci.x);
    // rmi = rmw always

    cm_to_cw.x = cw.x - cm.x;
    cm_to_cw.y = cw.y - cm.y;
    phi_w = atan2(cm_to_cw.y, cm_to_cw.x);
    rmw = sqrt(pow(cm_to_cw.x, 2) + pow(cm_to_cw.y, 2));
}

void SetTrajectoryAction::updateTheta()
{
    // theta: angle from rotation center to edge of foot WS wrt cm
    // https://mathworld.wolfram.com/Circle-CircleIntersection.html
    double x = (pow(rmw, 2) - pow(dwi, 2) + pow(rmw, 2))/(2.0*rmw);
    double y = sqrt(pow(rmw, 2) - pow(x, 2));
    theta = asin(y/rmw); // y = rmw*sin(theta)
}

void SetTrajectoryAction::updateAEPPEP()
{
    AEP.x = rmw*cos(phi_w - dir*theta) + cm.x;
    AEP.y = rmw*sin(phi_w - dir*theta) + cm.y;
    AEP.z = -base_height;

    cm_to_AEP.x = AEP.x - cm.x;
    cm_to_AEP.y = AEP.y - cm.y;
    phi_AEP = atan2(cm_to_AEP.y, cm_to_AEP.x);

    PEP.x = rmw*cos(phi_w + dir*theta) + cm.x;
    PEP.y = rmw*sin(phi_w + dir*theta) + cm.y;
    PEP.z = -base_height;

    cm_to_PEP.x = PEP.x - cm.x;
    cm_to_PEP.y = PEP.y - cm.y;
    phi_PEP = atan2(cm_to_PEP.y, cm_to_PEP.x);
}

void SetTrajectoryAction::calcStepRadius(const double& base_height)
{
    double beta;
    beta = asin(base_height/(femur_length + tibia_length)); // full stretch leg angle
    dwi = (femur_length + tibia_length)*cos(beta)/2.0; // workspace radius
    dcw = coxa_length + dwi; // radius from hip to workspace center

    // Limit dwi to use smaller workspace radius without changing cw workspace center
    dwi *= 0.7;
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

void SetTrajectoryAction::buttonCB(const std_msgs::BoolConstPtr& msg)
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
