#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/SetPoseAction.h"
#include "hexapod_control/GaitAction.h"
#include "hexapod_control/Pose.h"
#include "hexapod_control/SolveFKPose.h"
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
		this->strideTimeSubscriber = node.subscribe("/hexapod/gait/stride_time", 10, &SetTrajectoryAction::strideTimeCB, this);
		this->strideHeightSubscriber = node.subscribe("/hexapod/gait/stride_height", 10, &SetTrajectoryAction::strideHeightCB, this);
		this->strideLengthSubscriber = node.subscribe("/hexapod/gait/stride_length", 10, &SetTrajectoryAction::strideLengthCB, this);
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
        hip_angle = hip_angle*M_PI/180; // convert to radians

        start = ros::Time::now().toSec();

        std::tie(dwi, dcw) = calcStepRadius(base_height); // TODO: make base_height variable
        foot_radius = base_radius + dcw;

        ci = initialFootPos(base_height); // initialize with current foot position
        ROS_INFO("initial ci: (%f, %f, %f)", ci.x, ci.y, ci.z);
     
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
    void strideTimeCB(const std_msgs::Float64ConstPtr& msg);
    void strideHeightCB(const std_msgs::Float64ConstPtr& msg);
    void strideLengthCB(const std_msgs::Float64ConstPtr& msg);
    void dutyFactorCB(const std_msgs::Float64ConstPtr& msg);
    void phaseCB(const std_msgs::Float64ConstPtr& msg);
    void commandCB(const geometry_msgs::Vector3ConstPtr& msg);
    geometry_msgs::Point getFootPos();
    geometry_msgs::Point initialFootPos(const double& base_height);
    geometry_msgs::Point getHipPos();
    std::tuple<double, double> calcStepRadius(const double& base_height);
    double calcXYAngle(const geometry_msgs::Point& a, const geometry_msgs::Point& b);
    double LPF1(const double& goal, const double& z_prev);
    void jointStatesCB(const sensor_msgs::JointStateConstPtr& msg);
    ros::NodeHandle node;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> client;
    ros::Subscriber jointStateSubscriber;
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
	double stride_time, stride_height, stride_length, duty_factor;
    sensor_msgs::JointState current_state;
    geometry_msgs::Point hip;
    geometry_msgs::Point ci;
    double base_radius, base_height, hip_angle, foot_radius;
    double femur_length, tibia_length, coxa_length;
    double dwi, dcw;
    // bool initialized = false;
    std::string stage;
    double eps = 0.05;
};

void SetTrajectoryAction::updateTrajectory()
{
    // initialized = false;

    hexapod_control::Pose target_pose;

    // TODO: while loop prevents other callbacks from happening!
    // come up with other method that is smooth and at rate(50)

    // ros::Rate rate(50);
    // while (true)
    // {
        // Get current foot position
        // geometry_msgs::Point ci = getFootPos();
        // ROS_INFO("FK ci: (%f, %f, %f)", ci.x, ci.y, ci.z);

        elapsed = ros::Time::now().toSec() - start;
        // TODO: make stride_time based on stride_length?
        t = elapsed + initial_phase*stride_time; // account for initial_phase
        Tc = elapsed - last_elapsed; // control interval
        last_elapsed = elapsed;

        // Calculate the current phase
        current_phase = fmod(t/stride_time, 1.0);
        // if (!initialized && current_phase >= duty_factor)
        // {
        //     initialized = true;
        // }

        // Calculate distance from body center to motion center
        double rm;
        try
        {
            rm = speed/yaw;
        }
        catch(const std::exception& e)
        {
            rm = 0.0;
            ROS_WARN("Divide by zero error. %s", e.what());
        }

        // phi: current angle of the line connecting motion center and body center
        // geometry_msgs::Point ux; // unit vector in body CF X-axis
        // ux.x = 1.0; ux.y = 0.0; ux.z = 0.0;
        // double phi = calcXYAngle(ci, ux);
        double phi = M_PI_2 - yaw_angle;
        double delta_phi = yaw*Tc;

        // Calculate motion center
        geometry_msgs::Point cm;
        cm.x = rm*cos(phi);
        cm.y = rm*sin(phi);
        cm.z = -base_height;

        // Calculate center of foot workspace
        geometry_msgs::Point cw;
        hip = getHipPos();
        cw.x = dcw*cos(hip_angle) + hip.x;
        cw.y = dcw*sin(hip_angle) + hip.y;
        cw.z = -base_height;

        // Calculate current foot center
        // phi_i: current angle of the line connecting motion center and foot position
        geometry_msgs::Point ux; // unit vector in body CF X-axis
        ux.x = 1.0; // TODO: is this in body CF or something wrong like foot or hip CF?
        ux.y = 0.0;
        geometry_msgs::Point cm_to_ci; // vector from cm to ci
        cm_to_ci.x = ci.x - cm.x;
        cm_to_ci.y = ci.y - cm.y;
        double phi_i = calcXYAngle(cm_to_ci, ux); // TODO: confirm this works
        double rmi = sqrt(pow(cm_to_ci.x, 2) + pow(cm_to_ci.y, 2));

        // theta: angle from rotation center to edge of foot WS wrt cm
        double theta = acos(1 - 0.125*pow(dwi/rmi, 2));
        // stride length is 2*dwi (center of foot workspace)

        geometry_msgs::Point AEP;
        geometry_msgs::Point cm_to_cw; // vector from cm to cw
        cm_to_cw.x = cw.x - cm.x;
        cm_to_cw.y = cw.y - cm.y;
        double phi_w = calcXYAngle(cm_to_cw, ux); // angle from cm to cw
        AEP.x = rmi*cos(phi_w - theta) + cm.x;
        AEP.y = rmi*sin(phi_w - theta) + cm.y;
        AEP.z = -base_height;

        double transfer_phase, alpha, d_AEP, z_mh, transfer_time, phi_plus;
        double tG_plus, delta_tG, k_plus, delta_x, delta_y;

        // Support phase
        if (current_phase < duty_factor)
        {
            // double support_phase = current_phase/duty_factor;
            // ci.x = rmi*cos(phi_w + theta*(2*support_phase - 1)) + cm.x;
            // ci.y = rmi*sin(phi_w + theta*(2*support_phase - 1)) + cm.y;

            ci.x = rmi*cos(phi_i + delta_phi) + cm.x;
            ci.y = rmi*sin(phi_i + delta_phi) + cm.y;
            ci.z = -base_height;
            stage = "Support Phase";
        }
        // Transfer phase
        else
        {
            transfer_phase = (current_phase - duty_factor)/(1.0 - duty_factor);
            alpha = atan2(AEP.y - ci.y, AEP.x - ci.x);
            d_AEP = sqrt(pow(AEP.x - ci.x, 2) + pow(AEP.y - ci.y, 2));
            z_mh = stride_height;

            transfer_time = stride_time*(1.0 - duty_factor);
            phi_plus = (1.0 - transfer_phase)*(2*theta);
            tG_plus = (1.0 - transfer_phase)*transfer_time;
            delta_tG = delta_phi/(phi_plus/tG_plus); 
            k_plus = round(tG_plus/delta_tG);

            delta_x = (d_AEP + z_mh)*cos(alpha)/k_plus;
            delta_y = (d_AEP + z_mh)*sin(alpha)/k_plus;

            ci.x += delta_x;
            ci.y += delta_y;

            double epsilon = 0.01;
            if (ci.x - AEP.x > epsilon)
            {
                ci.z = LPF1(z_mh, ci.z); // lifting
            }
            else
            {
                ci.z += (-z_mh - ci.z)/k_plus; // lowering
            }
            stage = "Transfer Phase";
        }

        ROS_INFO("stage: %s, phase: %f", stage.c_str(), current_phase);
        if (stage == "Support Phase")
        {
            ROS_INFO("rmi: %f, phi_i: %f, delta_phi: %f", rmi, phi_i, delta_phi);
        }
        else if (stage == "Transfer Phase")
        {
            ROS_INFO("d_AEP: %f, alpha: %f, k_plus: %f", d_AEP, alpha, k_plus);
            ROS_INFO("rmi: %f, phi_w: %f, theta: %f", rmi, phi_w, theta);
            ROS_INFO("transfer time: %f, phi_plus: %f, tG_plus: %f, delta_tG: %f",
                transfer_time, phi_plus, tG_plus, delta_tG);
            ROS_INFO("AEP: (%f, %f, %f)", AEP.x, AEP.y, AEP.z);
        }
        ROS_INFO("cm: (%f, %f);    cw: (%f, %f);    ci: (%f, %f, %f)\n",
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

    //     rate.sleep();
    // }
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

void SetTrajectoryAction::strideTimeCB(const std_msgs::Float64ConstPtr& msg)
{
    this->stride_time = msg->data;
}

void SetTrajectoryAction::strideHeightCB(const std_msgs::Float64ConstPtr& msg)
{
    this->stride_height = msg->data;
}

void SetTrajectoryAction::strideLengthCB(const std_msgs::Float64ConstPtr& msg)
{
    this->stride_length = msg->data;
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

geometry_msgs::Point SetTrajectoryAction::getFootPos()
{
    // Send FK request to service
    hexapod_control::SolveFKPose fkMsg;
    fkMsg.request.joint_positions = current_state.position;
    fkClient.call(fkMsg);
    
    geometry_msgs::Point foot;
    foot.x = fkMsg.response.solution.x;
    foot.y = fkMsg.response.solution.y;
    foot.z = fkMsg.response.solution.z;
    
    return foot;
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
    // TODO: make hip location a gazebo look-up so it's more accurate
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
    dwi = (femur_length + tibia_length)*cos(beta)/2; // workspace radius
    dcw = coxa_length + dwi; // radius from hip to workspace center

    return std::make_tuple(dwi, dcw);
}

double SetTrajectoryAction::calcXYAngle(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
    double a_dot_b = a.x*b.x + a.y*b.y;
    double a_mag = sqrt(pow(a.x, 2) + pow(a.y, 2));
    double b_mag = sqrt(pow(b.x, 2) + pow(b.y, 2));

    return acos(a_dot_b/(a_mag*b_mag));
}

// 1st order low-pass filter
double SetTrajectoryAction::LPF1(const double& goal, const double& z_prev)
{
    // https://dsp.stackexchange.com/questions/60277/is-the-typical-implementation-of-low-pass-filter-in-c-code-actually-not-a-typica
    double a = 0.1; // as a approaches 0, the cutoff of the filter decreases
    double z_next = (1 - a) * z_prev + a * goal; // "simplistic" low pass filter
    // double z_next = (1 - a) * z_prev + a * (goal + goal_prev) / 2; // "proper" low pass filter (zero at Nyquist and more attenuation close to Nyquist)

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
