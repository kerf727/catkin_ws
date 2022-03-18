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

        std::tie(dw, dcw) = calcStepRadius(base_height); // TODO: make base_height variable
     
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

	void commandCB(const geometry_msgs::Vector3ConstPtr& msg)
	{
		this->speed = msg->x;
		this->yaw = msg->y;
		this->yaw_angle = msg->z;
        
        double start = ros::Time::now().toSec();
		double eps = 0.05;
		
        // initialized = false;

		double t, elapsed, last_elapsed, Tc;
		hexapod_control::Pose target_pose;

		current_phase = initial_phase;

        // Get current foot position
        geometry_msgs::Point ci = getFootPos();
        ROS_INFO("time 0 ci: (%f, %f, %f)", ci.x, ci.y, ci.z);
        ROS_INFO("dw: %f, dcw: %f\n", dw, dcw);

		ros::Rate rate(50);
		while (true)
		{
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
            if (yaw != 0.0)
            {
                rm = speed/yaw;
            }
            else
            {
                rm = (speed > 0.0) ? inf : -inf;
            }

            // phi: current angle of the line connecting motion center and body center
            // geometry_msgs::Point ux; // unit vector in body CF X-axis
            // ux.x = 1.0; ux.y = 0.0; ux.z = 0.0;
            // double phi = calcXYAngle(ci, ux);
            double phi = yaw_angle; // TODO: confirm if this can just be desired yaw angle

            // Calculate motion center
            geometry_msgs::Point cm;
            cm.x = rm*cos(M_PI - phi); // TODO: confirm angle calculation
            cm.y = rm*sin(M_PI - phi);
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
            double delta_phi = yaw*Tc;
            double rmi = sqrt(pow(cm_to_ci.x, 2) + pow(cm_to_ci.y, 2));

            // theta: angle from center to edge of foot WS wrt cm
            double theta = acos(1 - 0.125*pow(dw/(base_radius + dcw), 2));

            geometry_msgs::Point AEP;
            geometry_msgs::Point cm_to_cw; // vector from cm to cw
            cm_to_cw.x = cw.x - cm.x;
            cm_to_cw.y = cw.y - cm.y;
            double phi_w = calcXYAngle(cm_to_cw, ux); // angle from cm to cw
            AEP.x = rmi*cos(phi_w - theta) + cm.x;
            AEP.y = rmi*sin(phi_w - theta) + cm.x;
            AEP.z = -base_height;

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
                double transfer_phase = (current_phase - duty_factor)/(1.0 - duty_factor);
                double alpha = atan2(AEP.y - ci.y, AEP.x - ci.x);
                double d_AEP = sqrt(pow(AEP.x - ci.x, 2) + pow(AEP.y - ci.y, 2));
                double z_mh = stride_height;

                double transfer_time = stride_time*(1.0 - duty_factor);
                double tG_plus = (1.0 - transfer_phase)*transfer_time;
                double phi_plus = (1.0 - transfer_phase)*(2*theta);
                double delta_tG = delta_phi/(phi_plus/tG_plus); 
                double k_plus = round(tG_plus/delta_tG);

                double delta_x = (d_AEP + z_mh)*cos(alpha)/k_plus;
                double delta_y = (d_AEP + z_mh)*sin(alpha)/k_plus;

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
            ROS_INFO("speed: %f, yaw: %f, yaw_angle : %f", speed, yaw, yaw_angle);
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

			rate.sleep();
		}
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

    void strideTimeCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->stride_time = msg->data;
	}

    void strideHeightCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->stride_height = msg->data;
	}

    void strideLengthCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->stride_length = msg->data;
	}

    void dutyFactorCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->duty_factor = msg->data;
	}

    void phaseCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->initial_phase = msg->data;
	}

    geometry_msgs::Point getFootPos()
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

    geometry_msgs::Point getHipPos()
    {
        // TODO: make hip location a gazebo look-up so it's more accurate
        geometry_msgs::Point hip;
        hip.x = base_radius*cos(hip_angle);
        hip.y = base_radius*sin(hip_angle);
        hip.z = 0.0;

        return hip;
    }

    std::tuple<double, double> calcStepRadius(const double& base_height)
    {
        double beta, dw, dcw;
        beta = asin(base_height/(femur_length + tibia_length)); // full stretch leg angle
        dw = (femur_length + tibia_length)*cos(beta)/2; // workspace radius
        dcw = coxa_length + dw; // radius from hip to workspace center

        return std::make_tuple(dw, dcw);
    }

    double calcXYAngle(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
    {
        double a_dot_b = a.x*b.x + a.y*b.y;
        double a_mag = sqrt(pow(a.x, 2) + pow(a.y, 2));
        double b_mag = sqrt(pow(b.x, 2) + pow(b.y, 2));

        return acos(a_dot_b/(a_mag*b_mag));
    }

    // 1st order low-pass filter
    double LPF1(const double& goal, const double& z_prev) {
        // https://dsp.stackexchange.com/questions/60277/is-the-typical-implementation-of-low-pass-filter-in-c-code-actually-not-a-typica
        double a = 0.1; // as a approaches 0, the cutoff of the filter decreases
        double z_next = (1 - a) * z_prev + a * goal; // "simplistic" low pass filter
        // double z_next = (1 - a) * z_prev + a * (goal + goal_prev) / 2; // "proper" low pass filter (zero at Nyquist and more attenuation close to Nyquist)

        return z_next;
    }

    void jointStatesCB(const sensor_msgs::JointStateConstPtr& msg)
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

private:
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
	double initial_phase, current_phase;
    double speed, yaw, yaw_angle;
	double stride_time, stride_height, stride_length, duty_factor;
    sensor_msgs::JointState current_state;
    geometry_msgs::Point hip;
    double base_radius, base_height, hip_angle;
    double femur_length, tibia_length, coxa_length;
    double dw, dcw;
    // bool initialized = false;
    std::string stage;
    double inf = 1e6; // approximate infinity. increase if necessary
};

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
