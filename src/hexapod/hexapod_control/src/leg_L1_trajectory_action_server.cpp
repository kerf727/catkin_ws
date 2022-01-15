#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "hexapod_control/SetPoseAction.h"
#include "hexapod_control/GaitAction.h"
#include "hexapod_control/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/GetLinkState.h"
#include "math.h"

class SetTrajectoryAction
{
public:
    SetTrajectoryAction(std::string name):
        server(node, name, boost::bind(&SetTrajectoryAction::executeCB, this, _1), false),
        client("leg_L1_pose_action", true),
        actionName(name)
    {
        this->node = node;

        ROS_INFO("Waiting for Pose State Server...");
        this->client.waitForServer(ros::Duration(30));

		ROS_INFO("Subscribing to Gait Controller...");
		this->dutyFactorSubscriber = node.subscribe("/hexapod/gait/duty_factor", 10, &SetTrajectoryAction::dutyFactorCB, this);
		this->bodyVelocitySubscriber = node.subscribe("/hexapod/gait/body_velocity", 10, &SetTrajectoryAction::bodyVelocityCB, this);
        this->stopCommandSubscriber = node.subscribe("/hexapod/gait/stop", 10, &SetTrajectoryAction::stopCommandCB, this);

        ROS_INFO("Subscribing to Gazebo GetLinkState service");
        this->linkStateClient = node.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
		
		ROS_INFO("Starting...");
		server.start();
    }

	~SetTrajectoryAction()
	{
		this->node.shutdown();
	}

	void executeCB(const hexapod_control::GaitGoalConstPtr& goal)
	{
		double start = ros::Time::now().toSec();

		this->initial_phase = goal->initial_phase;
		this->stride_time = goal->stride_time;
		this->stride_height = goal->stride_height;
        this->z_mh = goal->stride_height;
		double eps = 0.05;
		
        bool preempted = false;
        stop = false;
        initialized = false;

		double t, elapsed;
		steps = 0;
		current_phase = this->initial_phase;
		hexapod_control::Pose targetPose;

		ros::Rate rate(50);
		while (true)
		{
            // Check if preempted or canceled
            if (server.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Trajectory action preempted, ending trajectory...");
                preempted = true;
            }

            elapsed = ros::Time::now().toSec() - start;
			t = elapsed + this->initial_phase * stride_time; // account for initial_phase

            // Calculate the current phase
			current_phase = fmod(t / this->stride_time, 1.0);
            if (!initialized && current_phase >= duty_factor)
            {
                initialized = true;
            }

            // Calculate the target position
            geometry_msgs::Point position;
            if (!initialized)
            {
                position = initialTrajectory(elapsed);
            }
            else
            {
                position = sineTrajectory(current_phase);
            }

            if ((stop || preempted) && stage.compare("Support Phase") == 0)
            {
                break;
            }

			// Build message
			targetPose.x = position.x;
			targetPose.y = position.y;
			targetPose.z = position.z;
			targetPose.rotx = std::vector<double>{1.0, 0.0, 0.0};
			targetPose.roty = std::vector<double>{0.0, 1.0, 0.0};
			targetPose.rotz = std::vector<double>{0.0, 0.0, 1.0};

			// Send parameters to pose action client
			hexapod_control::SetPoseGoal poseAction;
			poseAction.goal = targetPose;
			poseAction.eps = eps;
			this->client.sendGoal(poseAction,
				boost::bind(&SetTrajectoryAction::publishResult, this, _1, _2),
				boost::bind(&SetTrajectoryAction::activeCB, this),
				boost::bind(&SetTrajectoryAction::publishFeedback, this, _1));

			this->actionFeedback.current_phase = current_phase;
			this->actionFeedback.targetPose = targetPose;
			this->actionFeedback.currentPose = currentPose;
            this->actionFeedback.stage = stage;
			server.publishFeedback(this->actionFeedback);

            if (current_phase == initial_phase)
            {
                steps++;
            }

			rate.sleep();
		}

		// Publish result
        this->actionResult.stepsTaken = steps;
        if (preempted)
        {
            server.setPreempted(this->actionResult);
        }
        else
        {
		    server.setSucceeded(this->actionResult);
        }
	}

	void publishFeedback(const hexapod_control::SetPoseFeedback::ConstPtr& poseFeedback)
	{
		currentPose = poseFeedback->currentPose;
	}

	void publishResult(const actionlib::SimpleClientGoalState& state,
		const hexapod_control::SetPoseResult::ConstPtr& poseResult)
	{

	}

	void activeCB()
	{

	}

	void dutyFactorCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->duty_factor = msg->data;
	}

	void bodyVelocityCB(const std_msgs::Float64ConstPtr& msg)
	{
		this->body_velocity = msg->data;
	}

    void stopCommandCB(const std_msgs::BoolConstPtr& msg)
	{
		this->stop = msg->data;
	}

private:
	std::string actionName;
	ros::NodeHandle node;
	actionlib::SimpleActionServer<hexapod_control::GaitAction> server;
	actionlib::SimpleActionClient<hexapod_control::SetPoseAction> client;
    ros::ServiceClient linkStateClient;
	hexapod_control::GaitFeedback actionFeedback;
	hexapod_control::GaitResult actionResult;
	ros::Subscriber dutyFactorSubscriber;
	ros::Subscriber bodyVelocitySubscriber;
    ros::Subscriber stopCommandSubscriber;
	hexapod_control::Pose currentPose;
	double initial_phase;
	double current_phase;
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
    
    struct hex // current state of the robot
    {
        int n_legs = 6;                 // number of legs
        geometry_msgs::Point F;         // body position
        double Fv;                      // body velocity (m/s)
        double Fw;                      // body angular velocity (rad/s)
        geometry_msgs::Point Cm;        // body motion center
        geometry_msgs::Vector3 Fr;      // body to motion center distance
        double Fphi;                    // body to motion center angle
        double h_gr[6];                 // estimated ground height (m)
        double beta[6];                 // leg to ground angle
        double gamma[6];                // body to hip angle
        double r_ws[6];                 // workspace radius
        geometry_msgs::Point Cw[6];     // foot workspace centers
        geometry_msgs::Point Ci[6];     // foot positions
        double phase[6];                // leg phase
        geometry_msgs::Vector3 r_m[6];  // foot to motion center distance
        double phi[6];                  // foot to motion center angle
        double phi_plus[6];             // angle to PEP (remaining)
        double phi_minus[6];            // angle to AEP (already travelled)
        double t_Gplus[6];              // time remaining in current gait phase
        double k_plus[6];               // max number of control intervals to reach the next AEP or PEP point
        double alpha[6];                // (swing) angle from foot to AEP point
        double d_AEP[6];                // (swing) distance from foot to AEP point
        geometry_msgs::Point AEP[6];    // anterior extreme position
        geometry_msgs::Point PEP[6];    // posterior extreme position
        
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
                double x2 = Cm[i].x + (dx * a / d);
                double y2 = Cm[i].y + (dy * a / d);
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
                geometry_msgs::Vector3 vPEP = PEP[i] - Cm[i];
                geometry_msgs::Vector3 vAEP = AEP[i] - Cm[i];
                geometry_msgs::Vector3 vCi = Ci[i] - Cm[i];
                geometry_msgs::Vector3 vCf = Cf[i] - Cm[i];
                geometry_msgs::Vector3 vCm = [1, 0, 0]; // vector pointing in x-axis direction

                phi = v2Dangle(vCf, vCm);
                phi[i] = v2Dangle(vCi, vCm);
                phi_plus[i] = v2Dangle(vPEP, vCi);
                phi_minus[i] = v2Dangle(vAEP, vCi);
            }
        }

        // Update phase timing values for each leg
        void updatePhaseTiming()
        {
            double delta_phi = hex.Fw * Tc;
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
                Cw[i].z = h_gr[i]; // ground height
            }
        }
        
        // Calculate angle between two 2D vectors
        double v2Dangle(geometry_msgs::Vector3& v1, geometry_msgs::Vector3& v2)
        {
            v1dotv2 = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
            v1_length = v1.x*v1.x + v1.y*v1.y + v1.z*v1.z;
            v2_length = v2.x*v2.x + v2.y*v2.y + v2.z*v2.z;
            return acos(v1dotv2 / (v1_length * v2_length));
        }
    }

private:
    geometry_msgs::Point initialTrajectory(double& elapsed)
    {
        // Position w.r.t. body
        geometry_msgs::Point position;
        double x, y, z;

        // Just move forward until reach initial phase
        x = x_offset - body_velocity * elapsed;
        y = y_offset;
        z = z_offset;
        stage = "Initialization";

        position.x = x;
        position.y = y;
        position.z = z;

        return position;
    }

    geometry_msgs::Point sineTrajectory(double& phase)
    {
        // Position w.r.t. body
        geometry_msgs::Point position;
        double x, y, z;

        double strideLength = stride_time * body_velocity;

        // Support phase
        if (phase < duty_factor)
        {
            double supportPhase = phase / duty_factor;
            x = x = x_offset + (strideLength / 2 - strideLength * supportPhase);
            y = y_offset;
            z = z_offset;
            stage = "Support Phase";
        }
        // Transfer phase
        else
        {
            double transferPhase = (phase - duty_factor) / (1.0 - duty_factor);
            x = x_offset + (strideLength * transferPhase - strideLength / 2);
            y = y_offset;
            z = 0.5 * stride_height * cos(2 * M_PI * transferPhase - M_PI) + z_offset + stride_height/2;
            stage = "Transfer Phase";
        }
        
        position.x = x;
        position.y = y;
        position.z = z;

        return position;
    }

    std::string getLegName(int& i)
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

    geometry_msgs::Point getPosition(std::string linkName)
    {
        gazebo_msgs::GetLinkState linkStateMsg;
        linkStateMsg.request.link_name = linkName;
        linkStateMsg.request.reference_frame = "world";
        this->linkStateClient.call(linkStateMsg);

        return linkStateMsg.response.link_state.pose.position;
    }

    geometry_msgs::Point getBodyPosition(int& i)
    {
        return getPosition("body");
    }

    geometry_msgs::Point getHipPosition(int& i)
    {
        return getPosition("hip_" + getLegName(i));
    }

    geometry_msgs::Point getFootPosition(int& i)
    {
        return getPosition("foot_" + getLegName(i));
    }

    // 1st order low-pass filter
    double LPF1(double& goal, double& z_prev) {
        // https://dsp.stackexchange.com/questions/60277/is-the-typical-implementation-of-low-pass-filter-in-c-code-actually-not-a-typica
        double a = 0.1; // as a approaches 0, the cutoff of the filter decreases
        double z_next = (1 - a) * z_prev + a * goal; // "simplistic" low pass filter
        // double z_next = (1 - a) * z_prev + a * (goal + goal_prev) / 2; // "proper" low pass filter (zero at Nyquist and more attenuation close to Nyquist)

        return z_next;
    }

    void stancePhase(int& i)
    {
        hex.updateAll(); // update all robot values

        double delta_phi = hex.Fw * Tc;
        hex.Fr = hex.Fv / hex.Fw; // radius of center of motion
        hex.Cm.x = hex.Fr * cos(hex.Fphi - M_PI_2);
        hex.Cm.y = hex.Fr * sin(hex.Fphi - M_PI_2);
        hex.Cm.z = 0.0;

        hex.Ci[i] = getFootPosition(i);
        hex.r_m[i] = hex.Ci[i] - hex.Cm; // distance from C_i (foot location) to c_m (common motion center)

        hex.Ci[i].x = hex.r_m[i]*cos(hex.phi[i] - delta_phi) + hex.Cm.x;
        hex.Ci[i].y = hex.r_m[i]*sin(hex.phi[i] - delta_phi) + hex.Cm.y;
        hex.Ci[i].z = hex.h_gr[i];
    }

    void liftingPhase(int& i)
    {
        hex.updateAll(); // update all robot values

        hex.alpha[i] = atan2(hex.AEP[i].y - hex.Ci[i].y, hex.AEP[i].x - hex.Ci[i].x);
        hex.d_AEP[i] = sqrt(pow(hex.AEP[i].x - hex.Ci[i].x, 2) + pow(hex.AEP[i].y - hex.Ci[i].y, 2));
        
        double k_plus = hex.k_plus(i);
        hex.Ci[i].x += (hex.d_AEP[i] + z_mh) * cos(hex.alpha[i]) / k_plus;
        hex.Ci[i].y += (hex.d_AEP[i] + z_mh) * sin(hex.alpha[i]) / k_plus;
        hex.Ci[i].z = LPF1(z_mh, hex.Ci[i].z);
    }

    void loweringPhase(int& i)
    {
        hex.updateAll(); // update all robot values

        double k_plus = hex.k_plus(i);
        hex.Ci[i].z += (hex.h_gr[i] - hex.Ci[i].z) / k_plus;
    }
};

int main(int argc, char **argv)
{
    ROS_INFO("Starting Trajectory Action Server...");
    ros::init(argc, argv, "leg_L1_trajectory_action");
    ROS_INFO("Initialized ros...");

    SetTrajectoryAction actionServer("leg_L1_trajectory_action");
    ROS_INFO("Spinning node...");
    ros::spin();
    return 0;
}
