#include <ros/ros.h>
#include "hexapod_control/SolveIKPose.h"
#include "hexapod_control/SolveFKPose.h"
#include "hexapod_control/Pose.h"

namespace hexapod_control
{
    class AnalyticalKinematicsSolver
    {
    public:
        AnalyticalKinematicsSolver(ros::NodeHandle* node)
        {
            this->node = node;

            ROS_INFO("Initializing FKPoseSolver service...");
            this->fkL1PoseService = node->advertiseService("/hexapod/leg_L1/afk", &AnalyticalKinematicsSolver::solveFKPoseLegL1, this);
            this->fkR1PoseService = node->advertiseService("/hexapod/leg_R1/afk", &AnalyticalKinematicsSolver::solveFKPoseLegR1, this);
            this->fkL2PoseService = node->advertiseService("/hexapod/leg_L2/afk", &AnalyticalKinematicsSolver::solveFKPoseLegL2, this);
            this->fkR2PoseService = node->advertiseService("/hexapod/leg_R2/afk", &AnalyticalKinematicsSolver::solveFKPoseLegR2, this);
            this->fkL3PoseService = node->advertiseService("/hexapod/leg_L3/afk", &AnalyticalKinematicsSolver::solveFKPoseLegL3, this);
            this->fkR3PoseService = node->advertiseService("/hexapod/leg_R3/afk", &AnalyticalKinematicsSolver::solveFKPoseLegR3, this);

            ROS_INFO("Initializing IKPoseSolver service...");
            this->ikL1PoseService = node->advertiseService("/hexapod/leg_L1/aik", &AnalyticalKinematicsSolver::solveIKPoseLegL1, this);
            this->ikR1PoseService = node->advertiseService("/hexapod/leg_R1/aik", &AnalyticalKinematicsSolver::solveIKPoseLegR1, this);
            this->ikL2PoseService = node->advertiseService("/hexapod/leg_L2/aik", &AnalyticalKinematicsSolver::solveIKPoseLegL2, this);
            this->ikR2PoseService = node->advertiseService("/hexapod/leg_R2/aik", &AnalyticalKinematicsSolver::solveIKPoseLegR2, this);
            this->ikL3PoseService = node->advertiseService("/hexapod/leg_L3/aik", &AnalyticalKinematicsSolver::solveIKPoseLegL3, this);
            this->ikR3PoseService = node->advertiseService("/hexapod/leg_R3/aik", &AnalyticalKinematicsSolver::solveIKPoseLegR3, this);
            
            node->getParam("/hexapod/geometry/base/radius", base_radius);
            node->getParam("/hexapod/geometry/base/z_offset", base_z_offset);
            node->getParam("/hexapod/geometry/coxa_length", coxa_length);
            node->getParam("/hexapod/geometry/femur_length", femur_length);
            node->getParam("/hexapod/geometry/tibia_length", tibia_length);

            ROS_INFO("Ready.");
        }

        ~AnalyticalKinematicsSolver()
        {
            this->node->shutdown();
        }

        bool solveFKPose(hexapod_control::SolveFKPoseRequest& req,
                         hexapod_control::SolveFKPoseResponse& res,
                         int leg_ID)
        {
            // TODO: Slight error on X and Y coordinates compared to other kinematics solver
            std::vector<double> joint_positions = req.joint_positions;
            double theta1 = joint_positions[0];
            double theta2 = joint_positions[1];
            double theta3 = joint_positions[2];

            switch(leg_ID)
            {
                case 0: // L1
                    leg_name = "L1";
                    break;
                case 1: // R1
                    leg_name = "R1";
                    break;
                case 2: // L2
                    leg_name = "L2";
                    break;
                case 3: // R2
                    leg_name = "R2";
                    break;
                case 4: // L3
                    leg_name = "L3";
                    break;
                case 5: // R3
                    leg_name = "R3";
                    break;
            }

            double c_hip, s_hip, f_prime, t_prime;
            Vector3 hip, knee, ankle, foot;

            node->getParam("/hexapod/geometry/leg_" + leg_name + "/hip_angle", hip_angle);
            hip_angle = hip_angle * M_PI / 180; // convert to radians

            hip = {base_radius*cos(hip_angle),
                   base_radius*sin(hip_angle),
                   0.0};

            c_hip = cos(hip_angle + theta1);
            s_hip = sin(hip_angle + theta1);
            knee = {hip.x + coxa_length*c_hip,
                    hip.y + coxa_length*s_hip,
                    hip.z};

            f_prime = femur_length*cos(theta2); // femur length projected onto ground
            ankle = {knee.x + f_prime*c_hip,
                     knee.y + f_prime*s_hip,
                     knee.z + femur_length*sin(theta2)};

            t_prime = tibia_length*cos(theta3 - theta2);  // tibia length projected onto ground
            foot = {ankle.x + t_prime*c_hip,
                    ankle.y + t_prime*s_hip,
                    ankle.z - tibia_length*sin(theta3 - theta2)};

            res.solution.x = foot.x;
            res.solution.y = foot.y;
            res.solution.z = foot.z;

            ROS_INFO("hip_angle: %f", hip_angle);
            ROS_INFO("hip: {%f, %f, %f}", hip.x, hip.y, hip.z);
            ROS_INFO("knee: {%f, %f, %f}", knee.x, knee.y, knee.z);
            ROS_INFO("ankle: {%f, %f, %f}", ankle.x, ankle.y, ankle.z);
            ROS_INFO("foot: {%f, %f, %f}", foot.x, foot.y, foot.z);

            // TODO: actually calculate rotation based on ankle to foot angle
            res.solution.rotx = std::vector<double>{1.0, 0.0, 0.0};
            res.solution.roty = std::vector<double>{0.0, 1.0, 0.0};
            res.solution.rotz = std::vector<double>{0.0, 0.0, 1.0};

            res.result = 0;
            res.error_message = "No error";

            ROS_DEBUG("Solved position: {%f, %f, %f}", res.solution.x, res.solution.y, res.solution.z);
            ROS_DEBUG("Solved rotx: {%f, %f, %f}", res.solution.rotx[0], res.solution.rotx[1], res.solution.rotx[2]);
            ROS_DEBUG("Solved roty: {%f, %f, %f}", res.solution.roty[0], res.solution.roty[1], res.solution.roty[2]);
            ROS_DEBUG("Solved rotz: {%f, %f, %f}", res.solution.rotz[0], res.solution.rotz[1], res.solution.rotz[2]);

            return true;
        }

        bool solveIKPose(hexapod_control::SolveIKPoseRequest& req,
                         hexapod_control::SolveIKPoseResponse& res,
                         int leg_ID)
        {
            std::vector<double> initial_state = req.initial_state;
            Vector3 b2f = {req.goal.x, req.goal.y, req.goal.z}; // base to foot
            std::vector<std::vector<double>> R = {req.goal.rotx, req.goal.roty, req.goal.rotz};

            switch(leg_ID)
            {
                case 0: // L1
                    leg_name = "L1";
                    break;
                case 1: // R1
                    leg_name = "R1";
                    break;
                case 2: // L2
                    leg_name = "L2";
                    break;
                case 3: // R2
                    leg_name = "R2";
                    break;
                case 4: // L3
                    leg_name = "L3";
                    break;
                case 5: // R3
                    leg_name = "R3";
                    break;
            }

            double theta1, theta2, theta3, f2k_len, phi, rho;
            Vector3 s, b2h, f2h, s2, b2k, f2k;

            node->getParam("/hexapod/geometry/leg_" + leg_name + "/hip_angle", hip_angle);
            hip_angle = hip_angle * M_PI / 180; // convert to radians

            // l = o + R*s - u
            s = {base_radius*cos(hip_angle),
                 base_radius*sin(hip_angle),
                 0.0}; // 0.01? 0.0?
            b2h = multRbyS(R, s); // base to hip
            
            f2h = b2h - b2f; // foot to hip
            theta1 = atan2(-f2h.y, -f2h.x) - hip_angle; // neg to get correct direction
            // TODO: confirm it's atan-hip and not hip-atan

            s2 = {s.x + coxa_length*cos(hip_angle + theta1),
                  s.y + coxa_length*sin(hip_angle + theta1),
                  s.z};
            b2k = multRbyS(R, s2); // base to knee

            f2k = b2k - b2f; // foot to knee
            f2k_len = f2k.norm(); // length from foot to knee

            phi = asin((f2k.z - f2h.z) / coxa_length);
            rho = atan(f2k.z / sqrt(pow(f2k.x, 2) + pow(f2k.y, 2)));
            
            theta2 = acos((pow(femur_length, 2) + pow(f2k_len, 2) - pow(tibia_length, 2)) / (2*femur_length*f2k_len)) - (phi + rho);
            theta3 = M_PI - acos((pow(femur_length, 2) + pow(tibia_length, 2) - pow(f2k_len, 2)) / (2*femur_length*tibia_length));

            ROS_INFO("hip_angle: %f", hip_angle);
            ROS_INFO("atan: %f", atan2(-f2h.y, -f2h.x));
            ROS_INFO("theta1: %f", theta1);
            ROS_INFO("theta2: %f", theta2);
            ROS_INFO("theta3: %f", theta3);
            ROS_INFO("b2h: {%f, %f, %f}", b2h.x, b2h.y, b2h.z);
            ROS_INFO("f2h: {%f, %f, %f}", f2h.x, f2h.y, f2h.z);
            ROS_INFO("b2k: {%f, %f, %f}", b2k.x, b2k.y, b2k.z);
            ROS_INFO("f2k: {%f, %f, %f}", f2k.x, f2k.y, f2k.z);

            res.solution = std::vector<double>{theta1, theta2, theta3};
            res.result = 0;
            res.error_message = "No error";

            ROS_DEBUG("Solved joint angles: {%f, %f, %f}", res.solution[0], res.solution[1], res.solution[2]);

            return true;
        }

       struct Vector3
        {
            double x, y, z;

            Vector3() :
                x(0.0), y(0.0), z(0.0) {}

            Vector3(double x, double y, double z) :
                x(x), y(y), z(z) {}

            Vector3 operator+(const Vector3& other)
            {
                return Vector3(x + other.x, y + other.y, z + other.z);
            }

            Vector3 operator-(const Vector3& other)
            {
                return Vector3(x - other.x, y - other.y, z - other.z);
            }

            double norm()
            {
                return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
            }
        };

        Vector3 multRbyS(std::vector<std::vector<double>> R, Vector3 s)
        {
            Vector3 result;
            result.x = R[0][0]*s.x + R[0][1]*s.y + R[0][2]*s.z;
            result.y = R[1][0]*s.x + R[1][1]*s.y + R[1][2]*s.z;
            result.z = R[2][0]*s.x + R[2][1]*s.y + R[2][2]*s.z;

            return result;
        }

        bool solveFKPoseLegL1(hexapod_control::SolveFKPoseRequest &req,
                              hexapod_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, 0);
        }

        bool solveFKPoseLegR1(hexapod_control::SolveFKPoseRequest &req,
                              hexapod_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, 1);
        }

        bool solveFKPoseLegL2(hexapod_control::SolveFKPoseRequest &req,
                              hexapod_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, 2);
        }

        bool solveFKPoseLegR2(hexapod_control::SolveFKPoseRequest &req,
                              hexapod_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, 3);
        }

        bool solveFKPoseLegL3(hexapod_control::SolveFKPoseRequest &req,
                              hexapod_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, 4);
        }

        bool solveFKPoseLegR3(hexapod_control::SolveFKPoseRequest &req,
                              hexapod_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, 5);
        }

        bool solveIKPoseLegL1(hexapod_control::SolveIKPoseRequest &req,
                              hexapod_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, 0);
        }

        bool solveIKPoseLegR1(hexapod_control::SolveIKPoseRequest &req,
                              hexapod_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, 1);
        }

        bool solveIKPoseLegL2(hexapod_control::SolveIKPoseRequest &req,
                              hexapod_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, 2);
        }

        bool solveIKPoseLegR2(hexapod_control::SolveIKPoseRequest &req,
                              hexapod_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, 3);
        }

        bool solveIKPoseLegL3(hexapod_control::SolveIKPoseRequest &req,
                              hexapod_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, 4);
        }

        bool solveIKPoseLegR3(hexapod_control::SolveIKPoseRequest &req,
                              hexapod_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, 5);
        }

 

    private:
        ros::NodeHandle* node;
        ros::ServiceServer fkL1PoseService;
        ros::ServiceServer fkR1PoseService;
        ros::ServiceServer fkL2PoseService;
        ros::ServiceServer fkR2PoseService;
        ros::ServiceServer fkL3PoseService;
        ros::ServiceServer fkR3PoseService;
        ros::ServiceServer ikL1PoseService;
        ros::ServiceServer ikR1PoseService;
        ros::ServiceServer ikL2PoseService;
        ros::ServiceServer ikR2PoseService;
        ros::ServiceServer ikL3PoseService;
        ros::ServiceServer ikR3PoseService;
        double base_radius, base_z_offset, hip_angle, coxa_length, femur_length, tibia_length;
        std::string leg_name;
    };
} //namespace hexapod_control

int main(int argc, char **argv)
{
    ROS_INFO("Starting Kinematics solver...");
    ros::init(argc, argv, "analytical_kinematics_solver");
    ros::NodeHandle node;

    hexapod_control::AnalyticalKinematicsSolver solver(&node);

    //ros::spin();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}