#include <ros/ros.h>
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "hexapod_control/SolveIKPose.h"
#include "hexapod_control/SolveFKPose.h"
#include "hexapod_control/Pose.h"

namespace hexapod_control
{
    class KinematicsSolver
    {
    public:
        KinematicsSolver(ros::NodeHandle* node, KDL::Tree tree)
        {
            this->node = node;

            KDL::Chain chain;
            tree.getChain("hexapod/base_link", "hexapod/L1_foot_link", this->leg_L1);
            tree.getChain("hexapod/base_link", "hexapod/R1_foot_link", this->leg_R1);
            tree.getChain("hexapod/base_link", "hexapod/L2_foot_link", this->leg_L2);
            tree.getChain("hexapod/base_link", "hexapod/R2_foot_link", this->leg_R2);
            tree.getChain("hexapod/base_link", "hexapod/L3_foot_link", this->leg_L3);
            tree.getChain("hexapod/base_link", "hexapod/R3_foot_link", this->leg_R3);

            ROS_INFO("Initializing IKPoseSolver service...");
            this->ikL1PoseService = node->advertiseService("/hexapod/leg_L1/ik", &KinematicsSolver::solveIKPoseLegL1, this);
            this->ikR1PoseService = node->advertiseService("/hexapod/leg_R1/ik", &KinematicsSolver::solveIKPoseLegR1, this);
            this->ikL2PoseService = node->advertiseService("/hexapod/leg_L2/ik", &KinematicsSolver::solveIKPoseLegL2, this);
            this->ikR2PoseService = node->advertiseService("/hexapod/leg_R2/ik", &KinematicsSolver::solveIKPoseLegR2, this);
            this->ikL3PoseService = node->advertiseService("/hexapod/leg_L3/ik", &KinematicsSolver::solveIKPoseLegL3, this);
            this->ikR3PoseService = node->advertiseService("/hexapod/leg_R3/ik", &KinematicsSolver::solveIKPoseLegR3, this);
            
            ROS_INFO("Initializing FKPoseSolver service...");
            this->fkL1PoseService = node->advertiseService("/hexapod/leg_L1/fk", &KinematicsSolver::solveFKPoseLegL1, this);
            this->fkR1PoseService = node->advertiseService("/hexapod/leg_R1/fk", &KinematicsSolver::solveFKPoseLegR1, this);
            this->fkL2PoseService = node->advertiseService("/hexapod/leg_L2/fk", &KinematicsSolver::solveFKPoseLegL2, this);
            this->fkR2PoseService = node->advertiseService("/hexapod/leg_R2/fk", &KinematicsSolver::solveFKPoseLegR2, this);
            this->fkL3PoseService = node->advertiseService("/hexapod/leg_L3/fk", &KinematicsSolver::solveFKPoseLegL3, this);
            this->fkR3PoseService = node->advertiseService("/hexapod/leg_R3/fk", &KinematicsSolver::solveFKPoseLegR3, this);

            ROS_INFO("Ready.");
        }

        ~KinematicsSolver()
        {
            this->node->shutdown();
        }

        bool solveIKPose(hexapod_control::SolveIKPoseRequest& req,
                         hexapod_control::SolveIKPoseResponse& res,
                         KDL::Chain chain)
        {
            ROS_DEBUG("Solving IK Pose...");
            ROS_DEBUG("Initializing solver...");

            Eigen::Matrix<double, 6, 1> weights;
            weights.col(0) << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;

            KDL::ChainIkSolverPos_LMA posSolver(chain, weights);
            std::vector<double> initial_state = req.initial_state;

            KDL::JntArray jntArray = KDL::JntArray(3);
            jntArray.data = Eigen::Map<const Eigen::VectorXd>(initial_state.data(), initial_state.size());

            ROS_DEBUG("Parsing rotation...");
            KDL::Vector rotx = KDL::Vector(req.goal.rotx[0], req.goal.rotx[1], req.goal.rotx[2]);
            KDL::Vector roty = KDL::Vector(req.goal.roty[0], req.goal.roty[1], req.goal.roty[2]);
            KDL::Vector rotz = KDL::Vector(req.goal.rotz[0], req.goal.rotz[1], req.goal.rotz[2]);

            ROS_DEBUG("Parsing posiion...");
            KDL::Vector targetpos = KDL::Vector(req.goal.x, req.goal.y, req.goal.z);
            KDL::Rotation targetrot = KDL::Rotation(rotx, roty, rotz);

            ROS_DEBUG("Generating goal...");
            KDL::Frame targetFrame = KDL::Frame(targetrot, targetpos);

            KDL::JntArray jntSol = KDL::JntArray(3);
            ROS_DEBUG("[%f, %f, %f]; [[%f, %f, %f], [%f, %f, %f], [%f, %f, %f]]",
                        targetpos.data[0], targetpos.data[1], targetpos.data[2],
                        rotx.data[0], roty.data[0], rotz.data[0],
                        rotx.data[1], roty.data[1], rotz.data[1],
                        rotx.data[2], roty.data[2], rotz.data[2]);

            ROS_DEBUG("Running solver...");
            int result = posSolver.CartToJnt(jntArray, targetFrame, jntSol);
            ROS_DEBUG("Result: %d", result);

            for (int i = 0; i < jntSol.rows() * jntSol.columns(); i++)
            {
                ROS_DEBUG("%f", jntSol.data[i]);
            }

            std::vector<double> sol(jntSol.data.data(), jntSol.data.data() + jntSol.data.rows() * jntSol.data.cols());

            res.solution = sol;
            res.result = result;
            res.error_message = std::string(posSolver.strError(res.result));
            return true;
        }

        bool solveFKPose(hexapod_control::SolveFKPoseRequest &req,
                         hexapod_control::SolveFKPoseResponse &res,
                         KDL::Chain chain)
        {
            ROS_DEBUG("FK Pose request received.");
            KDL::ChainFkSolverPos_recursive posSolver(chain);

            ROS_DEBUG("Joint positions:");
            std::vector<double> joint_positions = req.joint_positions;
            for (int i = 0; i < joint_positions.size(); ++i)
            {
                ROS_DEBUG("  %f", joint_positions[i]);
            }

            KDL::JntArray jntArray = KDL::JntArray(7);
            jntArray.data = Eigen::Map<const Eigen::VectorXd>(joint_positions.data(), joint_positions.size());

            KDL::Frame frame = KDL::Frame();

            int result = posSolver.JntToCart(jntArray, frame);
            res.result = result;
            res.error_message = std::string(posSolver.strError(res.result));

            res.solution.x = frame.p.data[0];
            res.solution.y = frame.p.data[1];
            res.solution.z = frame.p.data[2];

            std::vector<double> rotx = std::vector<double> {
                frame.M.UnitX().data[0],
                frame.M.UnitX().data[1],
                frame.M.UnitX().data[2]
            };
            
            std::vector<double> roty = std::vector<double> {
                frame.M.UnitY().data[0],
                frame.M.UnitY().data[1],
                frame.M.UnitY().data[2]
            };

            std::vector<double> rotz = std::vector<double> {
                frame.M.UnitZ().data[0],
                frame.M.UnitZ().data[1],
                frame.M.UnitZ().data[2]
            };

            res.solution.rotx = rotx;
            res.solution.roty = roty;
            res.solution.rotz = rotz;

            ROS_DEBUG("Solved position: {%f, %f, %f}", res.solution.x, res.solution.y, res.solution.z);
            ROS_DEBUG("Solved rotx: {%f, %f, %f}", rotx[0], rotx[1], rotx[2]);
            ROS_DEBUG("Solved roty: {%f, %f, %f}", roty[0], roty[1], roty[2]);
            ROS_DEBUG("Solved rotz: {%f, %f, %f}", rotz[0], rotz[1], rotz[2]);

            return true;
        }

        bool solveIKPoseLegL1(hexapod_control::SolveIKPoseRequest &req,
                              hexapod_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, this->leg_L1);
        }

        bool solveIKPoseLegR1(hexapod_control::SolveIKPoseRequest &req,
                              hexapod_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, this->leg_R1);
        }

        bool solveIKPoseLegL2(hexapod_control::SolveIKPoseRequest &req,
                              hexapod_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, this->leg_L2);
        }

        bool solveIKPoseLegR2(hexapod_control::SolveIKPoseRequest &req,
                              hexapod_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, this->leg_R2);
        }

        bool solveIKPoseLegL3(hexapod_control::SolveIKPoseRequest &req,
                              hexapod_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, this->leg_L3);
        }

        bool solveIKPoseLegR3(hexapod_control::SolveIKPoseRequest &req,
                              hexapod_control::SolveIKPoseResponse &res)
        {
            return solveIKPose(req, res, this->leg_R3);
        }

        bool solveFKPoseLegL1(hexapod_control::SolveFKPoseRequest &req,
                              hexapod_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, this->leg_L1);
        }

        bool solveFKPoseLegR1(hexapod_control::SolveFKPoseRequest &req,
                              hexapod_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, this->leg_R1);
        }

        bool solveFKPoseLegL2(hexapod_control::SolveFKPoseRequest &req,
                              hexapod_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, this->leg_L2);
        }

        bool solveFKPoseLegR2(hexapod_control::SolveFKPoseRequest &req,
                              hexapod_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, this->leg_R2);
        }

        bool solveFKPoseLegL3(hexapod_control::SolveFKPoseRequest &req,
                              hexapod_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, this->leg_L3);
        }

        bool solveFKPoseLegR3(hexapod_control::SolveFKPoseRequest &req,
                              hexapod_control::SolveFKPoseResponse &res)
        {
            return solveFKPose(req, res, this->leg_R3);
        }

    private:
        ros::NodeHandle* node;
        ros::ServiceServer ikL1PoseService;
        ros::ServiceServer ikR1PoseService;
        ros::ServiceServer ikL2PoseService;
        ros::ServiceServer ikR2PoseService;
        ros::ServiceServer ikL3PoseService;
        ros::ServiceServer ikR3PoseService;
        ros::ServiceServer fkL1PoseService;
        ros::ServiceServer fkR1PoseService;
        ros::ServiceServer fkL2PoseService;
        ros::ServiceServer fkR2PoseService;
        ros::ServiceServer fkL3PoseService;
        ros::ServiceServer fkR3PoseService;
        KDL::Chain leg_L1;
        KDL::Chain leg_R1;
        KDL::Chain leg_L2;
        KDL::Chain leg_R2;
        KDL::Chain leg_L3;
        KDL::Chain leg_R3;
    };
} // namespace hexapod_control

int main(int argc, char **argv)
{
    ROS_INFO("Starting Inverse Kinematics solver...");
    ROS_INFO("Building robot tree from param server...");
    ros::init(argc, argv, "kinematics_solver");
    ros::NodeHandle node;
    std::string robot_desc_string;
    node.param("/robot_description", robot_desc_string, std::string());
    KDL::Tree tree;

    if (!kdl_parser::treeFromString(robot_desc_string, tree))
    {
        ROS_INFO("Failed to construct kdl tree");
        return 0;
    }
    else
    {
        ROS_INFO("Build chain of %d segments", tree.getNrOfSegments());
    }

    KDL::SegmentMap::const_iterator it;
    for (it = tree.getSegments().begin(); it != tree.getSegments().end(); it++)
    {
        ROS_INFO("Segment %s", it->second.segment.getName().c_str());
    }

    hexapod_control::KinematicsSolver solver(&node, tree);

    //ros::spin();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}