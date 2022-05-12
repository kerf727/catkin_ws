// Helper Functions for gait_controller.cpp
#include "geometry_msgs/Point.h"

geometry_msgs::Point calcCm(
    const double& yaw_angle, const double& rm, const double& base_height);

geometry_msgs::Point calcCw(
    const double& base_height, const double& base_radius,
    const double& hip_angle, const double& dcw);

double calcPhiI(
    const geometry_msgs::Point& cm, const geometry_msgs::Point& ci);

std::tuple<double, double> calcPhiW(
    const geometry_msgs::Point& cm, const geometry_msgs::Point& cw);

double calcTheta(const double& rmw, const double& dwi);

std::tuple<geometry_msgs::Point, geometry_msgs::Point> calcAEPPEP(
    const geometry_msgs::Point& cm, 
    const double& rmw, const double& phi_w, const double& dir,
    const double& theta, const double& base_height);

std::tuple<double, double, double> calcPhiAEPPEP(
    const geometry_msgs::Point& cm, const geometry_msgs::Point& AEP,
    const geometry_msgs::Point& PEP, const double& phi_i_old);

double LPF1(const double& goal, const double& z_prev, const double& rate);

double mapRange(const double& inValue,
                const double& minInRange, const double& maxInRange,
                const double& minOutRange, const double& maxOutRange);

std::tuple<double, double> calcStepRadius(
    const double& base_height, const double& coxa_length,
    const double& femur_length, const double& tibia_length);
