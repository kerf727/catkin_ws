// Helper Functions for gait_controller.cpp

#include "gait_controller_library.h"

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
    const double& hip_angle, const double& dcw)
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
    double rmw = sqrt(pow(cm_to_cw.x, 2.0) + pow(cm_to_cw.y, 2.0));
    std::tuple<double, double> result(phi_w, rmw);
    
    return result;
}

double calcTheta(const double& rmw, const double& dwi)
{
    // theta: angle from rotation center to edge of foot WS wrt cm
    // https://mathworld.wolfram.com/Circle-CircleIntersection.html
    double x = (pow(rmw, 2.0) - pow(dwi, 2.0) + pow(rmw, 2.0))/(2.0*rmw);
    double y = sqrt(pow(rmw, 2.0) - pow(x, 2.0));
    
    return asin(y/rmw); // y = rmw*sin(theta)
}

std::tuple<geometry_msgs::Point, geometry_msgs::Point> calcAEPPEP(
    const geometry_msgs::Point& cm, 
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

std::tuple<double, double, double> calcPhiAEPPEP(
    const geometry_msgs::Point& cm, const geometry_msgs::Point& AEP,
    const geometry_msgs::Point& PEP, const double& phi_i_old)
{
    geometry_msgs::Point cm_to_AEP, cm_to_PEP;
    cm_to_AEP.x = AEP.x - cm.x;
    cm_to_AEP.y = AEP.y - cm.y;
    cm_to_PEP.x = PEP.x - cm.x;
    cm_to_PEP.y = PEP.y - cm.y;
    
    double phi_i, phi_AEP, phi_PEP;
    phi_i = phi_i_old;
    phi_AEP = atan2(cm_to_AEP.y, cm_to_AEP.x);
    phi_PEP = atan2(cm_to_PEP.y, cm_to_PEP.x);

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
    
    std::tuple<double, double, double> result(phi_i, phi_AEP, phi_PEP);

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

std::tuple<double, double> calcStepRadius(
    const double& base_height, const double& coxa_length,
    const double& femur_length, const double& tibia_length)
{
    double beta, dwi, dcw;
    beta = asin(base_height/(femur_length + tibia_length)); // full stretch leg angle
    dwi = (femur_length + tibia_length)*cos(beta)/2.0; // workspace radius
    dcw = coxa_length + dwi; // radius from hip to workspace center

    // Limit dwi to use smaller workspace radius without changing cw workspace center
    dwi *= 0.5;

    std::tuple<double, double> result(dcw, dwi);

    return result;
}