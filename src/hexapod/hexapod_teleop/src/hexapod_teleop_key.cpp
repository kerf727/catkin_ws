// #include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "hexapod_teleop/TeleopAction.h"
#include "geometry_msgs/Twist.h"
#include "signal.h"
#include "termios.h"
#include "stdio.h"

#define KEYCODE_RIGHT 0x43 
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_Q 0x71
// #define KEYCODE_R 0x64
// #define KEYCODE_L 0x61
// #define KEYCODE_U 0x77
#define KEYCODE_D 0x73

class TeleopHexapod
{
public:
    TeleopHexapod(std::string name) : L1_IK_client("leg_L1_IK_action", true),
                                      R1_IK_client("leg_R1_IK_action", true),
                                      L2_IK_client("leg_L2_IK_action", true),
                                      R2_IK_client("leg_R2_IK_action", true),
                                      L3_IK_client("leg_L3_IK_action", true),
                                      R3_IK_client("leg_R3_IK_action", true),
                                      actionName(name)
    {
        pos_x = 0.0;
        rot_x = 0.0;
        pos_y = 0.0;
        rot_y = 0.0;
        pos_z = 0.0;
        rot_z = 0.0;
        pos_scale = 1.0;
        rot_scale = 1.0;
        stationary_mode = "Stationary/Position";

        // node.param("scale_pos", pos_scale, pos_scale);
        // node.param("scale_rot", rot_scale, rot_scale);

        ROS_INFO("Waiting for IK Servers...");
        L1_IK_client.waitForServer(ros::Duration(30));
        R1_IK_client.waitForServer(ros::Duration(30));
        L2_IK_client.waitForServer(ros::Duration(30));
        R2_IK_client.waitForServer(ros::Duration(30));
        L3_IK_client.waitForServer(ros::Duration(30));
        R3_IK_client.waitForServer(ros::Duration(30));

        ROS_INFO("Teleop controller ready.");
    }

    void keyLoop(int kfd, struct termios cooked, struct termios raw)
    {
        char c;
        bool dirty = false;

        // Get the console in raw mode                                                              
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &=~ (ICANON | ECHO);

        // Setting a new line, then end of file                         
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);

        puts("Reading from keyboard");
        puts("---------------------------");
        puts("Use arrow keys to move the hexapod.");

        for(;;)
        {
            // Get the next event from the keyboard  
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                exit(-1);
            }

            pos_x = rot_x = 0.0;
            pos_y = rot_y = 0.0;
            pos_z = rot_z = 0.0;
            ROS_DEBUG("value: 0x%02X\n", c);

            switch(c)
            {
                case KEYCODE_LEFT:
                    ROS_INFO("LEFT");
                    if (stationary_mode == "Stationary/Position")
                    {
                        pos_x = -trans;
                    }
                    else if (stationary_mode == "Stationary/Orientation")
                    {
                        rot_y =  -rot_deg*M_PI/180;
                    }
                    dirty = true;
                    break;

                case KEYCODE_RIGHT:
                    ROS_INFO("RIGHT");
                    if (stationary_mode == "Stationary/Position")
                    {
                        pos_x =  trans;
                    }
                    else if (stationary_mode == "Stationary/Orientation")
                    {
                        rot_y =  rot_deg*M_PI/180;
                    }
                    dirty = true;
                    break;

                case KEYCODE_UP:
                    ROS_INFO("UP");
                    if (stationary_mode == "Stationary/Position")
                    {
                        pos_y =  trans;
                    }
                    else if (stationary_mode == "Stationary/Orientation")
                    {
                        rot_x =  -rot_deg*M_PI/180;
                    }
                    dirty = true;
                    break;

                case KEYCODE_DOWN:
                    ROS_INFO("DOWN");
                    if (stationary_mode == "Stationary/Position")
                    {
                        pos_y = -trans;
                    }
                    else if (stationary_mode == "Stationary/Orientation")
                    {
                        rot_x =  rot_deg*M_PI/180;
                    }
                    dirty = true;
                    break;

                case KEYCODE_Q:
                    ROS_INFO("Q");
                    // Switch between Position and Orientation modes
                    if (stationary_mode == "Stationary/Position")
                    {
                        stationary_mode = "Stationary/Orientation";
                    }
                    else if (stationary_mode == "Stationary/Orientation")
                    {
                        stationary_mode = "Stationary/Position";
                    }
                    dirty = true;
                    break;

                case KEYCODE_D:
                    ROS_INFO("D");
                    // Switch between Position and Orientation modes
                    if (movement_mode == "Stationary")
                    {
                        movement_mode = "Moving";
                    }
                    else if (movement_mode == "Moving")
                    {
                        movement_mode = "Stationary";
                    }
                    dirty = true;
                    break;
            }

            geometry_msgs::Twist twist;
            twist.linear.x  = pos_scale*pos_x;
            twist.linear.y  = pos_scale*pos_y;
            twist.linear.z  = pos_scale*pos_z;
            twist.angular.x = rot_scale*rot_x;
            twist.angular.y = rot_scale*rot_y;
            twist.angular.z = rot_scale*rot_z;
            
            if(dirty == true)
            {
                hexapod_teleop::TeleopGoal goal;
                goal.movement_mode = movement_mode; // Stationary or Moving
                goal.twist = twist;
                L1_IK_client.sendGoal(goal);
                R1_IK_client.sendGoal(goal);
                L2_IK_client.sendGoal(goal);
                R2_IK_client.sendGoal(goal);
                L3_IK_client.sendGoal(goal);
                R3_IK_client.sendGoal(goal);

                dirty = false;
            }
        }

        return;
    }

private:
    std::string actionName;
    // ros::NodeHandle node;
    actionlib::SimpleActionClient<hexapod_teleop::TeleopAction> L1_IK_client;
    actionlib::SimpleActionClient<hexapod_teleop::TeleopAction> R1_IK_client;
    actionlib::SimpleActionClient<hexapod_teleop::TeleopAction> L2_IK_client;
    actionlib::SimpleActionClient<hexapod_teleop::TeleopAction> R2_IK_client;
    actionlib::SimpleActionClient<hexapod_teleop::TeleopAction> L3_IK_client;
    actionlib::SimpleActionClient<hexapod_teleop::TeleopAction> R3_IK_client;
    double pos_x, rot_x, pos_y, rot_y, pos_z, rot_z, pos_scale, rot_scale;
    std::string stationary_mode, movement_mode;
    double trans = 0.02;
    double rot_deg = 15;
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    ROS_INFO("Starting Teleop Action Server...");
    ros::init(argc, argv, "teleop_hexapod");
    ROS_INFO("Initialized ros...");

    ROS_INFO("IK Servers started, initializing client...");
    TeleopHexapod teleop_hexapod("teleop_controller");
    signal(SIGINT, quit);
    
    ROS_INFO("Sending teleop goal...");
    teleop_hexapod.keyLoop(kfd, cooked, raw);

    // ros::spin();

    return 0;
}