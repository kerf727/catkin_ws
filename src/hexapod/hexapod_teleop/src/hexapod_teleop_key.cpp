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
        base_pos.x = 0.0;
        base_pos.y = 0.0;
        base_pos.z = 0.0;
        base_rot.x = 0.0;
        base_rot.y = 0.0;
        base_rot.z = 0.0;

        // update this to hexapod's current location?
        hex_pos.x = 0.0;
        hex_pos.y = 0.0;
        hex_pos.z = 0.05; // figure out what this should be for default height
        hex_rot = 0.0;

        base_pos_scale = 1.0;
        base_rot_scale = 1.0;
        hex_pos_scale = 1.0;
        hex_rot_scale = 1.0;

        base_pos_shift = 0.02;
        base_rot_deg = 15; // degrees
        hex_pos_delta = 0.04;
        hex_rot_delta = 10; // degrees

        stationary_mode = "Stationary/Position";

        // node.param("scale_pos", base_pos.scale, base_pos.scale);
        // node.param("scale_rot", base_rot.scale, base_rot.scale);

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
            
            // base_pos.x = base_rot.x = 0.0;
            // base_pos.y = base_rot.y = 0.0;
            // base_pos.z = base_rot.z = 0.0;

            ROS_DEBUG("value: 0x%02X\n", c);

            switch(c)
            {
                case KEYCODE_LEFT:
                    ROS_INFO("LEFT");
                    if (movement_mode == "Stationary")
                    {
                        if (stationary_mode == "Stationary/Position")
                        {
                            base_pos.x = -base_pos_shift;
                        }
                        else if (stationary_mode == "Stationary/Orientation")
                        {
                            base_rot.y = -base_rot_deg*M_PI/180;
                        }
                    }
                    else if (movement_mode == "Moving")
                    {
                        hex_pos.x -= hex_pos_delta;
                    }

                    dirty = true;
                    break;

                case KEYCODE_RIGHT:
                    ROS_INFO("RIGHT");
                    if (movement_mode == "Stationary")
                    {
                        if (stationary_mode == "Stationary/Position")
                        {
                            base_pos.x = base_pos_shift;
                        }
                        else if (stationary_mode == "Stationary/Orientation")
                        {
                            base_rot.y = base_rot_deg*M_PI/180;
                        }
                    }
                    else if (movement_mode == "Moving")
                    {
                        hex_pos.x += hex_pos_delta;
                    }

                    dirty = true;
                    break;

                case KEYCODE_UP:
                    ROS_INFO("UP");
                    if (movement_mode == "Stationary")
                    {
                        if (stationary_mode == "Stationary/Position")
                        {
                            base_pos.y = base_pos_shift;
                        }
                        else if (stationary_mode == "Stationary/Orientation")
                        {
                            base_rot.x = -base_rot_deg*M_PI/180;
                        }
                    }
                    else if (movement_mode == "Moving")
                    {
                        hex_pos.y += hex_pos_delta;
                    }

                    dirty = true;
                    break;

                case KEYCODE_DOWN:
                    ROS_INFO("DOWN");
                    if (movement_mode == "Stationary")
                    {
                        if (stationary_mode == "Stationary/Position")
                        {
                            base_pos.y = -base_pos_shift;
                        }
                        else if (stationary_mode == "Stationary/Orientation")
                        {
                            base_rot.x = base_rot_deg*M_PI/180;
                        }
                    }
                    else if (movement_mode == "Moving")
                    {
                        hex_pos.y -= hex_pos_delta;
                    }

                    dirty = true;
                    break;

                case KEYCODE_Q:
                    ROS_INFO("Q");
                    // Switch between Stationary Position and Stationary Orientation modes
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
                    // Switch between Stationary and Moving modes
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

            geometry_msgs::Twist base_twist;
            base_twist.linear.x  = base_pos_scale*base_pos.x;
            base_twist.linear.y  = base_pos_scale*base_pos.y;
            base_twist.linear.z  = base_pos_scale*base_pos.z;
            base_twist.angular.x = base_rot_scale*base_rot.x;
            base_twist.angular.y = base_rot_scale*base_rot.y;
            base_twist.angular.z = base_rot_scale*base_rot.z;

            geometry_msgs::Twist hex_twist;
            hex_twist.linear.x  = hex_pos_scale*hex_pos.x;
            hex_twist.linear.y  = hex_pos_scale*hex_pos.y;
            hex_twist.linear.z  = hex_pos_scale*hex_pos.z;
            hex_twist.angular.z = hex_rot_scale*hex_rot;
            
            if(dirty == true)
            {
                hexapod_teleop::TeleopGoal goal;
                goal.movement_mode = movement_mode; // Stationary or Moving
                goal.base_twist = base_twist;
                goal.hex_twist = hex_twist;
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

private:
    std::string actionName;
    // ros::NodeHandle node;
    actionlib::SimpleActionClient<hexapod_teleop::TeleopAction> L1_IK_client;
    actionlib::SimpleActionClient<hexapod_teleop::TeleopAction> R1_IK_client;
    actionlib::SimpleActionClient<hexapod_teleop::TeleopAction> L2_IK_client;
    actionlib::SimpleActionClient<hexapod_teleop::TeleopAction> R2_IK_client;
    actionlib::SimpleActionClient<hexapod_teleop::TeleopAction> L3_IK_client;
    actionlib::SimpleActionClient<hexapod_teleop::TeleopAction> R3_IK_client;
    double base_pos_scale, base_rot_scale, hex_pos_scale, hex_rot_scale;
    Vector3 base_pos, base_rot, hex_pos;
    double hex_rot;
    std::string movement_mode;
    double base_pos_shift, base_rot_deg, hex_pos_delta, hex_rot_delta;
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