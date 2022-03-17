// #include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "signal.h"
#include "termios.h"
#include "stdio.h"

#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_LEFT 0x44
#define KEYCODE_RIGHT 0x43 
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20

class TeleopHexapod
{
public:
    TeleopHexapod(std::string name)
    {
        ROS_INFO("Publishing to Gait Controller and Trajectory Action Servers...");
        this->gaitModePublisher = node.advertise<std_msgs::String>("/hexapod/gait/gait_mode", 1);
        this->baseTwistPublisher = node.advertise<geometry_msgs::Twist>("/hexapod/gait/base_twist", 1);
        this->speedPublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/speed", 1);
        this->yawPublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/yaw", 1);
        this->yawAnglePublisher = node.advertise<std_msgs::Float64>("/hexapod/gait/yaw_angle", 1);

        // Initialize published variables
        input_mode = "Moving";
        gait_mode = "Moving/Position";
        
        // Stationary mode variables
        base_pos.x = 0.0;
        base_pos.y = 0.0;
        base_pos.z = 0.0;
        base_rot.x = 0.0;
        base_rot.y = 0.0;
        base_rot.z = 0.0;

        // Stationary mode keyboard constants
        base_pos_delta = 0.02;
        base_rot_delta = 15 * M_PI / 180;

        // Moving mode variables
        speed = 0.0;
        yaw = 0.0;
        yaw_angle = 0.0;

        // Moving mode keyboard constants
        speed_delta = 0.04;
        yaw_delta = 10 * M_PI / 180;

        ROS_INFO("Teleop controller ready.\n");
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
            
            ROS_DEBUG("value: 0x%02X\n", c);

            switch(c)
            {
                case KEYCODE_W:
                    ROS_INFO("W");
                    if (input_mode == "Stationary")
                    {
                        base_pos.x = 0.0;
                        base_pos.y = base_pos_delta;
                        gait_mode = "Stationary/Position";
                    }
                    else if (input_mode == "Moving")
                    {
                        speed = speed_delta;
                        yaw = 0.0;
                        yaw_angle = 0;
                        gait_mode = "Moving/Position";
                    }
                    dirty = true;
                    break;
                
                case KEYCODE_A:
                    ROS_INFO("A");
                    if (input_mode == "Stationary")
                    {
                        base_pos.x = -base_pos_delta;
                        base_pos.y = 0.0;
                        gait_mode = "Stationary/Position";
                    }
                    else if (input_mode == "Moving")
                    {
                        speed = speed_delta;
                        yaw = 0.0;
                        yaw_angle = 270;
                        gait_mode = "Moving/Position";
                    }
                    dirty = true;
                    break;
                
                case KEYCODE_S:
                    ROS_INFO("S");
                    if (input_mode == "Stationary")
                    {
                        base_pos.x = 0.0;
                        base_pos.y = -base_pos_delta;
                        gait_mode = "Stationary/Position";
                    }
                    else if (input_mode == "Moving")
                    {
                        speed = speed_delta;
                        yaw = 0.0;
                        yaw_angle = 180;
                        gait_mode = "Moving/Position";
                    }
                    dirty = true;
                    break;

                case KEYCODE_D:
                    ROS_INFO("D");
                    if (input_mode == "Stationary")
                    {
                        base_pos.x = base_pos_delta;
                        base_pos.y = 0.0;
                        gait_mode = "Stationary/Position";
                    }
                    else if (input_mode == "Moving")
                    {
                        speed = speed_delta;
                        yaw = 0.0;
                        yaw_angle = 90;
                        gait_mode = "Moving/Position";
                    }
                    dirty = true;
                    break;

                case KEYCODE_UP:
                    ROS_INFO("UP");
                    if (input_mode == "Stationary")
                    {
                        base_rot.x = -base_rot_delta;
                        base_rot.y = 0.0;
                        gait_mode = "Stationary/Orientation";
                    }
                    dirty = true;
                    break;

                case KEYCODE_DOWN:
                    ROS_INFO("DOWN");
                    if (input_mode == "Stationary")
                    {
                        base_rot.x = base_rot_delta;
                        base_rot.y = 0.0;
                        gait_mode = "Stationary/Orientation";
                    }
                    dirty = true;
                    break;
                    
                case KEYCODE_LEFT:
                    ROS_INFO("LEFT");
                    if (input_mode == "Stationary")
                    {
                        base_rot.x = 0.0;
                        base_rot.y = -base_rot_delta;
                        gait_mode = "Stationary/Orientation";
                    }
                    else if (input_mode == "Moving")
                    {
                        speed = speed_delta;
                        yaw = yaw_delta;
                        gait_mode = "Moving/Orientation";
                    }
                    dirty = true;
                    break;

                case KEYCODE_RIGHT:
                    ROS_INFO("RIGHT");
                    if (input_mode == "Stationary")
                    {
                        base_rot.x = 0.0;
                        base_rot.y = base_rot_delta;
                        gait_mode = "Stationary/Orientation";
                    }
                    else if (input_mode == "Moving")
                    {
                        speed = speed_delta;
                        yaw = -yaw_delta;
                        gait_mode = "Moving/Orientation";
                    }
                    dirty = true;
                    break;

                case KEYCODE_SPACE:
                    ROS_INFO("SPACE");
                    // Switch between Stationary and Moving modes
                    if (input_mode == "Moving")
                    {
                        input_mode = "Stationary";
                    }
                    else if (input_mode == "Stationary")
                    {
                        input_mode = "Moving";
                    }
                    dirty = true;
                    break;
            }

            geometry_msgs::Twist base_twist;
            base_twist.linear.x  = base_pos.x;
            base_twist.linear.y  = base_pos.y;
            base_twist.linear.z  = base_pos.z;
            base_twist.angular.x = base_rot.x;
            base_twist.angular.y = base_rot.y;
            base_twist.angular.z = base_rot.z;
            
            if(dirty == true)
            {
                geometry_msgs::Twist baseTwistMsg;
                baseTwistMsg.linear = base_twist.linear;
                baseTwistMsg.angular = base_twist.angular;
                baseTwistPublisher.publish(baseTwistMsg);

                std_msgs::Float64 speedMsg;
                speedMsg.data = speed;
                speedPublisher.publish(speedMsg);

                std_msgs::Float64 yawMsg;
                yawMsg.data = yaw;
                yawPublisher.publish(yawMsg);

                std_msgs::Float64 yawAngleMsg;
                yawAngleMsg.data = yaw_angle;
                yawAnglePublisher.publish(yawAngleMsg);

                // Gait Controller callback triggered when gait_mode topic received
                // Publish this last so other variables are updated first
                std_msgs::String gaitModeMsg;
                gaitModeMsg.data = gait_mode;
                gaitModePublisher.publish(gaitModeMsg);

                dirty = false;
            }
        }

        return;
    }

private:
    ros::NodeHandle node;
    ros::Publisher gaitModePublisher;
    ros::Publisher baseTwistPublisher;
    ros::Publisher speedPublisher;
    ros::Publisher yawAnglePublisher;
    ros::Publisher yawPublisher;
    std::string input_mode, gait_mode;
    geometry_msgs::Vector3 base_pos, base_rot;
    double speed, yaw, speed_delta, yaw_delta, yaw_angle;
    double base_pos_delta, base_rot_delta, hex_rot_delta;
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
    TeleopHexapod teleop_hexapod("teleop_hexapod");
    signal(SIGINT, quit);
    
    ROS_INFO("Sending teleop goal...");
    teleop_hexapod.keyLoop(kfd, cooked, raw);

    return 0;
}