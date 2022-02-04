#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_RIGHT 0x43 
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_Q 0x71
// #define KEYCODE_R 0x64
// #define KEYCODE_L 0x61
// #define KEYCODE_U 0x77
// #define KEYCODE_D 0x73

class TeleopHexapod
{
public:
    TeleopHexapod();
    void keyLoop();

private:
    ros::NodeHandle node;
    ros::Publisher twistPublisher;
    double pos_x, rot_x, pos_y, rot_y, pos_z, rot_z, pos_scale, rot_scale;
    int mode;
    double trans = 0.02;
    double rot_deg = 15;
};

TeleopHexapod::TeleopHexapod():
    pos_x(0.0),
    rot_x(0.0),
    pos_y(0.0),
    rot_y(0.0),
    pos_z(0.0),
    rot_z(0.0),
    pos_scale(1.0),
    rot_scale(1.0),
    mode(0)
{
    node.param("scale_pos", pos_scale, pos_scale);
    node.param("scale_rot", rot_scale, rot_scale);

    twistPublisher = node.advertise<geometry_msgs::Twist>("/hexapod/teleop", 1);
}

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
    ros::init(argc, argv, "teleop_hexapod");
    TeleopHexapod teleop_hexapod;
    signal(SIGINT,quit);
    teleop_hexapod.keyLoop();
    return(0);
}

void TeleopHexapod::keyLoop()
{
    char c;
    bool dirty=false;

    // get the console in raw mode                                                              
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
        // get the next event from the keyboard  
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
                if (mode == 0) // Position
                {
                    pos_x = -trans;
                    pos_y =  0.0;
                    pos_z =  0.0;
                }
                else if (mode == 1) // Orientation
                {
                    rot_x =  0.0;
                    rot_y =  -rot_deg*M_PI/180;
                    rot_z =  0.0;
                }
                dirty = true;
                break;
            case KEYCODE_RIGHT:
                ROS_INFO("RIGHT");
                if (mode == 0) // Position
                {
                    pos_x =  trans;
                    pos_y =  0.0;
                    pos_z =  0.0;
                }
                else if (mode == 1) // Orientation
                {
                    rot_x =  0.0;
                    rot_y =  rot_deg*M_PI/180;
                    rot_z =  0.0;
                }
                dirty = true;
                break;
            case KEYCODE_UP:
                ROS_INFO("UP");
                if (mode == 0) // Position
                {
                    pos_x =  0.0;
                    pos_y =  trans;
                    pos_z =  0.0;
                }
                else if (mode == 1) // Orientation
                {
                    rot_x =  -rot_deg*M_PI/180;
                    rot_y =  0.0;
                    rot_z =  0.0;
                }
                dirty = true;
                break;
            case KEYCODE_DOWN:
                ROS_INFO("DOWN");
                if (mode == 0) // Position
                {
                    pos_x =  0.0;
                    pos_y = -trans;
                    pos_z =  0.0;
                }
                else if (mode == 1) // Orientation
                {
                    rot_x =  rot_deg*M_PI/180;
                    rot_y =  0.0;
                    rot_z =  0.0;
                }
                dirty = true;
                break;
            case KEYCODE_Q:
                ROS_INFO("Q");
                if (mode == 0)
                {
                    mode = 1;
                }
                else if (mode == 1)
                {
                    mode = 0;
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
            twistPublisher.publish(twist);    
            dirty=false;
        }
    }

    return;
}