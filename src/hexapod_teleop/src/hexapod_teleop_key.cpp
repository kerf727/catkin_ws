#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class TeleopHexapod
{
public:
    TeleopHexapod();
    void keyLoop();

private:
    ros::NodeHandle nh_;
    ros::Publisher twist_pub_;
    double pos_x, rot_x, pos_y, rot_y, pos_z, rot_z, p_scale, r_scale;
};

TeleopHexapod::TeleopHexapod():
    pos_x(0),
    rot_x(0),
    pos_y(0),
    rot_y(0),
    pos_z(0),
    rot_z(0),
    p_scale(1.0),
    r_scale(1.0)
{
    nh_.param("scale_pos", p_scale, p_scale);
    nh_.param("scale_rot", r_scale, r_scale);

    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/hexapod/teleop", 1);
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

        pos_x = rot_x = 0;
        pos_y = rot_y = 0;
        pos_z = rot_z = 0;
        ROS_DEBUG("value: 0x%02X\n", c);

        switch(c)
        {
            case KEYCODE_L:
            ROS_DEBUG("LEFT");
            pos_x = -0.2;
            dirty = true;
            break;
            case KEYCODE_R:
            ROS_DEBUG("RIGHT");
            pos_x = 0.2;
            dirty = true;
            break;
            case KEYCODE_U:
            ROS_DEBUG("UP");
            pos_y = 0.2;
            dirty = true;
            break;
            case KEYCODE_D:
            ROS_DEBUG("DOWN");
            pos_y = -0.2;
            dirty = true;
            break;
        }

        geometry_msgs::Twist twist;
        twist.linear.x = p_scale*pos_x;
        twist.linear.y = p_scale*pos_x;
        twist.linear.z = p_scale*pos_x;
        twist.angular.x = r_scale*rot_x;
        twist.angular.y = r_scale*rot_y;
        twist.angular.z = r_scale*rot_z;
        
        if(dirty == true)
        {
            twist_pub_.publish(twist);    
            dirty=false;
        }
    }

    return;
}