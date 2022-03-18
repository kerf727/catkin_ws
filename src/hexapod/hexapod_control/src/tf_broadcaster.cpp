#include "ros/ros.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_broadcaster.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "hexapod_tf_broadcaster");

    ros::NodeHandle node;
    tf2_ros::TransformBroadcaster L1broadcaster;
    tf2_ros::TransformBroadcaster L2broadcaster;
    tf2_ros::TransformBroadcaster L3broadcaster;
    tf2_ros::TransformBroadcaster R1broadcaster;
    tf2_ros::TransformBroadcaster R2broadcaster;
    tf2_ros::TransformBroadcaster R3broadcaster;
    
    ROS_INFO("Starting TF Broadcasters...");
    
    double tibia_length;
    node.getParam("/hexapod/geometry/tibia_length", tibia_length);

    ros::Rate rate(100); // TODO: set this to gazebo link_state update rate
    while(node.ok())
    {
        ros::Time current_time = ros::Time::now();

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = current_time;
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = tibia_length;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        transformStamped.header.frame_id = "robot::hexapod/L1_tibia_link";
        transformStamped.child_frame_id = "robot::hexapod/L1_foot_link";
        L1broadcaster.sendTransform(transformStamped);

        transformStamped.header.frame_id = "robot::hexapod/L2_tibia_link";
        transformStamped.child_frame_id = "robot::hexapod/L2_foot_link";
        L2broadcaster.sendTransform(transformStamped);

        transformStamped.header.frame_id = "robot::hexapod/L3_tibia_link";
        transformStamped.child_frame_id = "robot::hexapod/L3_foot_link";
        L3broadcaster.sendTransform(transformStamped);

        transformStamped.header.frame_id = "robot::hexapod/R1_tibia_link";
        transformStamped.child_frame_id = "robot::hexapod/R1_foot_link";
        R1broadcaster.sendTransform(transformStamped);

        transformStamped.header.frame_id = "robot::hexapod/R2_tibia_link";
        transformStamped.child_frame_id = "robot::hexapod/R2_foot_link";
        R2broadcaster.sendTransform(transformStamped);

        transformStamped.header.frame_id = "robot::hexapod/R3_tibia_link";
        transformStamped.child_frame_id = "robot::hexapod/R3_foot_link";
        R3broadcaster.sendTransform(transformStamped);
        
        rate.sleep();
    }
}