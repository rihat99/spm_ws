#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


void poseCallback2(const geometry_msgs::Quaternion& msg){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "platform";

    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 1.0;

    transformStamped.transform.rotation.x = msg.x;
    transformStamped.transform.rotation.y = msg.y;
    transformStamped.transform.rotation.z = msg.z;
    transformStamped.transform.rotation.w = msg.w;

    br.sendTransform(transformStamped);
    
}

int main(int argc, char** argv){

    ros::init(argc, argv, "platform_tf_broadcaster");
    ros::NodeHandle node;

    int control_frequency;
    node.param<int>("control_frequency", control_frequency, 30);

    ros::Subscriber sub = node.subscribe("/platform_orientation", 1, poseCallback2);

    ros::Duration(1).sleep();


    ros::Rate loop_rate(control_frequency);
    while(ros::ok()) {
        ros::spinOnce();
		loop_rate.sleep();
    }

    // ros::spin();
    return 0;
}
