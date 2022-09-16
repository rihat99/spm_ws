#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <um7/Reset.h>

ros::Publisher pub;



void poseCallback2(const sensor_msgs::Imu& msg){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "base";

    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;

    transformStamped.transform.rotation.x = msg.orientation.w;
    transformStamped.transform.rotation.y = msg.orientation.x;
    transformStamped.transform.rotation.z = msg.orientation.y;
    transformStamped.transform.rotation.w = msg.orientation.z;

    br.sendTransform(transformStamped);


    tf2::Quaternion q;
    q.setX(msg.orientation.w);
    q.setY(msg.orientation.x);
    q.setZ(msg.orientation.y);
    q.setW(msg.orientation.z);

    geometry_msgs::Quaternion new_msg;

    new_msg.x = q.getX();
    new_msg.y = q.getY();
    new_msg.z = q.getZ();
    new_msg.w = q.getW();

    pub.publish(new_msg);

    // ros::Duration(0.1).sleep();
    
}

int main(int argc, char** argv){

    ros::init(argc, argv, "imu_tf_broadcaster");
    ros::NodeHandle node;

    int control_frequency;
    node.param<int>("control_frequency", control_frequency, 30);

    ros::Subscriber sub = node.subscribe("/wired_imu/data", 1, poseCallback2);

    pub = node.advertise<geometry_msgs::Quaternion>("base_orientation", 1);

    ros::Duration(1).sleep();

    ros::ServiceClient client = node.serviceClient<um7::Reset>("/wired_imu/reset");
    um7::Reset srv;
    srv.request.reset_ekf = 1;
    srv.request.set_mag_ref = 1;
    srv.request.zero_gyros = 1;

    if (client.call(srv))
    {
        ROS_INFO("IMU reset success (wired)");
    }
    else
    {
        ROS_ERROR("Failed to call IMU reset (wired)");
    }

    ros::Rate loop_rate(control_frequency);
    while(ros::ok()) {
        ros::spinOnce();
		loop_rate.sleep();
    }

    // ros::spin();
    return 0;
}
