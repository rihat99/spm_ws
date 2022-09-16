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

double r_to_d(double angle) {
  return angle / M_PI * 180;
}
double d_to_r(double angle) {
  return angle /180 * M_PI;
}


void poseCallback2(const sensor_msgs::Imu& msg){

    tf2::Quaternion q;
    q.setX(msg.orientation.w);
    q.setY(msg.orientation.x);
    q.setZ(msg.orientation.y);
    q.setW(msg.orientation.z);
    

    tf2::Quaternion offset_q;
	offset_q.setRPY(0, 0, d_to_r(-30));

	q = offset_q * q * offset_q.inverse();


    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "platform_imu";

    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 1.2;

    transformStamped.transform.rotation.x = q.getX();
    transformStamped.transform.rotation.y = q.getY();
    transformStamped.transform.rotation.z = q.getZ();
    transformStamped.transform.rotation.w = q.getW();

    br.sendTransform(transformStamped);


    // tf2::Quaternion q;
    // q.setX(msg.x);
    // q.setY(msg.w);
    // q.setZ(-msg.y);
    // q.setW(msg.z);
}


int main(int argc, char** argv){

    ros::init(argc, argv, "imu_tf_broadcaster_wireless");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/wireless_imu/data", 1, poseCallback2);

    ros::Duration(1).sleep();

    ros::ServiceClient client = node.serviceClient<um7::Reset>("/wireless_imu/reset");
    um7::Reset srv;
    srv.request.reset_ekf = 1;
    srv.request.set_mag_ref = 1;
    srv.request.zero_gyros = 1;

    if (client.call(srv))
    {
        ROS_INFO("IMU reset success (wireless)");
    }
    else
    {
        ROS_ERROR("Failed to call IMU reset (wireless)");
    }

    ros::Rate loop_rate(20);
    while(ros::ok()) {
        ros::spinOnce();
		loop_rate.sleep();
    }

    // ros::spin();
    return 0;
}
