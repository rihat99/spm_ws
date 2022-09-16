#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

#include "platform_manipulator_msgs/Joy.h"


double scale[3] = {0, 0, 0};
bool rotate;
bool rotate_inv;
ros::Publisher pub;


bool check_scale() {
    double length = std::sqrt(scale[0]*scale[0] + scale[1]*scale[1] + scale[2]*scale[2]);
    ROS_INFO("%f", length);

    return false;
}

void joy_callback(const sensor_msgs::Joy& msg) {
    // ROS_INFO("Left-right: %f, Up-down: %f", msg.axes[0], msg.axes[1]);
    scale[0] = -msg.axes[0] * std::sqrt(2) / 2; 
    scale[1] = msg.axes[1] * std::sqrt(2) / 2;
    scale[2] = std::sqrt(1 - scale[0]*scale[0] - scale[1]*scale[1]);

    rotate = msg.buttons[0];
    rotate_inv = msg.buttons[1];
    // check_scale();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "read_joy");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/joy", 1, joy_callback);
    pub = n.advertise<platform_manipulator_msgs::Joy>("platform_orientation", 1);

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        platform_manipulator_msgs::Joy new_msg;
        new_msg.joy.x = scale[0];
        new_msg.joy.y = scale[1];
        new_msg.joy.z = scale[2];
        new_msg.rotate = rotate;
        new_msg.rotate_inv = rotate_inv;
        pub.publish(new_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}