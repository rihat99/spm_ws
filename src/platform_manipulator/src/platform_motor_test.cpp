#include "ros/ros.h"
#include "platform_manipulator_msgs/Position.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

#include <vector>



int main(int argc, char **argv) {
	ros::init(argc, argv, "plarform_motor_test");
	ros::NodeHandle n;

	ros::Publisher motor_pub = n.advertise<platform_manipulator_msgs::Position>("constrained_motor_positions", 1000);

	ros::Rate loop_rate(1);
	int position[3] = {0, 0, 0};

	platform_manipulator_msgs::Position msg;
    msg.id = {1, 2, 3};
    msg.position = {position[0], position[1], position[2]};
    motor_pub.publish(msg);

	ros::Duration(4).sleep();

    while (ros::ok()) {
		position[0] += 100;
		position[1] -= 50;
		position[2] += 25;

		platform_manipulator_msgs::Position msg;
		msg.id = {1, 2, 3};
		msg.position = {position[0], position[1], position[2]};
		motor_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
    }


    return 0;
}