#include "ros/ros.h"
#include "platform_manipulator_msgs/Position.h"
// #include "platform_manipulator_msgs/ResetMotors.h"
// #include "dynamixel_workbench_msgs/DynamixelStateList.h"
// #include "dynamixel_workbench_msgs/DynamixelCommand.h"

#include "motor_controller.h"
#include <string>
#include <vector>

MotorController controller;

void set_position_callback(const platform_manipulator_msgs::Position& msg) {
	controller.setGoalPosition(msg.position);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "platform_motor_setter");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("constrained_motor_positions", 1, set_position_callback);

	int baud_rate;
	std::string port;

	std::vector<int> dxl_ids = {1, 2, 3};

	int velocity_profile;
	int acceleration_profile;

	n.param<std::string>("usb_port_motor", port, "/dev/ttyUSB0");
	n.param<int>("dxl_baud_rate", baud_rate, 1000000);
	n.param<int>("velocity_profile", velocity_profile, 100);
	n.param<int>("acceleration_profile", acceleration_profile, 50);



	controller.init(baud_rate, dxl_ids, port);
	controller.setProfileVelocity(velocity_profile);
	controller.setProfileAcceleration(acceleration_profile);

	ros::Duration(1).sleep();
	controller.enable_torque();
	
	ros::spin();

	controller.disable_torque();

	return 0;
}