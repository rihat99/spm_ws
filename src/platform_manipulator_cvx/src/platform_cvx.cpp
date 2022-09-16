#include "ros/ros.h"
#include "platform_manipulator_msgs/Position.h"

#include <vector>

#define deg2rad 0.01745329251
#define rad2deg 57.2957795131

extern "C" {
	#include "solver.h"

	void solve_for_theta(double, double, double);
	void load_default_data(void);
	void set_defaults(void);
}

Vars vars;
Params params;
Workspace work;
Settings settings;

std::vector<int> old_angles = {0, 0, 0};
int offset = 0;

int pos_mod(int n, int m) {
	if (n >= 0) return n%m;
	else return n%m + m;
}

ros::Publisher pub;

void pos_callback(const platform_manipulator_msgs::Position::ConstPtr& msg) {
	ROS_INFO("inv kinm angles: %i %i %i", msg->position[0], msg->position[1], msg->position[2]);
	std::vector<int> angles = {msg->position[0], msg->position[1], msg->position[2]};

	for (int i = 0; i < 3; ++i) {
		angles[i] = old_angles[i] + angles[i] - old_angles[i]%360;

		if (angles[i] - old_angles[i] > 350) {
			angles[i] -= 360;
		} else if (angles[i] - old_angles[i] < -350) {
			angles[i] += 360;
		}
	}
	old_angles = angles;

	int mean = (angles[0] + angles[1] + angles[2]) / 3;
	offset = 180 - mean;

	ROS_INFO("before angles: %i %i %i", angles[0], angles[1], angles[2]);

	solve_for_theta(angles[0] + offset, angles[1] + offset, angles[2] + offset);
	// angles = {vars.theta[0], vars.theta[1], vars.theta[2]};

	for (int i = 0; i < 3; ++i) {
		angles[i] = vars.theta[i];
		angles[i] -= offset;
	}


	ROS_INFO("after angles: %i %i %i", angles[0], angles[1], angles[2]);

	platform_manipulator_msgs::Position new_msg;
	new_msg.id = msg->id;
	new_msg.position = {angles[0], angles[1], angles[2]};
	pub.publish(new_msg);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "cvx");
	ros::NodeHandle n;

	set_defaults();
	load_default_data();

	pub = n.advertise<platform_manipulator_msgs::Position>("constrained_motor_positions", 1);
	ros::Subscriber subPositions = n.subscribe("/calculated_motor_positions", 1, pos_callback);
	
	ros::spin();

	return 0;
}
