#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>


double r_to_d(double angle) {
  return angle / M_PI * 180;
}
double d_to_r(double angle) {
  return angle /180 * M_PI;
}
double asind(double value) {
  return r_to_d(std::asin(value));
}
double acosd(double value) {
  return r_to_d(std::acos(value));
}
double sind(double angle) {
  return std::sin(d_to_r(angle));
}
double cosd(double angle) {
    return std::cos(d_to_r(angle));
}
double atand(double value) {
	return r_to_d(std::atan(value));
}
double atand2(double value, double value2) {
	return r_to_d(std::atan2(value, value2));
}

double scale[2] = {90, 90};

int speed = 4;
double increment_deg = 0;
int rotation_speed;
ros::Publisher pub;


void joy_callback(const sensor_msgs::Joy& msg) {
    // ROS_INFO("Left-right: %f, Up-down: %f", msg.axes[0], msg.axes[1]);
    scale[0] += speed * msg.axes[0]; 
    scale[1] -= speed * msg.axes[1];

    for (int i = 0; i < 2; ++i) {
        // scale[i] += msg.axes[i];
        if (scale[i] < 45) scale[i] = 45;
        else if (scale[i] > 135) scale[i] = 135;
    }
    if (std::pow(cosd(scale[0]), 2) + std::pow(cosd(scale[1]), 2) > 0.5) {
        double temp_0 = acosd(sind(45) * cosd(scale[0]) / std::sqrt(std::pow(cosd(scale[0]), 2) + std::pow(cosd(scale[1]), 2)));
        double temp_1 = acosd(sind(45) * cosd(scale[1]) / std::sqrt(std::pow(cosd(scale[0]), 2) + std::pow(cosd(scale[1]), 2)));
      	scale[0] = temp_0;
      	scale[1] = temp_1;
    }

    if (msg.buttons[0]) increment_deg += rotation_speed;
    if (msg.buttons[1]) increment_deg -= rotation_speed;
    if (msg.buttons[2]) {
		increment_deg = 0;
		scale[0] = 90;
		scale[1] = 90;
	}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "read_joy");
    ros::NodeHandle n;

    int control_frequency;

    n.param<int>("control_frequency", control_frequency, 30);
    n.param<int>("joystick_speed", speed, 4);
    n.param<int>("rotation_speed", rotation_speed, 4);

    ros::Subscriber sub = n.subscribe("/joy", 1, joy_callback);
    pub = n.advertise<geometry_msgs::Quaternion>("platform_orientation", 1);

    ros::Rate loop_rate(control_frequency);

    while(ros::ok()) {
        geometry_msgs::Quaternion new_msg;

		tf2::Quaternion q;
		q.setRPY(d_to_r(scale[1] - 90), d_to_r(90 - scale[0]), 0);

		tf2::Quaternion rotate_q;
		rotate_q.setRPY(0, 0, d_to_r(increment_deg));
		q = q * rotate_q;

		new_msg.x = -q.getY();
		new_msg.y = q.getX();
		new_msg.z = q.getZ();
		new_msg.w = q.getW();

        pub.publish(new_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}