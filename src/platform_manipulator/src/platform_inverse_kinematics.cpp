#define _USE_MATH_DEFINES

#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "platform_manipulator_msgs/Position.h"
#include "geometry_msgs/Vector3.h"
#include "platform_manipulator_msgs/Joy.h"
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
double sind(double angle) {
  return std::sin(d_to_r(angle));
}
double cosd(double angle) {
    return std::cos(d_to_r(angle));
}
double atand(double value) {
	return r_to_d(std::atan(value));
}
std::vector<double> cross(std::vector<double> &u, std::vector<double> &v) {
	std::vector<double> w(3);
	w[0] = u[1] * v[2] - u[2] * v[1];
	w[1] = u[2] * v[0] - u[0] * v[2];
	w[2] = u[0] * v[1] - u[1] * v[0];
	return w;
}
std::vector<double> normalize(std::vector<double> v) {
	double length = std::sqrt(std::pow(v[0], 2) + std::pow(v[1], 2) + std::pow(v[2], 2));
	v[0] /= length; v[1] /= length; v[2] /= length;
	return v;
}
std::vector<double> scalar_mult(double num, std::vector<double> vec) {
	vec[0] *= num; vec[1] *= num; vec[2] *= num;
	return vec;
}
std::vector<double> add_vectors(std::vector<double> a, std::vector<double> b) {
	std::vector<double> c(3);
	c[0] = a[0] + b[0];
	c[1] = a[1] + b[1];
	c[2] = a[2] + b[2];
	return c;
}

double beta = 90;
double gamma_ = 0;
double alpha1 = 45;
double alpha2 = 90;
double alpha3 = 2 * asind(sind(beta) * std::cos(M_PI/6));
int eta[] = {0, 2*1*180/3, 2*2*180/3};

std::vector<double> centerline = {0, 1, 0};

tf2::Quaternion base_q;


std::vector<double> inverse_kinematics(std::vector<std::vector<double>>& v) {
	std::vector<double> angles(3);

	std::vector<double> A(3, 0);
	std::vector<double> B(3, 0);
	std::vector<double> C(3, 0);

	std::vector<std::vector<double>> T(2, std::vector<double>(3, 0));
	std::vector<std::vector<double>> theta(2, std::vector<double>(3, 0));

  	for (int i = 0; i < 3; ++i) {
      	A[i] =  cosd(eta[i])*sind(gamma_)*cosd(alpha1)*v[0][i]
               -cosd(eta[i])*cosd(gamma_)*sind(alpha1)*v[0][i]
               +sind(eta[i])*sind(gamma_)*cosd(alpha1)*v[1][i]
               -sind(eta[i])*cosd(gamma_)*sind(alpha1)*v[1][i]
               				-cosd(gamma_)*cosd(alpha1)*v[2][i]
               				-sind(gamma_)*sind(alpha1)*v[2][i]
               									 -cosd(alpha2);

		B[i] =  sind(eta[i])*sind(alpha1)*v[0][i]
               -cosd(eta[i])*sind(alpha1)*v[1][i];

		C[i] =  cosd(eta[i])*sind(gamma_)*cosd(alpha1)*v[0][i]
               +cosd(eta[i])*cosd(gamma_)*sind(alpha1)*v[0][i]
               +sind(eta[i])*sind(gamma_)*cosd(alpha1)*v[1][i]
               +sind(eta[i])*cosd(gamma_)*sind(alpha1)*v[1][i]
                            -cosd(gamma_)*cosd(alpha1)*v[2][i]
                            +sind(gamma_)*sind(alpha1)*v[2][i]
                                                -cosd(alpha2);
		T[0][i] = (-2*B[i]+std::sqrt((std::pow(2*B[i],2))-4*A[i]*C[i]))/(2*A[i]);
		T[1][i] = (-2*B[i]-std::sqrt((std::pow(2*B[i],2))-4*A[i]*C[i]))/(2*A[i]);

		theta[0][i] = 2 * atand(T[0][i]);
		theta[1][i] = 2 * atand(T[1][i]);
  	}

	angles[0] = theta[0][0];
	angles[1] = theta[0][1];
	angles[2] = theta[0][2];

	for (int k = 0; k < 3; ++k) {
		if (std::isnan(angles[k])) {
			angles[k] = 0;
		}
	}

	return angles;
}

std::vector<std::vector<double>> find_v(std::vector<double>& n) {
	
	std::vector<double> v1 = cross(n, centerline);
	// v1 = normalize(v1);
	// v1 = add_vectors(scalar_mult(cosd(increment_deg), v1), scalar_mult(sind(increment_deg), cross(n, v1)));
	v1 = normalize(v1);

	std::vector<double> v2 = add_vectors(scalar_mult(cosd(120), v1), scalar_mult(sind(120), cross(n, v1)));
	v2 = normalize(v2);

	std::vector<double> v3 = add_vectors(scalar_mult(cosd(240), v1), scalar_mult(sind(240), cross(n, v1)));
	v3 = normalize(v3);

	std::vector<std::vector<double>> v(3, std::vector<double>(3));
	v[0][0] = v1[0]; v[1][0] = v1[1]; v[2][0] = v1[2];
	v[0][1] = v2[0]; v[1][1] = v2[1]; v[2][1] = v2[2];
	v[0][2] = v3[0]; v[1][2] = v3[1]; v[2][2] = v3[2];

	return v;

}

std::vector<double> check_n(std::vector<double> n) {
	if (n[2] <  cosd(42)) {
		n[0] = sind(42) * n[0] / std::sqrt(1 - std::pow(n[2], 2));
		n[1] = sind(42) * n[1] / std::sqrt(1 - std::pow(n[2], 2));
		n[2] = cosd(42);
	}
	return n;
}

ros::Publisher pub;
ros::Publisher pub_q;

void platform_callback(const geometry_msgs::Quaternion& msg) {
	tf2::Quaternion plat_q;
    plat_q.setX(msg.x);
    plat_q.setY(msg.y);
    plat_q.setZ(msg.z);
    plat_q.setW(msg.w);

	tf2::Quaternion q = base_q.inverse() * plat_q;
	// tf2::Quaternion q = plat_q;

    tf2::Matrix3x3 m(q);
    tf2::Vector3 axis = m.getColumn(2);
	tf2::Vector3 axis_2 = m.getColumn(1);
	std::vector<double> n = {axis.getX(), axis.getY(), axis.getZ()};
	centerline[0] = axis_2.getX(); centerline[1] = axis_2.getY(); centerline[2] = axis_2.getZ(); 

	// n = check_n(n);
	ROS_INFO("%f %f %f ::: %f", n[0], n[1], n[2], 90-asind(n[2]));
	std::vector<std::vector<double>> v = find_v(n);
	std::vector<double> angles = inverse_kinematics(v);

	platform_manipulator_msgs::Position new_msg;
	new_msg.id = {1, 2, 3};
	new_msg.position = {int(angles[0]), int(angles[1]), int(angles[2])};
	pub.publish(new_msg);

	geometry_msgs::Quaternion new_msg_q;
	new_msg_q.x = q.getX();
	new_msg_q.y = q.getY();
	new_msg_q.z = q.getZ();
	new_msg_q.w = q.getW();

	pub_q.publish(new_msg_q);
}

void base_callback(const geometry_msgs::Quaternion& msg) {
	base_q.setX(msg.x);
	base_q.setY(msg.y);
	base_q.setZ(msg.z);
	base_q.setW(msg.w);
}


int main(int argc, char **argv) {
    
	ros::init(argc, argv, "platform_inverse_kinematics");
	ros::NodeHandle n;
	
	pub = n.advertise<platform_manipulator_msgs::Position>("calculated_motor_positions", 1);
	pub_q = n.advertise<geometry_msgs::Quaternion>("platform_compensated_orientation", 1);
	ros::Subscriber sub_platform = n.subscribe("platform_orientation", 1, platform_callback);
	ros::Subscriber sub_base = n.subscribe("base_orientation", 1, base_callback);

	ros::spin();

    return 0;
}