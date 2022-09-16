#include "ros/ros.h"
#include "math.h"
#include <vector>
#include "geometry_msgs/Vector3.h"
#include "Eigen/Dense"
#include "platform_manipulator_msgs/Position.h"

#define PI 3.14159265359
#define deg2rad 0.01745329251
#define rad2deg 57.2957795131


// bool checkCol; 
// int rotate;
// int dont;


// Design parameters
float beta = 90;
float gamma_ = 0;
float alpha1 = 45;
float alpha2 = 90;
float alpha3 = 2*asin(sin(beta*deg2rad)*cos(PI/6));
float eta [3];

float step = 5.0;
float rem = 0.0;


std::vector<float> inverse_kinematics(Eigen::Matrix <float, 1, 3> point) {
    Eigen::Matrix <float, 1, 3> refline_fixed = {-1,0,0};
    Eigen::Matrix <float, 1, 3> v1;
    Eigen::Matrix <float, 1, 3> v2;
    Eigen::Matrix <float, 1, 3> v3;
    
    
    v1 = point.cross(refline_fixed.transpose());
    v1 = v1/sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2]);

    v1 = cos(1*deg2rad)*v1+sin(1*deg2rad)*point.cross(v1);
    v1 = v1/sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2]);
    
    v2 = cos(120*deg2rad)*v1+sin(120*deg2rad)*point.cross(v1);
    v2 = v2/sqrt(v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2]);
    
    v3 = cos(240*deg2rad)*v1+sin(240*deg2rad)*point.cross(v1);
    v3 = v3/sqrt(v3[0]*v3[0]+v3[1]*v3[1]+v3[2]*v3[2]);

    Eigen::Matrix3f  vs;
    vs.col(0) = v1.transpose();
    vs.col(1) = v2.transpose();
    vs.col(2) = v3.transpose();
    
    //IK 
    
    Eigen::Matrix< float, 1, 3>  A = Eigen::Matrix< float, 1, 3>::Zero();
    Eigen::Matrix< float, 1, 3>  B = Eigen::Matrix< float, 1, 3>::Zero();
    Eigen::Matrix< float, 1, 3>  C = Eigen::Matrix< float, 1, 3>::Zero();
    Eigen::Matrix< float, 2, 3>  T = Eigen::Matrix< float, 2, 3>::Zero();
    Eigen::Matrix< float, 2, 3>  theta = Eigen::Matrix< float, 2, 3>::Zero();
	  
	for(int i = 0; i<3; i++) {
	  	
	  	A[i] = cos(eta[i]*deg2rad)*sin(gamma_*deg2rad)*cos(alpha1*deg2rad)*vs(0,i)
	  		-cos(eta[i]*deg2rad)*cos(gamma_*deg2rad)*sin(alpha1*deg2rad)*vs(0,i)
	  		+sin(eta[i]*deg2rad)*sin(gamma_*deg2rad)*cos(alpha1*deg2rad)*vs(1,i)
	  		-sin(eta[i]*deg2rad)*cos(gamma_*deg2rad)*sin(alpha1*deg2rad)*vs(1,i)
	  					-cos(gamma_*deg2rad)*cos(alpha1*deg2rad)*vs(2,i)
	  					-sin(gamma_*deg2rad)*sin(alpha1*deg2rad)*vs(2,i)
	  							-cos(alpha2*deg2rad);
	  							
	  	B[i] = sin(eta[i]*deg2rad)*sin(alpha1*deg2rad)*vs(0,i)
	  		-cos(eta[i]*deg2rad)*sin(alpha1*deg2rad)*vs(1,i);
	  		
	  	C[i] = cos(eta[i]*deg2rad)*sin(gamma_*deg2rad)*cos(alpha1*deg2rad)*vs(0,i)
	  		+cos(eta[i]*deg2rad)*cos(gamma_*deg2rad)*sin(alpha1*deg2rad)*vs(0,i)
	  		+sin(eta[i]*deg2rad)*sin(gamma_*deg2rad)*cos(alpha1*deg2rad)*vs(1,i)
	  		+sin(eta[i]*deg2rad)*cos(gamma_*deg2rad)*sin(alpha1*deg2rad)*vs(1,i)
	  					-cos(gamma_*deg2rad)*cos(alpha1*deg2rad)*vs(2,i)
	  					+sin(gamma_*deg2rad)*sin(alpha1*deg2rad)*vs(2,i)
	  							-cos(alpha2*deg2rad);
	  							
	  	T(0,i) = (-2*B(i)+sqrt(((2*B(i))*(2*B(i)))-4*A(i)*C(i)))/(2*A(i));
		T(1,i) = (-2*B(i)-sqrt(((2*B(i))*(2*B(i)))-4*A(i)*C(i)))/(2*A(i));
		
		theta(0,i) = 2*atan(T(0,i))*rad2deg;
		theta(1,i) = 2*atan(T(1,i))*rad2deg;
	}
		
	std::vector<float> angles(3);
    for (int i=0;i<3;i++) {
        if (theta(0,i) != theta(0,i)) {
        angles[i] = 0;
        } else {
        angles[i] = theta(0,i);
        }
	}   

    return angles;
    
}

std::vector<double> check_n(std::vector<double> n) {
	if (n[2] < sin(45*deg2rad)) {
		n[0] = sin(45*deg2rad) * n[0] / std::sqrt(1 - std::pow(n[2], 2));
		n[1] = sin(45*deg2rad) * n[1] / std::sqrt(1 - std::pow(n[2], 2));
		n[2] = sin(45*deg2rad);
	}
	return n;
}

ros::Publisher pub;

void orientation_callback(const geometry_msgs::Vector3& msg) {
	std::vector<double> n = {msg.x, msg.y, msg.z};
	n = check_n(n);
	ROS_INFO("%f %f %f", n[0], n[1], n[2]);
    Eigen::Matrix <float, 1, 3> point = {n[0], n[1], n[2]};
	std::vector<float> angles = inverse_kinematics(point);

	platform_manipulator_msgs::Position new_msg;
	new_msg.id = {1, 2, 3};
	new_msg.position = {int(angles[0]), int(angles[1]), int(angles[2])};
	pub.publish(new_msg);
}


int main(int argc, char **argv) {
    
    for (int i=0;i<3;i++) {
        eta[i] = 2*i*180/3;
    }

	ros::init(argc, argv, "platform_inverse_kinematics");
	ros::NodeHandle n;
	
	pub = n.advertise<platform_manipulator_msgs::Position>("motor_positions", 100);
	ros::Subscriber sub = n.subscribe("platform_orientation", 1000, orientation_callback);

	ros::spin();



    return 0;
}