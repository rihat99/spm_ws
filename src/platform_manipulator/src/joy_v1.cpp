#include "ros/ros.h"
#include "math.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Joy.h"
#include "Eigen/Dense"

#define PI 3.14159265359
#define deg2rad 0.01745329251
#define rad2deg 57.2957795131

  geometry_msgs::Point j;
  geometry_msgs::Point c;
  geometry_msgs::Point a;
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  geometry_msgs::Point ik_p;
  geometry_msgs::Twist t1; 
  bool checkCol; 
  int rotate;
  int dont;
	
void subColCallback(const std_msgs::Bool::ConstPtr& msg)
{
  checkCol = msg->data;
}
	
void subJointsCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  c.x = msg->x;
  c.y = msg->y;
  c.z = msg->z;
  //ROS_INFO("Received: %f, %f, %f", c.x,c.y, c.z);
  
}

void subJoyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  a.x = msg->axes[0];
  a.y = msg->axes[1];
  a.z = 0;
  rotate = msg->buttons[1];
  dont = msg->buttons[2];
  
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "joy_v1");
  ros::NodeHandle n;
  
  // topics
  ros::Publisher joints2 = n.advertise<geometry_msgs::Point>("/ik_joints", 100);  
  ros::Publisher joints = n.advertise<geometry_msgs::Point>("/joints_sub", 100);  
    
  ros::Subscriber subJoints = n.subscribe("/joints_get", 100, subJointsCallback);
  ros::Subscriber jj = n.subscribe("/joy", 100, subJoyCallback);
  ros::Subscriber subCol = n.subscribe("/col", 100, subColCallback);
  
	
  // Design parameters
  float beta = 90;
  float gamma = 0;
  float alpha1 = 45;
  float alpha2 = 90;
  float alpha3 = 2*asin(sin(beta*deg2rad)*cos(PI/6));
  float eta [3];
  for (int i=0;i<3;i++) {
  	eta[i] = 2*i*180/3;
  }
  float step = 5.0;
  float rem = 0.0;
    ros::Rate loop_rate(20);
  ros::Rate loop_rate2(1);

  
while (ros::ok())
  {

	  // Input values
	  Eigen::Matrix <float, 1, 3> point = {0.71,0,0.71};
	  Eigen::Matrix <float, 1, 3> refline_fixed = {-1,0,0};
	  Eigen::Matrix <float, 1, 3> v1;
	  Eigen::Matrix <float, 1, 3> v2;
	  Eigen::Matrix <float, 1, 3> v3;
	  
	  float scale[3];
	  scale[0] = -a.x * sqrt(2)*8/20;  //x -> leftt/right
	  scale[1] = a.y * sqrt(2)*8/20;  //y -> up/down
	  scale[2] = sqrt(1 - scale[0]*scale[0] - scale[1]*scale[1]);
	   
	  for(int i = 0; i<3; i++){
	  	point[i] = scale[i];
	  }
	  
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
	  	
	  	A[i] = cos(eta[i]*deg2rad)*sin(gamma*deg2rad)*cos(alpha1*deg2rad)*vs(0,i)
	  		-cos(eta[i]*deg2rad)*cos(gamma*deg2rad)*sin(alpha1*deg2rad)*vs(0,i)
	  		+sin(eta[i]*deg2rad)*sin(gamma*deg2rad)*cos(alpha1*deg2rad)*vs(1,i)
	  		-sin(eta[i]*deg2rad)*cos(gamma*deg2rad)*sin(alpha1*deg2rad)*vs(1,i)
	  					-cos(gamma*deg2rad)*cos(alpha1*deg2rad)*vs(2,i)
	  					-sin(gamma*deg2rad)*sin(alpha1*deg2rad)*vs(2,i)
	  							-cos(alpha2*deg2rad);
	  							
	  	B[i] = sin(eta[i]*deg2rad)*sin(alpha1*deg2rad)*vs(0,i)
	  		-cos(eta[i]*deg2rad)*sin(alpha1*deg2rad)*vs(1,i);
	  		
	  	C[i] = cos(eta[i]*deg2rad)*sin(gamma*deg2rad)*cos(alpha1*deg2rad)*vs(0,i)
	  		+cos(eta[i]*deg2rad)*cos(gamma*deg2rad)*sin(alpha1*deg2rad)*vs(0,i)
	  		+sin(eta[i]*deg2rad)*sin(gamma*deg2rad)*cos(alpha1*deg2rad)*vs(1,i)
	  		+sin(eta[i]*deg2rad)*cos(gamma*deg2rad)*sin(alpha1*deg2rad)*vs(1,i)
	  					-cos(gamma*deg2rad)*cos(alpha1*deg2rad)*vs(2,i)
	  					+sin(gamma*deg2rad)*sin(alpha1*deg2rad)*vs(2,i)
	  							-cos(alpha2*deg2rad);
	  							
	  	T(1,i) = (-2*B(i)+sqrt(((2*B(i))*(2*B(i)))-4*A(i)*C(i)))/(2*A(i));
		T(2,i) = (-2*B(i)-sqrt(((2*B(i))*(2*B(i)))-4*A(i)*C(i)))/(2*A(i));
		
		theta(1,i) = 2*atan(T(1,i))*rad2deg;
		theta(2,i) = 2*atan(T(2,i))*rad2deg;
	  }
		
	  float angles [3];
	  for (int i=0;i<3;i++) {
	  	if (theta(1,i) != theta(1,i)) {
	  	angles[i] = 0;
	  	} else {
	  	angles[i] = theta(1,i);
	  	}
	}
	 	
	j.x = angles[0];
	j.y = angles[1];
	j.z = angles[2];
	
	p1.x = angles[0];
	p1.y = angles[1];
	p1.z = angles[2];
	
	int xx = abs(p1.x/step);
	int yy = abs(p1.y/step);
	int zz = abs(p1.z/step);
	int steps = xx;
	if (steps < yy) {
		steps = yy;
	}
	if (steps < zz) {
		steps = yy;
	}
	int i=1;
	
	for (int i = 1; i<steps+2; i++) {
	
		if (checkCol == true){
			ROS_INFO("Collision detected");
			break;
		} 
		
	  	if (i>steps+1){
	  		loop_rate2.sleep();
	  		break;
	  		
	  	}
		if (i <= xx) {
			p2.x = i*(p1.x/xx);
		}
		if (i <= yy) {
			p2.y = i*(p1.y/yy);
		}
		if (i <= zz) {
			p2.z = -i*(p1.z/zz);
		}
		 
		loop_rate.sleep();
		joints.publish(p2);
	}
	
    	ros::spinOnce();

  	//joints2.publish(j);
  	//ROS_INFO("C: %f, %f, %f", A[0],B[0], C[0]);
  	//ROS_INFO("point: %f, %f, %f", theta(2,0),theta(1,0), theta(2,2));
  	ROS_INFO("angles: %f, %f, %f", angles[0],angles[1], angles[2]);
  	//ros::Rate loop_rate(1);
	}
  return 0;
}

