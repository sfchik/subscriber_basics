#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <sstream>
#include <cstdint>
#include <pedsim_msgs/TrackedPersons.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h>

#define LOOP_RATE 5  //Hz
#define PI 		  3.1416
#define A_F 	  1.0	  
#define B_F       1.0
#define LAMDA     2.0
#define Nij 	  0.3448
#define R_B 	  0.15
#define R_P 	  0.3
#define R_T 	  0.45 //R_B + R_P

typedef union {
		uint16_t flag;
		struct {
			bool FIRST_TRACKED;
			bool FIRST_RB_POS;
			bool FIRST_RB_GOAL;
			unsigned unknown : 13;
		};
}flag_t; 

class PedsimMsgRemap
{
public:
	PedsimMsgRemap(const ros::NodeHandle &node) : nh_(node) 
	{
		//setup flags
		flag_.flag = 0;

		//setup subscriber
		sub_tracked_ped_ = nh_.subscribe("/pedsim/tracked_persons", 1, &PedsimMsgRemap::callbackTrackedPed, this);
		// sub_robot_pos_ = nh_.subscribe("pedsim/robot_position", 1, &PedsimMsgRemap::callbackRobotPos, this);		
		sub_robot_pos_ = nh_.subscribe("/robot_position/corrected", 1, &PedsimMsgRemap::callbackRobotPos, this);
		sub_robot_goal_ = nh_.subscribe("/move_base_simple/goal", 1, &PedsimMsgRemap::callbackRobotGoal, this);		

		//setup_publisher
		pub_people_ = nh_.advertise<people_msgs::People>("/people", 10);

		//fetch parameters
		nh_.param("/subscriber_pedsim_remap/vel_x", vel_x_, 1.4);
		nh_.param("/subscriber_pedsim_remap/vel_y", vel_y_, 0.0);
		nh_.param("/subscriber_pedsim_remap/pos_x", pos_x_, 4.0);
		nh_.param("/subscriber_pedsim_remap/pos_y", pos_y_, 4.0);
		nh_.param("/subscriber_pedsim_remap/detect_radius", detect_radius_, 8.0);				
	}

	void loop(void);
	people_msgs::People msgRemap(void);
	people_msgs::People msgRemapWithFakePeople(double px, double py, double vx, double vy);	
	people_msgs::People msgRemapWithAmplitude(void);
	double getDistance(double x1, double y1, double x2, double y2);
	bool pedTowardsGoal(double gx, double gy, double px, double py, double vel_x, double vel_y);
	double amplitudeScaler(double distanceRnP, bool towards_goal); 
	double quatToYaw(std_msgs::Header header, geometry_msgs::Pose pose, bool rad);

	//feature functions
	double socialForceRobotPed(double angle_ped, double angle_robot, double ped_vel_x, double ped_vel_y, double distanceRnP);
	double densityPed(double angle_ped, double angle_robot, double ped_vel_x, double ped_vel_y, double distanceRnP);

	//callbacks
	void callbackTrackedPed(const pedsim_msgs::TrackedPersons::ConstPtr& msg);
	void callbackRobotPos(const nav_msgs::Odometry::ConstPtr& msg);
	void callbackRobotGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);	

private:
	//ros handler
	ros::NodeHandle nh_;

	//subscriber
	ros::Subscriber sub_tracked_ped_;
	ros::Subscriber sub_robot_pos_;
	ros::Subscriber sub_robot_goal_;

	//publisher
	ros::Publisher pub_people_;

	//init flag
	flag_t flag_;

	//init msg
	pedsim_msgs::TrackedPersons msg_tracked_peds_;
	people_msgs::People msg_people_;
	nav_msgs::Odometry msg_robot_pos_;	
	geometry_msgs::PoseStamped msg_robot_goal_;

	//parameters
	double vel_x_, vel_y_;
	double pos_x_, pos_y_;
	double detect_radius_;
};

void PedsimMsgRemap::callbackTrackedPed(const pedsim_msgs::TrackedPersons::ConstPtr& msg)
{
	flag_.FIRST_TRACKED = true;
	msg_tracked_peds_ = *msg;
}

void PedsimMsgRemap::callbackRobotPos(const nav_msgs::Odometry::ConstPtr& msg)
{
	flag_.FIRST_RB_POS = true;
	msg_robot_pos_ = *msg;
}

void PedsimMsgRemap::callbackRobotGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	flag_.FIRST_RB_GOAL = true;
	msg_robot_goal_ = *msg;
}

people_msgs::People PedsimMsgRemap::msgRemap()
{
	std::string s = std::to_string(0);
	people_msgs::Person msg_person;
	people_msgs::People msg_people;

	msg_people.header = msg_tracked_peds_.header;
	for(std::vector<pedsim_msgs::TrackedPerson>::iterator it = msg_tracked_peds_.tracks.begin(); 
									it != msg_tracked_peds_.tracks.end(); ++it) {
		s = std::to_string(it->track_id);
		msg_person.name = s;
		msg_person.position = it->pose.pose.position;
		msg_person.velocity.x = it->twist.twist.linear.x;
		msg_person.velocity.y = it->twist.twist.linear.y;
		msg_person.velocity.z = it->twist.twist.linear.z;
		msg_people.people.push_back(msg_person);
	}

	
	return msg_people;
}

people_msgs::People PedsimMsgRemap::msgRemapWithFakePeople(double px, double py, double vx, double vy)
{
	std::string s = std::to_string(0);
	people_msgs::Person msg_person;
	people_msgs::People msg_people;

	msg_people.header = msg_tracked_peds_.header;
	for(std::vector<pedsim_msgs::TrackedPerson>::iterator it = msg_tracked_peds_.tracks.begin(); 
									it != msg_tracked_peds_.tracks.end(); ++it) {
		s = std::to_string(it->track_id);
		msg_person.name = s;
		msg_person.position.x = 6.0;
		msg_person.position.y = 6.0;		
		msg_person.velocity.x = 1.4;
		msg_person.velocity.y = 0.0;
		msg_person.velocity.z = 0.0;

		msg_people.people.push_back(msg_person);
	}
	return msg_people;
}

double PedsimMsgRemap::quatToYaw(std_msgs::Header header, geometry_msgs::Pose pose, bool rad=true)
{
	double yaw, dummy_pitch, dummy_roll;  //must be double, or else will have error
	tf::Stamped<tf::Pose> quat;
	geometry_msgs::PoseStamped msg_convert;

	msg_convert.header = header;
	msg_convert.pose = pose;
	poseStampedMsgToTF(msg_convert, quat);
	quat.getBasis().getEulerYPR(yaw, dummy_pitch, dummy_roll);

	return (rad)?yaw:180.0*fabs(yaw)/PI;	
}

//must be in DEGREE!!!
double PedsimMsgRemap::socialForceRobotPed(double angle_ped, double angle_robot, double ped_vel_x, double ped_vel_y, double distanceRnP)
{
	if(ped_vel_x > 0.0001 && ped_vel_y > 0.0001) {
		angle_ped += 90.0; 
	} else if(ped_vel_x < -0.0001 && ped_vel_y > 0.0001) {
		angle_ped += 90.0; 
	} else if(ped_vel_x < -0.0001 && ped_vel_y < -0.0001) {
		angle_ped = 450.0 - angle_ped;
	} else {
		angle_ped = 90 - angle_ped;
	}

	if(distanceRnP > 8.0) {
		distanceRnP = 8.0;
	}

	double f = A_F*exp((R_T - distanceRnP)/B_F)*Nij*9.0*
			(LAMDA + 0.5*(1-LAMDA)*(1 + cos(PI*(angle_robot - angle_ped)/180.0)));

	return (f < 1.0)?f:1.0;	
}

bool PedsimMsgRemap::pedTowardsGoal(double gx, double gy, double px, double py, double vel_x, double vel_y)
{
	double dis_current = getDistance(gx, gy, px, py);
	double dis_estimate = getDistance(gx, gy, px+vel_x, py+vel_y); //linear estimate
	return (dis_estimate < dis_current)?true:false;
}

people_msgs::People PedsimMsgRemap::msgRemapWithAmplitude()
{
	std::string s = std::to_string(0);
	people_msgs::Person msg_person;
	people_msgs::People msg_people;

	msg_people.header = msg_tracked_peds_.header;
	for(std::vector<pedsim_msgs::TrackedPerson>::iterator it = msg_tracked_peds_.tracks.begin(); 
									it != msg_tracked_peds_.tracks.end(); ++it) {

		double distanceRnP = getDistance(msg_robot_pos_.pose.pose.position.x, 
											msg_robot_pos_.pose.pose.position.y, 
											it->pose.pose.position.x, 
											it->pose.pose.position.y);

		if(distanceRnP > 8.0)
			continue;

		s = std::to_string(it->track_id);
		msg_person.name = s;
		msg_person.position = it->pose.pose.position;
		msg_person.velocity.x = it->twist.twist.linear.x;
		msg_person.velocity.y = it->twist.twist.linear.y;
		msg_person.velocity.z = it->twist.twist.linear.z;

		//amplitude process
		double yaw = quatToYaw(msg_tracked_peds_.header, it->pose.pose, false);
		double amplitude_scaler = socialForceRobotPed(yaw, msg_robot_pos_.pose.pose.position.z, 
									it->twist.twist.linear.x, it->twist.twist.linear.y, distanceRnP);
		// bool towards_goal = pedTowardsGoal(msg_robot_goal_.pose.position.x, msg_robot_goal_.pose.position.y,
		// 						 it->pose.pose.position.x, it->pose.pose.position.y,
		// 						 it->twist.twist.linear.x, it->twist.twist.linear.y);
		bool towards_goal = pedTowardsGoal(20.0, 4.0,
								 it->pose.pose.position.x, it->pose.pose.position.y,
								 it->twist.twist.linear.x, it->twist.twist.linear.y);

		//ROS_INFO("%f", amplitude_scaler);
		//ROS_INFO("%f %f %f %f", msg_robot_pos_.pose.pose.position.z, yaw_corrected, msg_robot_pos_.pose.pose.position.z - yaw_corrected, f);		

		//TO DO figure out a better way to pass the amplitude
		//msg_person.velocity.z = amplitudeScaler(distanceRnP, towards_goal);  //previous way to scale amplitude
		msg_person.velocity.z = amplitude_scaler;

		msg_people.people.push_back(msg_person);
	}
	return msg_people;
}

double PedsimMsgRemap::getDistance(double x1, double y1, double x2, double y2) 
{
	return std::hypot(x1-x2,y1-y2);
}

double PedsimMsgRemap::amplitudeScaler(double distanceRnP, bool towards_goal) 
{
	double scale = 0.0;

	if(distanceRnP > 8) {
		scale = 0.0;
	} else {
		scale = (8.0 - distanceRnP)/8.0;
	}

	scale = (towards_goal)?0.0:scale;

	return scale;
}

void PedsimMsgRemap::loop()
{
	ros::Rate r(LOOP_RATE);
	while(ros::ok()) {
		if(flag_.FIRST_TRACKED == true && flag_.FIRST_RB_POS == true) { //&&
			//flag_.FIRST_RB_GOAL == true) {
			msg_people_ = msgRemapWithAmplitude();
			//ROS_INFO("hello_world");
			//msg_people_ = msgRemapWithFakePeople(10.0, 10.0, 1.0, 0.1);
			pub_people_.publish(msg_people_);
		}
		ros::spinOnce();
		r.sleep();
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "subscriber_pedsim_remap");
	ros::NodeHandle n;
	PedsimMsgRemap pmr(n);

	pmr.loop();

}