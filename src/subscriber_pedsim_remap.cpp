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
#include <nav_msgs/Path.h>
#include <std_msgs/Int16.h>

#define LOOP_RATE 10  //Hz
#define LOOP_RATE_FACTOR 1
#define PI 		  3.1416
#define A_F 	  1.0	  
#define B_F       1.0
#define LAMDA     2.0
#define Nij 	  0.3448
#define R_B 	  0.15
#define R_P 	  0.3
#define R_T 	  0.45 //R_B + R_P

//Social Proxemics Limit
#define INITIMATE_SPACE 		0.45
#define PERSONAL_SPACE_CLOSE 	0.76
#define PERSONAL_SPACE 			1.22
#define SOCIAL_SPACE 			3.0
#define PUBLIC_SPACE 			3.0

#define INITIMATE_TAG 		1
#define PERSONAL_CLOSE_TAG 	2
#define PERSONAL_TAG 		3
#define SOCIAL_TAG 			4
#define PUBLIC_TAG 			5
#define INIT_TAG   			6

typedef union {
		uint16_t flag;
		struct {
			bool FIRST_TRACKED;
			bool FIRST_RB_POS;
			bool FIRST_RB_GOAL;
			bool FIRST_VARY_GAUSS;   //can be removed after experiment is done
			bool FIRST_RB_PLAN;
			unsigned unknown : 11;
		};
}flag_t; 

enum class gaussian : int {NORMAL = 0, SOCIAL};

class PedsimMsgRemap
{
public:
	PedsimMsgRemap(const ros::NodeHandle &node) : nh_(node) 
	{
		//init metrics that requires initial comparison
		density_ = 0.0; 
		closest_dis_ = 100.0;
		y_hat_ = 0.0;

		//setup flags
		flag_.flag = 0;

		//fetch parameters
		nh_.param("/subscriber_pedsim_remap/vel_x", vel_x_, 1.4);
		nh_.param("/subscriber_pedsim_remap/vel_y", vel_y_, 0.0);
		nh_.param("/subscriber_pedsim_remap/pos_x", pos_x_, 4.0);
		nh_.param("/subscriber_pedsim_remap/pos_y", pos_y_, 4.0);
		nh_.param("/subscriber_pedsim_remap/detect_radius", detect_radius_, 8.0);	
		nh_.param("/subscriber_pedsim_remap/robot_radius", robot_radius_, 0.25);			
		nh_.param("/subscriber_pedsim_remap/social_gaussian", social_gaussian_, true);	
		nh_.param("/subscriber_pedsim_remap/robot_pos_topic", robot_pos_topic_, std::string("/pedsim/robot_position"));	
		nh_.param("/subscriber_pedsim_remap/person_track_topic", person_track_topic_, std::string("/pedsim/tracked_persons"));		

		//setup subscriber
		sub_tracked_ped_ = nh_.subscribe(person_track_topic_, 1, &PedsimMsgRemap::callbackTrackedPed, this);
		// sub_robot_pos_ = nh_.subscribe("pedsim/robot_position", 1, &PedsimMsgRemap::callbackRobotPos, this);		
		sub_robot_pos_ = nh_.subscribe(robot_pos_topic_, 1, &PedsimMsgRemap::callbackRobotPos, this);
		sub_robot_goal_ = nh_.subscribe("/move_base_simple/goal", 1, &PedsimMsgRemap::callbackRobotGoal, this);		
		sub_robot_plan_ = nh_.subscribe("/move_base_node/NavfnROS/plan", 1, &PedsimMsgRemap::callbackRobotPlan, this);	
		//setup_publisher
		pub_people_ = nh_.advertise<people_msgs::People>("/people", 10);		
	}

	void loop(void);
	people_msgs::People msgRemap(void);
	people_msgs::People msgRemapWithFakePeople(double px, double py, double vx, double vy);	
	people_msgs::People msgRemapWithAmplitude(void);
	double getDistance(double x1, double y1, double x2, double y2);
	bool pedTowardsGoal(double gx, double gy, double px, double py, double vel_x, double vel_y);
	double amplitudeScaler(double distanceRnP, bool towards_goal); 
	double quatToYaw(std_msgs::Header header, geometry_msgs::Pose pose, bool rad);
	double anisotropicFactorPed(double rx, double ry, double px, double py, double pvx, double pvy, double lamda);
	double anisotropicFactorRB(double rx, double ry, double px, double py, double rvx, double rvy, double lamda);	
	double circularSpecification(double C, double density, double sum_radius, double distanceRnP);
	void dynamicConfig(std::string node, std::string parameter, std::string value);	
	double planLengthCalc(nav_msgs::Path plan);

	//feature functions
	double socialForceRobotPed(double angle_ped, double angle_robot, double ped_vel_x, double ped_vel_y, double distanceRnP);
	double densityPed(double angle_ped, double angle_robot, double ped_vel_x, double ped_vel_y, double distanceRnP);

	//callbacks
	void callbackTrackedPed(const pedsim_msgs::TrackedPersons::ConstPtr& msg);
	void callbackRobotPos(const nav_msgs::Odometry::ConstPtr& msg);
	void callbackRobotGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);	
	void callbackRobotPlan(const nav_msgs::Path::ConstPtr& msg);

private:
	//ros handler
	ros::NodeHandle nh_;

	//subscriber
	ros::Subscriber sub_tracked_ped_;
	ros::Subscriber sub_robot_pos_;
	ros::Subscriber sub_robot_goal_;
	ros::Subscriber sub_robot_plan_;

	//publisher
	ros::Publisher pub_people_;

	//init flag
	flag_t flag_;

	//init msg
	pedsim_msgs::TrackedPersons msg_tracked_peds_;
	people_msgs::People msg_people_;
	nav_msgs::Odometry msg_robot_pos_;	
	geometry_msgs::PoseStamped msg_robot_goal_;
	nav_msgs::Path msg_robot_plan_;	

	//parameters
	double vel_x_, vel_y_;
	double pos_x_, pos_y_;
	double detect_radius_;
	double robot_radius_;
	std::string robot_pos_topic_, person_track_topic_;

	//metrics
	double density_, density_current_, closest_dis_;
	double y_hat_;
	std::vector<int> ped_id_;
	std::vector<int> social_region_;

	//active social or non-social gaussian
	bool social_gaussian_;
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

void PedsimMsgRemap::callbackRobotPlan(const nav_msgs::Path::ConstPtr& msg)
{
	flag_.FIRST_RB_PLAN = true;
	msg_robot_plan_ = *msg;
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
		msg_person.position.x = px;
		msg_person.position.y = py;		
		msg_person.velocity.x = vx;
		msg_person.velocity.y = vy;
		msg_person.velocity.z = 1.0;

		msg_people.people.push_back(msg_person);
	}

	return msg_people;
}

double PedsimMsgRemap::anisotropicFactorPed(double rx, double ry, double px, double py, double pvx, double pvy, double lamda)
{
	double init = 0.0;
	double vec_dist[2] = {rx - px, ry - py};  //{-(px - rx), -(py - ry)} the actual sign convention
	double vec_vel[2] = {pvx, pvy};
	double dot_product = std::inner_product(vec_dist, vec_dist+1, vec_vel, init);
	double cos_angle = dot_product/(hypot(vec_dist[0],vec_dist[1])*hypot(vec_vel[0],vec_vel[1]));
	double anisotropic = lamda + (1.0 - lamda)*(1.0+cos_angle)*0.5;
	return anisotropic;
}

double PedsimMsgRemap::anisotropicFactorRB(double rx, double ry, double px, double py, double rvx, double rvy, double lamda)
{
	double init = 0.0;
	double vec_dist[2] = {px - rx, py - ry};  //{-(px - rx), -(py - ry)} the actual sign convention
	double vec_vel[2] = {rvx, rvy};
	double dot_product = std::inner_product(vec_dist, vec_dist+1, vec_vel, init);
	double cos_angle = dot_product/(hypot(vec_dist[0],vec_dist[1])*hypot(vec_vel[0],vec_vel[1]));
	double anisotropic = lamda + (1.0 - lamda)*(1.0+cos_angle)*0.5;
	return anisotropic;
}

double PedsimMsgRemap::circularSpecification(double C, double density, double sum_radius, double distanceRnP)
{
	double B = exp(1/(C*density));
	return exp((-distanceRnP)/B);
//	return exp((-distanceRnP + C)/B);
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
	int ped_in_circle = 0;

	msg_people.header = msg_tracked_peds_.header;
	for(std::vector<pedsim_msgs::TrackedPerson>::iterator it = msg_tracked_peds_.tracks.begin(); 
									it != msg_tracked_peds_.tracks.end(); ++it) {

		double distanceRnP = getDistance(msg_robot_pos_.pose.pose.position.x, 
											msg_robot_pos_.pose.pose.position.y, 
											it->pose.pose.position.x, 
											it->pose.pose.position.y);

		closest_dis_ = (distanceRnP < closest_dis_)?distanceRnP:closest_dis_;

		if(distanceRnP > detect_radius_)
			continue;

		if(distanceRnP <= 2.0) ++ped_in_circle;

		bool recorded = false;
		int tag = INIT_TAG;
		int i = 0;
		if(distanceRnP - robot_radius_< INITIMATE_SPACE) {
			tag = INITIMATE_TAG;
		} else if(distanceRnP - robot_radius_< PERSONAL_SPACE_CLOSE) {
			tag = PERSONAL_CLOSE_TAG;
		} else if(distanceRnP - robot_radius_< PERSONAL_SPACE) {
			tag = PERSONAL_TAG;
		} else if(distanceRnP - robot_radius_< SOCIAL_SPACE) {
			tag = SOCIAL_TAG;
		} else {
			tag = PUBLIC_TAG;
		}
		for(std::vector<int>::iterator it_id = ped_id_.begin(); 
								it_id != ped_id_.end(); ++it_id) {
			if(*it_id == static_cast<int>(it->track_id)) {
				social_region_[i] = (social_region_[i] > tag)?tag:social_region_[i];
				recorded = true;
				break;
			}
			++i;
		}	
		if(recorded == false) {
			ped_id_.push_back(it->track_id);
			social_region_.push_back(tag);
		}

		s = std::to_string(it->track_id);
		msg_person.name = s;
		msg_person.position = it->pose.pose.position;
		msg_person.velocity.x = it->twist.twist.linear.x;
		msg_person.velocity.y = it->twist.twist.linear.y;
		msg_person.velocity.z = it->twist.twist.linear.z;

		//amplitude process
		// double yaw = quatToYaw(msg_tracked_peds_.header, it->pose.pose, false);
		// double amplitude_scaler = socialForceRobotPed(yaw, msg_robot_pos_.pose.pose.position.z, 
		// 							it->twist.twist.linear.x, it->twist.twist.linear.y, distanceRnP);
		// // bool towards_goal = pedTowardsGoal(msg_robot_goal_.pose.position.x, msg_robot_goal_.pose.position.y,
		// // 						 it->pose.pose.position.x, it->pose.pose.position.y,
		// // 						 it->twist.twist.linear.x, it->twist.twist.linear.y);
		// bool towards_goal = pedTowardsGoal(20.0, 4.0,
		// 						 it->pose.pose.position.x, it->pose.pose.position.y,
		// 						 it->twist.twist.linear.x, it->twist.twist.linear.y);

		double anisotropicped = anisotropicFactorPed(msg_robot_pos_.pose.pose.position.x, msg_robot_pos_.pose.pose.position.y,
				 it->pose.pose.position.x, it->pose.pose.position.y,
				  it->twist.twist.linear.x, it->twist.twist.linear.y, 0.35);

		double anisotropicrb = anisotropicped;
		if(msg_robot_pos_.twist.twist.linear.x < 0.01 && msg_robot_pos_.twist.twist.linear.x > -0.01 &&
			msg_robot_pos_.twist.twist.linear.y < 0.01 && msg_robot_pos_.twist.twist.linear.y > -0.01) {
			
		} else {
			anisotropicrb = anisotropicFactorRB(msg_robot_pos_.pose.pose.position.x, msg_robot_pos_.pose.pose.position.y,
					 it->pose.pose.position.x, it->pose.pose.position.y,
					  msg_robot_pos_.twist.twist.linear.x, msg_robot_pos_.twist.twist.linear.y, 0.0);
			//ROS_INFO("%f %f %f", anisotropicrb, msg_robot_pos_.twist.twist.linear.x, msg_robot_pos_.twist.twist.linear.y);
		}

		double circular_spec = circularSpecification(5.0, density_current_, 0.45, distanceRnP);
		circular_spec = (circular_spec > 1.0)?1.0:circular_spec/1.0;
		
		// ROS_INFO("%f %f %f %f", anisotropic, circular_spec, distanceRnP, circular_spec*anisotropic);
		//ROS_INFO("%f %f %f %f", msg_robot_pos_.pose.pose.position.z, yaw_corrected, msg_robot_pos_.pose.pose.position.z - yaw_corrected, f);		
		//ROS_INFO("%f %f", it->twist.twist.linear.x, it->twist.twist.linear.y);
		//TO DO figure out a better way to pass the amplitude
		//msg_person.velocity.z = amplitudeScaler(distanceRnP, towards_goal);  //previous way to scale amplitude
		if(social_gaussian_)
			msg_person.velocity.z = circular_spec*(anisotropicrb*anisotropicped); 
		else
			msg_person.velocity.z = 1;//circular_spec*(anisotropicrb*anisotropicped); //1.0;

		msg_people.people.push_back(msg_person);
	}

	double dense = ped_in_circle/(PI*4.0);
	density_current_ = (dense > 0.001)?dense:1.0/(PI*4.0);
	density_ = (dense > density_)?dense: density_;

	// ROS_INFO("%f", density);

	return msg_people;
}

double PedsimMsgRemap::getDistance(double x1, double y1, double x2, double y2) 
{
	return std::hypot(x1-x2,y1-y2);
}

void PedsimMsgRemap::dynamicConfig(std::string node, std::string parameter, std::string value)
{
	std::string s = "rosrun dynamic_reconfigure dynparam set " + node + " " + parameter + " " + value;
	system(s.c_str());	
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

double PedsimMsgRemap::planLengthCalc(nav_msgs::Path plan)
{
	double path_length = 0.0;
	double prev_x = 0.0;
	double prev_y = 0.0;
	double temp_y_hat_ = 0.0;
	y_hat_ = 0.0;	//have to reset y_hat 

	for(std::vector<geometry_msgs::PoseStamped>::iterator it = plan.poses.begin(); 
							it != plan.poses.end(); ++it) {
		if(it == plan.poses.begin()) {
			prev_x = it->pose.position.x;
			prev_y = it->pose.position.y;
			continue;
		}
		path_length += getDistance(prev_x, prev_y, it->pose.position.x, it->pose.position.y);
		prev_x = it->pose.position.x;
		prev_y = it->pose.position.y;

		//calc y_hat	
		temp_y_hat_ = fabs(fabs(it->pose.position.y) - 4.0);
		y_hat_ = (temp_y_hat_ > y_hat_)?temp_y_hat_:y_hat_;	
	}

	return path_length;
}

void PedsimMsgRemap::loop()
{
	ros::Rate r(LOOP_RATE);
	int i_print = 0;
	int i_initimate = 0, i_personal = 0, i_personal_close = 0, i_social = 0, i_public = 0;
	double amplitude = 0;
	double variance = 0;

	while(ros::ok()) {
		if(flag_.FIRST_TRACKED == true && flag_.FIRST_RB_POS == true) { //&&
			//flag_.FIRST_RB_GOAL == true) {
			//msg_people_ = msgRemapWithFakePeople(20.0, 4.0, -1.2, 0.0); //to publish the fake costmap
			msg_people_ = msgRemapWithAmplitude(); 				   //to publish the right costmap
			pub_people_.publish(msg_people_);

			//metrics calculation part
			if(++i_print > LOOP_RATE) {
				for(std::vector<int>::iterator it_region = social_region_.begin(); 
										it_region != social_region_.end(); ++it_region) {
					switch(*it_region) {
						case INITIMATE_TAG: ++i_initimate; break;
						case PERSONAL_CLOSE_TAG: ++i_personal_close; break;						
						case PERSONAL_TAG: ++i_personal; break;
						case SOCIAL_TAG: ++i_social; break;
						case PUBLIC_TAG: ++i_public; break;
						default: break;
					}
				}
				//dense%f dis%f in%d per%d soc%d pub%d 
				//std::cout << "hello world";
				ROS_INFO("%f,%f,%d,%d,%d", density_, closest_dis_ - robot_radius_, 
										i_initimate, i_personal_close, i_personal);
				i_initimate = i_personal = i_personal_close = i_social = i_public = 0; //this is not cumulative, so reset is needed
				i_print = 0;		
			}

			//experiment for varying gaussian @ LOOP RATE 5Hz
			/* if(flag_.FIRST_VARY_GAUSS == false) {
				for(double var = 1.75;var < 3.0;) {
					// for(double p_a = 0.2;p_a > 0.1;) {
					// 	msg_people_ = msgRemapWithFakePeople(20.0, 4.0, -1.26, 0.0); //to publish the fake costmap
					// 	pub_people_.publish(msg_people_);	
					// 	double amp = 50.0/p_a;
					// 	dynamicConfig("/move_base_node/global_costmap/social_compliance", "amplitude", std::to_string(amp));
					// 	dynamicConfig("/move_base_node/global_costmap/social_compliance", "covariance", std::to_string(var));
					// 	ros::spinOnce();
					// 	r.sleep();
					// 	double path_len = planLengthCalc(msg_robot_plan_);
					// 	ROS_INFO("%f %f %f %f %f", amp, var, path_len, y_hat_, 50.0/amp);
					// 	p_a -= 0.2;							
					// }
					for(double amp = 5;amp < 251;) {
						msg_people_ = msgRemapWithFakePeople(20.0, 4.0, -1.26, 0.0); //to publish the fake costmap
						pub_people_.publish(msg_people_);	
						dynamicConfig("/move_base_node/global_costmap/social_compliance", "amplitude", std::to_string(amp));
						dynamicConfig("/move_base_node/global_costmap/social_compliance", "covariance", std::to_string(var));
						ros::spinOnce();
						r.sleep();
						double path_len = planLengthCalc(msg_robot_plan_);
						ROS_INFO("%f %f %f %f %f", amp, var, path_len, y_hat_, 50.0/amp);
						amp += 5;							
					}					
					var += 0.25;
				}
				flag_.FIRST_VARY_GAUSS = true;		
			} */
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