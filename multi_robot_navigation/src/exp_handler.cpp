// Modules for client
#include "plan_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
// Modules for msg
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

#include <visualization_msgs/MarkerArray.h>
// Teleport interface
#include <bwi_msgs/RobotTeleporterInterface.h>

// Safety protocol
#include "ChickenSafety.h"
#include "multi_robot_collision_avoidance/EvalWaypoint.h"

#include <tf/transform_listener.h>
#include <ros/ros.h>

// Write simulation results in file.
#include <fstream>

#define PI (3.1415926)
typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

class MRH_Experiment{
protected:
	MoveBaseClient marvin_ac;
	MoveBaseClient roberto_ac;
  ros::NodeHandle *nh_;

  geometry_msgs::PoseWithCovarianceStamped roberto_spawn_pose;
  move_base_msgs::MoveBaseGoal roberto_init_pose;
  move_base_msgs::MoveBaseGoal roberto_goal_pose;
  ros::ServiceClient roberto_teleport;
  ros::ServiceClient roberto_clear_costmap;
  ros::Publisher     roberto_localizer;

  geometry_msgs::PoseWithCovarianceStamped marvin_spawn_pose;
  move_base_msgs::MoveBaseGoal marvin_init_pose;
  move_base_msgs::MoveBaseGoal marvin_goal_pose;
  ros::ServiceClient marvin_teleport;
  ros::ServiceClient marvin_clear_costmap;
  ros::Publisher     marvin_localizer;
	// Measuring times
	ros::Time start_time;
	ros::Rate *rate;
	// subscribe plans
  vector<geometry_msgs::Pose> robertoPlan_;
  vector<geometry_msgs::Pose> marvinPlan_;
  ros::Subscriber subscribeRobertoPlan_;
  ros::Subscriber subscribeMarvinPlan_;
  ros::AsyncSpinner *spinner;
	void subscribeRobertoPlan(const visualization_msgs::MarkerArray plan);
	void subscribeMarvinPlan(const visualization_msgs::MarkerArray plan);

	ros::ServiceClient siren_find;
public:
  MRH_Experiment():marvin_ac(""),roberto_ac(""){}
  MRH_Experiment(ros::NodeHandle* nh);
	// Reset environments
  void setSpawnPose(string name, float x, float y, float yaw);
  void setInitPose(string name, float x, float y, float yaw);
	void setGoalPose(string name, float x, float y, float yaw);
  void showInitPose(){
    ROS_INFO_STREAM(roberto_init_pose);
    ROS_INFO_STREAM(marvin_init_pose);
  }
  bool reset_environment();
	// basic fuctions
	void move(string robot_name, move_base_msgs::MoveBaseGoal goal);
	float dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt);
	bool checkCollision();
	// Experiments
	// without safety protocol
	vector<double> run_raw_experiment(int timeout, int delay);
	// Chicken safety protocol
	vector<double> run_chicken_experiment(int timeout);
	void goToParkingZone(string location);
	void go(vector<float> coord);
	// Siren safety protocol
	vector<double> run_siren_experiment(int timeout);
};

MRH_Experiment::MRH_Experiment(ros::NodeHandle* nh):marvin_ac("marvin/move_base"),roberto_ac("roberto/move_base")
{
  nh_ = nh;
	rate = new ros::Rate(10);

  nh_ -> setParam("roberto/move_base/recovery_behavior_enabled", false);
  nh_ -> setParam("marvin/move_base/recovery_behavior_enabled", false);

  // Teleport service client
  roberto_teleport = nh_->serviceClient<bwi_msgs::RobotTeleporterInterface>("roberto/teleport_robot");
  roberto_teleport.waitForExistence(ros::Duration(30));
  roberto_clear_costmap = nh_->serviceClient<std_srvs::Empty>("roberto/move_base/clear_costmaps");
  roberto_clear_costmap.waitForExistence(ros::Duration(30));
  marvin_teleport = nh_->serviceClient<bwi_msgs::RobotTeleporterInterface>("marvin/teleport_robot");
  marvin_teleport.waitForExistence(ros::Duration(30));
  marvin_clear_costmap = nh_->serviceClient<std_srvs::Empty>("marvin/move_base/clear_costmaps");
  marvin_clear_costmap.waitForExistence(ros::Duration(30));

  roberto_localizer = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>("/roberto/initialpose",1,true);
  marvin_localizer = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>("/marvin/initialpose",1,true);

	// subscribe EBandPlanner topic
	string commonEBand = "/move_base/EBandPlannerROS/eband_visualization_array";
	string myEBandTopic = "/roberto" + commonEBand;

	string othersEBandTopic = "/marvin" + commonEBand;
	this->subscribeRobertoPlan_ = nh_->subscribe(myEBandTopic, 1, &MRH_Experiment::subscribeRobertoPlan, this);
	this->subscribeMarvinPlan_ = nh_->subscribe(othersEBandTopic, 1, &MRH_Experiment::subscribeMarvinPlan, this);
	this->spinner = new ros::AsyncSpinner(2);
	this->spinner->start();

	siren_find = nh->serviceClient<multi_robot_collision_avoidance::EvalWaypoint>("eval_waypoint");
};
void MRH_Experiment::subscribeRobertoPlan(const visualization_msgs::MarkerArray plan){
  this->robertoPlan_.clear();
  for(int i=0; i<plan.markers.size(); i++){
    this->robertoPlan_.push_back(plan.markers[i].pose);
  }
};
void MRH_Experiment::subscribeMarvinPlan(const visualization_msgs::MarkerArray plan){
  this->marvinPlan_.clear();
  for(int i=0; i<plan.markers.size(); i++){
    this->marvinPlan_.push_back(plan.markers[i].pose);
  }
};
void MRH_Experiment::setInitPose(string name, float x, float y, float yaw){
  if(name == "marvin"){
    marvin_init_pose.target_pose.header.frame_id = "marvin/level_mux_map";
    marvin_init_pose.target_pose.header.stamp = ros::Time::now();

    marvin_init_pose.target_pose.pose.position.x = x;
    marvin_init_pose.target_pose.pose.position.y = y;
    tf::Quaternion marvin_quat;
    marvin_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(marvin_quat, marvin_init_pose.target_pose.pose.orientation);
  }
  else if(name=="roberto"){
    roberto_init_pose.target_pose.header.frame_id = "roberto/level_mux_map";
    roberto_init_pose.target_pose.header.stamp = ros::Time::now();

    roberto_init_pose.target_pose.pose.position.x = x;
    roberto_init_pose.target_pose.pose.position.y = y;
    tf::Quaternion roberto_quat;
    roberto_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(roberto_quat, roberto_init_pose.target_pose.pose.orientation);
  }
  else{
    ROS_INFO_STREAM("Available Options: [roberto marvin]");
  }
};
void MRH_Experiment::setGoalPose(string name, float x, float y, float yaw){
  if(name == "marvin"){
    marvin_goal_pose.target_pose.header.frame_id = "marvin/level_mux_map";
    marvin_goal_pose.target_pose.header.stamp = ros::Time::now();

    marvin_goal_pose.target_pose.pose.position.x = x;
    marvin_goal_pose.target_pose.pose.position.y = y;
    tf::Quaternion marvin_quat;
    marvin_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(marvin_quat, marvin_goal_pose.target_pose.pose.orientation);
  }
  else if(name=="roberto"){
    roberto_goal_pose.target_pose.header.frame_id = "roberto/level_mux_map";
    roberto_goal_pose.target_pose.header.stamp = ros::Time::now();

    roberto_goal_pose.target_pose.pose.position.x = x;
    roberto_goal_pose.target_pose.pose.position.y = y;
    tf::Quaternion roberto_quat;
    roberto_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(roberto_quat, roberto_goal_pose.target_pose.pose.orientation);
  }
  else{
    ROS_INFO_STREAM("Available Options: [roberto marvin]");
  }
};
void MRH_Experiment::setSpawnPose(string name, float x, float y, float yaw){
  if(name == "marvin"){
    marvin_spawn_pose.header.frame_id = "marvin/level_mux_map";
    marvin_spawn_pose.header.stamp = ros::Time::now();

    marvin_spawn_pose.pose.pose.position.x = x;
    marvin_spawn_pose.pose.pose.position.y = y;
    marvin_spawn_pose.pose.pose.position.z = 0.0;
    tf::Quaternion marvin_quat;
    marvin_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(marvin_quat, marvin_spawn_pose.pose.pose.orientation);

    marvin_spawn_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
    marvin_spawn_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
    marvin_spawn_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
  }
  else if(name=="roberto"){
    roberto_spawn_pose.header.frame_id = "roberto/level_mux_map";
    roberto_spawn_pose.header.stamp = ros::Time::now();

    roberto_spawn_pose.pose.pose.position.x = x;
    roberto_spawn_pose.pose.pose.position.y = y;
    roberto_spawn_pose.pose.pose.position.z = 0.0;
    tf::Quaternion roberto_quat;
    roberto_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(roberto_quat, roberto_spawn_pose.pose.pose.orientation);

    roberto_spawn_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
    roberto_spawn_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
    roberto_spawn_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
  }
  else{
    ROS_INFO_STREAM("Available Options: [roberto marvin]");
  }
};
bool MRH_Experiment::reset_environment(){
  std_srvs::Empty clear;

  // Teleport robots to spawn pose
  bwi_msgs::RobotTeleporterInterface roberto_rti;
  roberto_rti.request.pose = roberto_spawn_pose.pose.pose;
  roberto_teleport.call(roberto_rti);
  bwi_msgs::RobotTeleporterInterface marvin_rti;
  marvin_rti.request.pose = marvin_spawn_pose.pose.pose;
  marvin_teleport.call(marvin_rti);
	// Wait for teleport to complete.
  ros::Duration(0.5).sleep();// Always NEED TIME!
  // Publish current position to localization node
  roberto_spawn_pose.header.stamp = ros::Time::now();
  roberto_localizer.publish(roberto_spawn_pose);
  roberto_clear_costmap.call(clear);
  marvin_spawn_pose.header.stamp = ros::Time::now();
  marvin_localizer.publish(marvin_spawn_pose);
  marvin_clear_costmap.call(clear);

  // Further localize marvin by moving it to initial position
  roberto_ac.sendGoal(roberto_init_pose);
  marvin_ac.sendGoal(marvin_init_pose);

	// wait until both robot reaches experiment start position.
	ros::Rate r(10);
	int timeout = 0;
	while(ros::ok()){
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED &&
	  marvin_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			break;
		}
		r.sleep();
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::ABORTED ||
	  marvin_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			return false;
		}
		timeout = timeout+1;
		if(timeout>200){
			ROS_INFO_STREAM("Reset environment Timeout");
			return false;
		}
	}
  roberto_clear_costmap.call(clear);
  marvin_clear_costmap.call(clear);
	return true;
};
void MRH_Experiment::move(string robot_name, move_base_msgs::MoveBaseGoal goal){
	if(robot_name == "roberto") roberto_ac.sendGoal(goal);
	else if(robot_name == "marvin") marvin_ac.sendGoal(goal);
	else ROS_INFO_STREAM("Available robots are [roberto, marvin]");
};
float MRH_Experiment::dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt){
  return hypot(my_plan_pt.x - others_plan_pt.x, my_plan_pt.y - others_plan_pt.y);
};
bool MRH_Experiment::checkCollision(){
	// subscribe two local_plan topic
	// check distance between localplan[0]
	// if dist < 0.7, collision occur. return True;
	if(this->robertoPlan_.size() && this->marvinPlan_.size()){
		if(dist(this->robertoPlan_[0].position, this->marvinPlan_[0].position) < 0.7){
			return true;
		}
	}
	return false;
};
vector<double> MRH_Experiment::run_raw_experiment(int timeout, int delay=0.0){
	// This experiment is the baseline experiments
	// Two robot will simply go to the others location with vanila navigation stack
	// return how long it took for them to passby
	// delay: How much delay for marvin to wait after roberto moves.
	vector<double> endtime;
	endtime.push_back(-1.0);
	endtime.push_back(-1.0);

	//-----------------------------------------------------------------------------------
	// Prof. Joydeep's suggestion. Use buffer waypoint before experiment.
	move_base_msgs::MoveBaseGoal marvin_wp;
	marvin_wp.target_pose.header.frame_id = "marvin/level_mux_map";
	marvin_wp.target_pose.header.stamp = ros::Time::now();

	marvin_wp.target_pose.pose.position.x = marvin_init_pose.target_pose.pose.position.x;
	marvin_wp.target_pose.pose.position.y = marvin_init_pose.target_pose.pose.position.y;
	tf::Quaternion marvin_quat;
	marvin_quat.setRPY(0.0, 0.0, PI/180. * -20);
	tf::quaternionTFToMsg(marvin_quat, marvin_wp.target_pose.pose.orientation);

	marvin_ac.sendGoalAndWait(marvin_wp);
	//------------------------------------------------------------------------------------
	//Check Markovity ---------------- Additional experiments
	marvin_ac.sendGoalAndWait(this->marvin_init_pose);
	//------------------------------------------------------------------------------------
	this->start_time = ros::Time::now();
	if(delay == 0){
		this->move("marvin", marvin_goal_pose);
		this->move("roberto", roberto_goal_pose);
	}
	if(delay<0){
		this->move("marvin", marvin_goal_pose);
		ros::Duration(delay).sleep();
		this->move("roberto", roberto_goal_pose);
	}
	else{
		this->move("roberto", roberto_goal_pose);
		ros::Duration(delay).sleep();
		this->move("marvin", marvin_goal_pose);
	}

	int t = 0;
	bool roberto_check = true;
	bool marvin_check = true;
	while(ros::ok()){
		// check collision between two robots
		if(this->checkCollision()){
			endtime[0] = endtime[1] = 1000.0 + (ros::Time::now() - this->start_time).toSec();
			return endtime;
		}
		// if success, return how long it took
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && roberto_check){
			roberto_check = false;
			endtime[0] = (ros::Time::now() - this->start_time).toSec();
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && marvin_check){
			marvin_check = false;
			endtime[1] = (ros::Time::now() - this->start_time).toSec();
		}
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_INFO_STREAM("roberto aborted. RESUME");
			this->move("roberto", roberto_goal_pose);
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_INFO_STREAM("marvin  aborted. RESUME");
			this->move("marvin", marvin_goal_pose);
		}
		// if timeout, return timeout value
		if(t > timeout * 10){
			if(endtime[0]<0) endtime[0] = double(timeout);
			if(endtime[1]<0) endtime[1] = double(timeout);
		}
		t += 1;
		if(endtime[0]>0 && endtime[1]>0) break;
		this->rate->sleep();
	}
	return endtime;
};
vector<double> MRH_Experiment::run_chicken_experiment(int timeout){
	// This experiment is the experiment with baseline algorithm
	// One bold robot(roberto) will go to it's destination with vanila navigation stack
	// Other chicken robot(marvin) will run away to the pre-defined parking spot when they detect potential collision.
	// return how long it took for them to passby
	vector<double> endtime;
	endtime.push_back(-1.0);
	endtime.push_back(-1.0);
	// Activate safety protocol
	ChickenSafetyProtocol csp("/marvin", "/jinsoo");
	ros::Duration(0.5).sleep();

	int t = 0;
	bool roberto_check = true;
	bool marvin_check = true;

	bool danger = false;
	bool wait   = false;
	bool resume = false;
	string parkingZone;

	// Set both robots to go to their goal position
	this->move("roberto", roberto_goal_pose);// bold robot
	this->move("marvin", marvin_goal_pose); // chicken robot
	this->start_time = ros::Time::now();
	while(ros::ok()){
		if(wait == false){
			bool danger = csp.distanceBaseAlarm(0.6);
			if(danger == true){
				marvin_ac.cancelGoal();
				parkingZone = csp.findSafeZone();
				goToParkingZone(parkingZone);

				wait=true;
				this->rate->sleep();
			}
		}
		if(wait == true && csp.waitUntilSafe()){
			if(marvin_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				wait = false;
				this->move("marvin", marvin_goal_pose);
			}
		}
		// check collision between two robots
		if(this->checkCollision()){
			endtime[0] = endtime[1] = 1000.0;
			return endtime;
		}
		// if success, return how long it took
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && roberto_check){
			roberto_check = false;
			endtime[0] = (ros::Time::now() - this->start_time).toSec();

			// For reliable detection of chicken bot, move further.
			move_base_msgs::MoveBaseGoal tmp_goal_pose;
	    tmp_goal_pose.target_pose.header.frame_id = "roberto/level_mux_map";
	    tmp_goal_pose.target_pose.header.stamp = ros::Time::now();

	    tmp_goal_pose.target_pose.pose.position.x = 0.00;
	    tmp_goal_pose.target_pose.pose.position.y = 14.00;
	    tf::Quaternion tmp_quat;
	    tmp_quat.setRPY(0.0, 0.0, -PI/2);
	    tf::quaternionTFToMsg(tmp_quat, tmp_goal_pose.target_pose.pose.orientation);

			this->move("roberto", tmp_goal_pose);
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && marvin_check && wait == false){
			marvin_check = false;
			endtime[1] = (ros::Time::now() - this->start_time).toSec();
		}
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_INFO_STREAM("roberto aborted. RESUME");
			this->move("roberto", roberto_goal_pose);
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_INFO_STREAM("marvin  aborted. RESUME");
			this->move("marvin", marvin_goal_pose);
		}
		// if timeout, return timeout value
		if(t > timeout * 10){
			if(endtime[0]<0) endtime[0] = double(timeout);
			if(endtime[1]<0) endtime[1] = double(timeout);
		}
		t += 1;
		if(endtime[0]>0 && endtime[1]>0) break;
		this->rate->sleep();
	}
	return endtime;
};
vector<double> MRH_Experiment::run_siren_experiment(int timeout){
	// This experiment is the experiment with baseline algorithm
	// One bold robot(roberto) will go to it's destination with vanila navigation stack
	// Other chicken robot(marvin) will run away to the pre-defined parking spot when they detect potential collision.
	// return how long it took for them to passby
	vector<double> endtime;
	endtime.push_back(-1.0);
	endtime.push_back(-1.0);
	// Activate safety protocol
	ChickenSafetyProtocol csp("/marvin", "/jinsoo");
	ros::Duration(0.5).sleep();

	int t = 0;
	bool roberto_check = true;
	bool marvin_check = true;

	bool danger = false;
	bool wait   = false;
	bool resume = false;
	string parkingZone;

	// Set both robots to go to their goal position
	this->move("roberto", roberto_goal_pose);// bold robot
	this->move("marvin", marvin_goal_pose); // chicken robot
	this->start_time = ros::Time::now();
	while(ros::ok()){
		if(wait == false){
			bool danger = csp.distanceBaseAlarm(0.6);
			if(danger == true){
				marvin_ac.cancelGoal();

				multi_robot_collision_avoidance::EvalWaypoint srv;
				srv.request.chicken_pose = marvinPlan_[0];
				srv.request.bold_pose = robertoPlan_[0];

				move_base_msgs::MoveBaseGoal wp;
				if (siren_find.call(srv))
			  {
			    wp.target_pose.header.frame_id = "marvin/level_mux_map";
			    wp.target_pose.header.stamp = ros::Time::now();

			    wp.target_pose.pose.position.x = srv.response.waypoint.position.x;
			    wp.target_pose.pose.position.y = srv.response.waypoint.position.y;
			    wp.target_pose.pose.orientation = marvin_goal_pose.target_pose.pose.orientation;
			  }
			  else
			  {
			    ROS_ERROR("Failed to call service eval_waypoint");
			  }

				this->marvin_ac.sendGoalAndWait(wp);

				wait=true;
				this->rate->sleep();
			}
		}
		if(wait == true && csp.waitUntilSafe()){
			if(marvin_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				wait = false;
				this->move("marvin", marvin_goal_pose);
			}
		}
		// check collision between two robots
		if(this->checkCollision()){
			endtime[0] = endtime[1] = 1000.0;
			return endtime;
		}
		// if success, return how long it took
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && roberto_check){
			roberto_check = false;
			endtime[0] = (ros::Time::now() - this->start_time).toSec();
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && marvin_check && wait == false){
			marvin_check = false;
			endtime[1] = (ros::Time::now() - this->start_time).toSec();
		}
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_INFO_STREAM("roberto aborted. RESUME");
			this->move("roberto", roberto_goal_pose);
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_INFO_STREAM("marvin  aborted. RESUME");
			this->move("marvin", marvin_goal_pose);
		}
		// if timeout, return timeout value
		if(t > timeout * 10){
			if(endtime[0]<0) endtime[0] = double(timeout);
			if(endtime[1]<0) endtime[1] = double(timeout);
		}
		t += 1;
		if(endtime[0]>0 && endtime[1]>0) break;
		this->rate->sleep();
	}
	return endtime;
};
void MRH_Experiment::go(vector<float> coord){
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "marvin/level_mux_map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = coord[0];
  goal.target_pose.pose.position.y = coord[1];
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, coord[2]);
  tf::quaternionTFToMsg(quat, goal.target_pose.pose.orientation);
	this->marvin_ac.sendGoal(goal);
};
void MRH_Experiment::goToParkingZone(string location){
  vector<float> coord;
  if(location=="p3_15"){
    coord.push_back(3.627);
    coord.push_back(10.41);
    coord.push_back(PI/2);
    go(coord);
  }
  else if(location=="p3_16"){
    coord.push_back(5.65);
    coord.push_back(10.46);
    coord.push_back(PI/2);
    go(coord);
  }
  else if(location=="p3_18"){
    coord.push_back(17.922947);
    coord.push_back(10.459691);
    coord.push_back(PI/2);
    go(coord);
  }
  else if(location=="p3_20"){
    coord.push_back(20.495056);
    coord.push_back(8.198261);
    coord.push_back(-PI/2);
    go(coord);
  }
  else if(location=="p3_30"){
    coord.push_back(32.467041);
    coord.push_back(8.474984);
    coord.push_back(-PI/2);
    go(coord);
  }
  else if(location=="p3_31"){
    coord.push_back(39.029182);
    coord.push_back(8.375641);
    coord.push_back(-PI/2);
    go(coord);
  }
  else if(location=="p3_44"){
    coord.push_back(23.564869);
    coord.push_back(19.213001);
    coord.push_back(-PI/2);
    go(coord);
  }
  else if(location=="p3_45"){
    coord.push_back(3.834564);
    coord.push_back(19.154118);
    coord.push_back(-PI/2);
    go(coord);
  }
  else{
    ROS_INFO_STREAM(location << " not understood");
  }
};

int main(int argc, char** argv){
	if(argc != 3){
		ROS_INFO_STREAM("How to use:");
		ROS_INFO_STREAM("rosrun multi_robot_navigation exp_handler ${exp_type} ${num_exp}");
		ROS_INFO_STREAM("exp_type: [raw chicken]");
		exit(0);
		ROS_INFO_STREAM("rosrun multi_robot_navigation exp_handler ${exp_type} ${num_exp} ${time_out}");
	}
  ros::init(argc, argv, "environment_reset_test");
  ros::NodeHandle nh;

	ofstream outfile;
	string filename;
	string exp_name = argv[1];
	if(exp_name == "raw"){
		//outfile.open("/home/jinsoo/Desktop/raw_exp.txt", ios::out | ios::app);
		outfile.open("/home/jinsoo/Desktop/simulation_EXP/raw_exp_0_degree.txt", ios::out | ios::app);
	}
	else if(exp_name == "chicken"){
		//outfile.open("/home/jinsoo/Desktop/chicken_exp.txt", ios::out | ios::app);
		outfile.open("/home/jinsoo/Desktop/simulation_EXP/chicken_exp_0_degree.txt", ios::out | ios::app);
	}
	else if(exp_name == "siren"){
		outfile.open("/home/jinsoo/Desktop/simulation_EXP/siren_exp_with_8m_0_degree.txt", ios::out | ios::app);
	}
	else{
		ROS_INFO_STREAM("exp_type: [raw chicken siren]");
		exit(1);
	}
	outfile<<"reset_time, roberto time, marvin time"<<endl;
  MRH_Experiment eh(&nh);
  // Set initial environment for expermients
	eh.setSpawnPose("marvin", 4.8, 18.1, PI);
  eh.setInitPose("marvin", 8.8, 18.1, PI/180.*0.); // Test: -PI/12 0 PI/12  150 each
	eh.setGoalPose("marvin", 26.45, 18.15, 0.0);
	eh.setSpawnPose("roberto", 23.8, 18.1, 0.0);
  eh.setInitPose("roberto", 19.8, 18.1, PI);
	eh.setGoalPose("roberto", 2.15, 18.15, PI);
  //eh.showInitPose();
	float delay = 0.0;
	//Set experiments here
	double reset_time;
	for(int i=0; i<atoi(argv[2]); i++){
		if(i==150){
			eh.setInitPose("marvin", 8.8, 18.1, PI/180. * 15.);
			outfile.close();outfile.clear();
			outfile.open(("/home/jinsoo/Desktop/simulation_EXP/"+exp_name+"_exp_15_degree.txt").c_str(), ios::out|ios::app);
			outfile<<"reset_time, roberto time, marvin time"<<endl;
		}
		if(i==300){
			eh.setInitPose("marvin", 8.8, 18.1, PI/180. * 30.);
			outfile.close();outfile.clear();
			outfile.open(("/home/jinsoo/Desktop/simulation_EXP/"+exp_name+"_exp_30_degree.txt").c_str(), ios::out|ios::app);
			outfile<<"reset_time, roberto time, marvin time"<<endl;
		}
		if(i==450){
			eh.setInitPose("marvin", 8.8, 18.1, PI/180. * 45.);
			outfile.close();outfile.clear();
			outfile.open(("/home/jinsoo/Desktop/simulation_EXP/"+exp_name+"_exp_45_degree.txt").c_str(), ios::out|ios::app);
			outfile<<"reset_time, roberto time, marvin time"<<endl;
		}
		if(i==600){
			eh.setInitPose("marvin", 8.8, 18.1, PI/180. * -15.);
			outfile.close();outfile.clear();
			outfile.open(("/home/jinsoo/Desktop/simulation_EXP/"+exp_name+"_exp_-15_degree.tx").c_str(), ios::out|ios::app);
			outfile<<"reset_time, roberto time, marvin time"<<endl;
		}
		if(i==750){
			eh.setInitPose("marvin", 8.8, 18.1, PI/180. * -30.);
			outfile.close();outfile.clear();
			outfile.open(("/home/jinsoo/Desktop/simulation_EXP/"+exp_name+"_exp_-30_degree.tx").c_str(), ios::out|ios::app);
			outfile<<"reset_time, roberto time, marvin time"<<endl;
		}
		if(i==900){
			eh.setInitPose("marvin", 8.8, 18.1, PI/180. * -45.);
			outfile.close();outfile.clear();
			outfile.open(("/home/jinsoo/Desktop/simulation_EXP/"+exp_name+"_exp_-45_degree.tx").c_str(), ios::out|ios::app);
			outfile<<"reset_time, roberto time, marvin time"<<endl;
		}
		ros::Time start_time = ros::Time::now();
	  bool success = eh.reset_environment();
		// if reset fails, throw that experiment and reset again.
		if(!success){
			i = i-1;
			continue;
		}
		ROS_INFO_STREAM((i+1)<<"th experiment");
		reset_time = (ros::Time::now() - start_time).toSec();
		vector<double> result;
		if(exp_name == "raw") result = eh.run_raw_experiment(200, delay);
		else if(exp_name == "chicken") result = eh.run_chicken_experiment(100);
		else if(exp_name == "siren") result = eh.run_siren_experiment(75);
		outfile << reset_time << ", " << result[0] << ", " << result[1] << endl;
	}
}
