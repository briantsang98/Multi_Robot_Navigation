
#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <bwi_planning_common/structures.h>
#include <bwi_planning_common/utils.h>
#include <bwi_planning_common/interactions.h>

#include <ros/ros.h>
#include <bwi_msgs/RobotTeleporterInterface.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <bwi_tools/point.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <string>
#include <limits.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <std_srvs/Empty.h>

using namespace std;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace bwi_planning_common;
typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class Pass_Hallway{
public:
	Pass_Hallway():marvin_ac(""),roberto_ac(""){}
	Pass_Hallway(ros::NodeHandle* nh,int runs);
	void run();
	void callback(const Odometry::ConstPtr &roberto_odom, const Odometry::ConstPtr &marvin_odom);
	bool checkRobotsMovingAway(vector<int> &distances);
	void stopRobot(Client *client);
	void reset_positions();
	bool collision(const Point &roberto_odom, const Point &marvin_odom);
	bool backwards(const Point &roberto_odom, const Point &marvin_odom);
	void get_waypoint();
  
protected: 
	MoveBaseClient marvin_ac;
	MoveBaseClient roberto_ac;
    ros::NodeHandle *nh_;
    Client *roberto_client; 
    Client *marvin_client; 
	ros::ServiceClient make_plan_client_roberto;
    ros::ServiceClient make_plan_client_marvin;
    string marvin_door;
    string roberto_door;
    string marvin_fixed_frame;
    string roberto_fixed_frame;
    ros::Publisher stop_roberto;
    ros::Publisher stop_marvin;
    ofstream myfile;
    std::vector<float> robot_odom_distances;
    ros::ServiceClient marvin_teleporter_client;
    ros::ServiceClient roberto_teleporter_client;
    ros::ServiceClient resetMarvin;
    ros::ServiceClient resetRoberto;
    ros::Publisher roberto_relocalize;
    ros::Publisher marvin_relocalize;
    std::vector<float> times;
    bool robotsClose;
    bool reset;
    float episodeStartTime;
    float closeStartTime;
    bool roberto_center;
    bool marvin_center;
    bool successfulPass;

    float x_hallway_min = 16.0;
    float x_hallway_max = 24.0;
    float y_hallway_min = 6.5;
    float y_hallway_max = 9.25;

    //std::vector<float> thetas([0.0, 3.141/2.0, 3.141, 3.0*3.141/2.0])

    geometry_msgs::Pose waypoint;
    //std::vector<float> phi(17);
    float reward;


};

Pass_Hallway::Pass_Hallway( ros::NodeHandle* nh, int runs):marvin_ac("marvin/move_base"),roberto_ac("roberto/move_base")
{
  nh_ = nh;
  /*marvin_client = new Client("/marvin/action_executor/execute_plan", true);
  roberto_client = new Client("/roberto/action_executor/execute_plan", true);
  marvin_client->waitForServer();
  roberto_client->waitForServer();*/
  myfile.open("testfile.txt");
  if(myfile.is_open()){
    myfile<<"hi\n";
  }else{
    ROS_INFO("not open");
    exit(1);
  }
  stop_roberto = nh_->advertise<actionlib_msgs::GoalID> 
        ("/roberto/move_base/cancel", 1000, true); 
  stop_marvin = nh_->advertise<actionlib_msgs::GoalID> 
      ("/marvin/move_base/cancel", 1000, true); 
  roberto_teleporter_client = nh_->serviceClient<bwi_msgs::RobotTeleporterInterface>("roberto/teleport_robot");
  roberto_teleporter_client.waitForExistence();
  marvin_teleporter_client = nh_->serviceClient<bwi_msgs::RobotTeleporterInterface>("marvin/teleport_robot");
  marvin_teleporter_client.waitForExistence();
  resetMarvin = nh_ -> serviceClient<std_srvs::Empty>("marvin/move_base/clear_costmaps");
  resetMarvin.waitForExistence();
  resetRoberto = nh_ -> serviceClient<std_srvs::Empty>("roberto/move_base/clear_costmaps");
  resetRoberto.waitForExistence();
  roberto_relocalize = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>("/roberto/initialpose",100,true);
  marvin_relocalize = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>("/marvin/initialpose",100,true);
  marvin_fixed_frame = "marvin/level_mux_map";
  roberto_fixed_frame = "roberto/level_mux_map";
  while(!marvin_ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  while(!roberto_ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  for(int i = 0; i<runs;i++){
  	run();
  	// compute the reward
  	// reward = something

  	// append \phi, reward to running text file
  }
}



void Pass_Hallway::reset_positions(){

  bwi_msgs::RobotTeleporterInterface roberto_rti;
  geometry_msgs::Pose roberto_start;
  roberto_start.position.x = 24;
  roberto_start.position.y = 8.1;//8.45;
  roberto_start.position.z = 0;
  tf::Quaternion roberto_quat;
  roberto_quat.setRPY(0.0, 0.0, 3.1416);
  tf::quaternionTFToMsg(roberto_quat, roberto_start.orientation);
  roberto_rti.request.pose = roberto_start;
  roberto_teleporter_client.call(roberto_rti);

  bwi_msgs::RobotTeleporterInterface marvin_rti;
  geometry_msgs::Pose marvin_start;
  marvin_start.position.x = 16;
  marvin_start.position.y = 8.1;//7.75;
  marvin_start.position.z = 0;
  tf::Quaternion marvin_quat;
  marvin_quat.setRPY(0.0, 0.0, 0.0);
  tf::quaternionTFToMsg(marvin_quat, marvin_start.orientation);
  marvin_rti.request.pose = marvin_start;
  marvin_teleporter_client.call(marvin_rti);
  ros::Duration(0.1).sleep();

  geometry_msgs::PoseWithCovarianceStamped roberto_pose;
  roberto_pose.header.frame_id = roberto_fixed_frame;
  roberto_pose.header.stamp = ros::Time::now();
  roberto_pose.pose.pose.position.x = roberto_start.position.x;
  roberto_pose.pose.pose.position.y = roberto_start.position.y;
  roberto_pose.pose.pose.position.z = roberto_start.position.z;
  tf::quaternionTFToMsg(roberto_quat, roberto_pose.pose.pose.orientation);
  roberto_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
  roberto_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
  roberto_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
  std_srvs::Empty empty2;
  resetRoberto.call(empty2);
  roberto_relocalize.publish(roberto_pose);

  
  geometry_msgs::PoseWithCovarianceStamped marvin_pose;
  marvin_pose.header.frame_id = marvin_fixed_frame;
  marvin_pose.header.stamp = ros::Time::now();
  marvin_pose.pose.pose.position.x = marvin_start.position.x;
  marvin_pose.pose.pose.position.y = marvin_start.position.y;
  marvin_pose.pose.pose.position.z = marvin_start.position.z;
  tf::quaternionTFToMsg(marvin_quat, marvin_pose.pose.pose.orientation);
  marvin_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
  marvin_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
  marvin_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
  std_srvs::Empty empty;
  resetMarvin.call(empty);
  marvin_relocalize.publish(marvin_pose);

  /*roberto_center = false;
  marvin_center = false;*/
}

void Pass_Hallway::stopRobot(Client *client){
  ros::Publisher pub1;
  actionlib_msgs::GoalID msg;
  ROS_INFO("STOPPP");
  msg.id = ""; 
  if(client == marvin_client){
    stop_marvin.publish(msg);
  }else if(client == roberto_client){
    stop_roberto.publish(msg);
  }
  client -> cancelGoal();
}

bool Pass_Hallway::collision(const Point &roberto_odom, const Point &marvin_odom){
  float odom_distance = sqrt(pow(marvin_odom.x-roberto_odom.x,2)+pow(marvin_odom.y-roberto_odom.y,2));
  ROS_INFO_STREAM(odom_distance);
  if(odom_distance>0.01 && odom_distance < 1.0){
  	if(!robotsClose){
  		robotsClose = true;
  		closeStartTime = (float)clock();
  	}
  }else if(odom_distance>2 && odom_distance < 30){
  	robotsClose = false;
  }
  if(robotsClose){
  	if(((float)clock()-closeStartTime)/CLOCKS_PER_SEC > 5){
  		return true;
  	}
  }
  if(roberto_odom.x - marvin_odom.x < -1){
  	successfulPass = true;
  	return true;
  }
  return false;
}

bool Pass_Hallway::backwards(const Point &roberto_odom, const Point &marvin_odom){
	if(roberto_odom.x < 25){
		roberto_center = true;
	}else if(roberto_center){
		return true;
	}
	if(marvin_odom.x > 20){
		marvin_center = true;
	}else if(marvin_center){
		return true;
	}
	return false;
}


void Pass_Hallway::callback(const Odometry::ConstPtr &roberto_odom, const Odometry::ConstPtr &marvin_odom)
{ 
  ROS_INFO("in callback");
  if(collision(roberto_odom->pose.pose.position,marvin_odom->pose.pose.position)){
  	ROS_INFO("stopping");
  	//stopRobot(marvin_client);
  	//stopRobot(roberto_client);
  	reset = true;
  	float timeUsed;
  	if(successfulPass){
  		timeUsed = ((float)clock() - episodeStartTime)/CLOCKS_PER_SEC;
  	}else{
  		timeUsed = 120.0;
  	}
  	myfile << " Time:" << timeUsed << "\n";
  }
  move_base_msgs::MoveBaseGoal marv_goal;
  move_base_msgs::MoveBaseGoal rob_goal;

  marv_goal.target_pose.header.frame_id = "marvin/level_mux_map";
  marv_goal.target_pose.header.stamp = ros::Time::now();

  //marv_goal.target_pose.pose.position.z = 0;
  marv_goal.target_pose.pose.position.x = 25;
  marv_goal.target_pose.pose.position.y = 8.1;

  
  //marv_goal.target_pose.pose.orientation = marvin_odom->pose.pose.orientation;
  tf::Quaternion marvin_quat;
  marvin_quat.setRPY(0.0, 0.0, 0.0);
  tf::quaternionTFToMsg(marvin_quat, marv_goal.target_pose.pose.orientation);
  

  rob_goal.target_pose.header.frame_id = "roberto/level_mux_map";
  rob_goal.target_pose.header.stamp = ros::Time::now();

  //rob_goal.target_pose.pose.position.z = 0;
  rob_goal.target_pose.pose.position.x = 15;
  rob_goal.target_pose.pose.position.y = 8.1;
  //rob_goal.target_pose.pose.orientation = roberto_odom->pose.pose.orientation;
  tf::Quaternion roberto_quat;
  roberto_quat.setRPY(0.0, 0.0, 3.14159);
  tf::quaternionTFToMsg(roberto_quat, rob_goal.target_pose.pose.orientation);
  
  
  if(!reset){
  	marvin_ac.sendGoal(marv_goal);
  	roberto_ac.sendGoal(rob_goal);
  }
  /*while(!collision(roberto_odom->pose.pose.position,marvin_odom->pose.pose.position)){

	if(marvin_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED
		&& roberto_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	  times.push_back((float)(clock()-start)/CLOCKS_PER_SEC);
	  break;
	}
  }*/
}

void Pass_Hallway::get_waypoint()
{
	// generate random waypoint

	// check feasibility with global planner

	// generate corresponding \phi
	//    \phi = [x, y, \theta, ]
	// phi = something

	// waypoint = something
}

void Pass_Hallway::run() 
{
  
  /*ros::ServiceClient robot_clear_client = 
            nh_->serviceClient<std_srvs::Empty>("roberto/move_base/clear_costmaps");
  robot_clear_client.waitForExistence();
  std_srvs::Empty empty;
  robot_clear_client.call(empty);*/
  
  //ros::Duration(2).sleep();

  //roberto_client->stopTrackingGoal();
  //resumeRobot(roberto_client,marvin_door);
  marvin_door = "e3_3";
  roberto_door = "e3_4";
  reset_positions();
  //reset again to ensure proper localization
  reset_positions();

  //calculate waypoints, phi
  get_waypoint();

  reset = false;
  robotsClose = false;
  successfulPass = false;
  episodeStartTime = (float)clock();

  ROS_INFO("in run");
  message_filters::Subscriber<nav_msgs::Odometry> marvin_odom(*nh_, "/marvin/odom", 1);
  message_filters::Subscriber<nav_msgs::Odometry> roberto_odom(*nh_, "/roberto/odom", 1);
  TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> sync(roberto_odom, marvin_odom, 10);
  
  sync.registerCallback(boost::bind(&Pass_Hallway::callback, this, _1, _2));
  //ros::Rate r(20);
  while(ros::ok() && !reset){
  	ros::spinOnce();
  	//r.sleep();
  }
  return;
}


int main(int argc, char**argv) {
  ros::init(argc, argv, "multi_robot_training");
  ROS_INFO("in main");
    // ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandle nh;
  
  Pass_Hallway pass_hallway(&nh,5);
  
  /*for(int i = 0; i<1000;i++){
  	pass_hallway.run();
  }*/
  
  // thread_odom.join();
  
  return 0;
}