
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
#include <sensor_msgs/LaserScan.h>

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
	void readFile();
	void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void callback(const Odometry::ConstPtr &roberto_odom, const Odometry::ConstPtr &marvin_odom);
	bool checkRobotsMovingAway(vector<int> &distances);
	void stopRobot(Client *client);
	void reset_positions();
	bool evaluateRobotDistance(const Point &roberto_odom, const Point &marvin_odom);
	bool backwards(const Point &roberto_odom, const Point &marvin_odom);
	void get_waypoint();
	void get_waypoints();
  
protected: 
	MoveBaseClient marvin_ac;
	MoveBaseClient roberto_ac;
    ros::NodeHandle *nh_;
    Client *roberto_client; 
    Client *marvin_client; 
	//ros::ServiceClient make_plan_client_roberto;
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
    ros::Subscriber marvin_laser;
    bool robotsClose;
    bool reset;
    double episodeStartTime;
    double closeStartTime;
    bool roberto_center;
    bool marvin_center;
    bool successfulPass;
    bool backward;
    bool timeOut;
    bool goalsReached;
    bool collision;
    std::vector<float> robot_;

    float x_hallway_min = 16.0;
    float x_hallway_max = 24.0;
    float y_hallway_min = 6.5;
    float y_hallway_max = 9.25;
    float marvin_final_x = 25.0;
    float marvin_final_y = 8.1;
    float PI = 3.14159;
    float maxes[3];
    float phiSize = 14;
    std::vector<float> phi;
    std::vector<std::vector<float> > phiList;
    //std::vector<float> thetas([0.0, 3.141/2.0, 3.141, 3.0*3.141/2.0])

    geometry_msgs::Pose waypoint;
    sensor_msgs::LaserScan recent;
    //std::vector<float> phi(17);
};

Pass_Hallway::Pass_Hallway( ros::NodeHandle* nh, int runs):marvin_ac("marvin/move_base"),roberto_ac("roberto/move_base")
{
  nh_ = nh;
  /*marvin_client = new Client("/marvin/action_executor/execute_plan", true);
  roberto_client = new Client("/roberto/action_executor/execute_plan", true);
  marvin_client->waitForServer();
  roberto_client->waitForServer();*/
  nh_ -> setParam("roberto/move_base/recovery_behavior_enabled", false);
  nh_ -> setParam("marvin/move_base/recovery_behavior_enabled", false);
  readFile();
  srand(time(NULL));
  myfile.open("testfile.txt");
  if(!myfile.is_open()){
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
  make_plan_client_marvin = nh_->serviceClient<nav_msgs::GetPlan>("marvin/move_base/NavfnROS/make_plan");
  make_plan_client_marvin.waitForExistence();
  while(!marvin_ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  while(!roberto_ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  for(int i = 0; i<runs;i++){
  	ROS_INFO_STREAM(i);
  	run();
  	// compute the reward
  	// reward = something

  	// append \phi, reward to running text file
  }
}

void Pass_Hallway::readFile(){
	ifstream source("phi.txt");
	for(std::string line; std::getline(source, line); )   //read stream line by line
	{
	    std::istringstream in(line);      //make a stream for the line itself
	    std::vector<float> v;
	    for(int i = 0; i<phiSize;i++){
	    	float value;
	    	in >> value;
			v.push_back(value);
		} 
		phiList.push_back(v);
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
  ros::Duration(0.5).sleep();

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

bool Pass_Hallway::evaluateRobotDistance(const Point &roberto_odom, const Point &marvin_odom){
	if(!successfulPass){
		//distance between robot odometries
		float odom_distance = sqrt(pow(marvin_odom.x-roberto_odom.x,2)+pow(marvin_odom.y-roberto_odom.y,2));
		//ROS_INFO_STREAM(odom_distance);
		//set robotsClose based on odom_distance
		if(odom_distance>0.01 && odom_distance < 1.0){
			if(!robotsClose){
				robotsClose = true;
				closeStartTime = ros::Time::now().toSec();
			}
		}else if(odom_distance>2 && odom_distance < 30){
			robotsClose = false;
		}
		//calculate how long the robots have been close 
		if(robotsClose){
			if((ros::Time::now().toSec()-closeStartTime) > 10){
				collision = true;
				return true;
			}
		}
		//detect a pass
		if(roberto_odom.x - marvin_odom.x < -1){
			successfulPass = true;
			marvin_ac.cancelGoalsAtAndBeforeTime(ros::Time::now());
		}
	}
	else{
		//detect if robots have gotten close to the others' starting point
		if(roberto_odom.x < x_hallway_min + 1 && marvin_odom.x > x_hallway_max - 1){
			goalsReached = true;
			return true;
		}
	}
    return false;
}

bool Pass_Hallway::backwards(const Point &roberto_odom, const Point &marvin_odom){
	if(roberto_odom.x < 22){
		roberto_center = true;
	}else if(roberto_center || roberto_odom.x > 25){
		backward = true;
		return true;
	}
	if(marvin_odom.x > 18){
		marvin_center = true;
	}else if(marvin_center || marvin_odom.x < 15){
		backward = true;
		return true;
	}
	if(ros::Time::now().toSec() - episodeStartTime > 50){
		timeOut = true;
		return true;
	}
	return false;
}


void Pass_Hallway::callback(const Odometry::ConstPtr &roberto_odom, const Odometry::ConstPtr &marvin_odom)
{ 
  //ROS_INFO("in callback");
  if(evaluateRobotDistance(roberto_odom->pose.pose.position,marvin_odom->pose.pose.position)
  	||backwards(roberto_odom->pose.pose.position,marvin_odom->pose.pose.position)){
  	ROS_INFO("stopping");
  	marvin_ac.cancelGoalsAtAndBeforeTime(ros::Time::now());
  	roberto_ac.cancelGoalsAtAndBeforeTime(ros::Time::now());
  	//stopRobot(marvin_client);
  	//stopRobot(roberto_client);
  	reset = true;
  	double reward;
  	if(goalsReached){
  		reward = 10.0 - (ros::Time::now().toSec() - episodeStartTime);
  	}else if(timeOut){
  		reward = -1001;
  	}else if(collision){
  		reward = -1000;
  	}else if(backward){
  		reward = -999;
  	}
  	myfile << reward << "\n";
  	ros::Duration(3).sleep();
  }
  move_base_msgs::MoveBaseGoal marv_goal;
  move_base_msgs::MoveBaseGoal rob_goal;

  marv_goal.target_pose.header.frame_id = "marvin/level_mux_map";
  marv_goal.target_pose.header.stamp = ros::Time::now();

  //marv_goal.target_pose.pose.position.z = 0;
  
  if(successfulPass){
  	marv_goal.target_pose.pose.position.x = 25;
    marv_goal.target_pose.pose.position.y = 8.1;
    marv_goal.target_pose.pose.position.z = 0;
    tf::Quaternion marvin_quat;
    marvin_quat.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(marvin_quat, marv_goal.target_pose.pose.orientation);
  }else{
/*  	marv_goal.target_pose.pose.position.x = 16;
    marv_goal.target_pose.pose.position.y = 6.5;
    marv_goal.target_pose.pose.position.z = 0;
    tf::Quaternion marvin_quat;
    marvin_quat.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(marvin_quat, marv_goal.target_pose.pose.orientation);
  */
    marv_goal.target_pose.pose = waypoint;
    ROS_INFO_STREAM("x:" << waypoint.position.x << "y: " << waypoint.position.y);
  }
  

  
  //marv_goal.target_pose.pose.orientation = marvin_odom->pose.pose.orientation;

  rob_goal.target_pose.header.frame_id = "roberto/level_mux_map";
  rob_goal.target_pose.header.stamp = ros::Time::now();

  //rob_goal.target_pose.pose.position.z = 0;
  if(!roberto_odom->pose.pose.position.x < 17){
	rob_goal.target_pose.pose.position.x = 14;
  }else{
  	rob_goal.target_pose.pose.position.x = roberto_odom->pose.pose.position.x-2;
  }
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

void Pass_Hallway::laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
	float increment = scan->angle_increment;
	ROS_INFO_STREAM(increment);
	int points = (PI/4)/increment;
	ROS_INFO_STREAM(points);
	int index = 0;
	for(int i = 0; i<3;i++){
		float min = 30.0;
		for(int j = 0;j<points;j++){
			//ROS_INFO_STREAM(scan->ranges[index]);
			if(scan->ranges[index]<min){
				min = scan->ranges[index];
			}
			index++;
		}
		maxes[i] = min;
	}
}

void Pass_Hallway::get_waypoint(){
	int index = rand() % phiList.size();
	tf::Quaternion goal_quat;
	goal_quat.setRPY(0.0, 0.0, phiList.at(index).at(2));
	tf::quaternionTFToMsg(goal_quat, waypoint.orientation);
	waypoint.position.x = phiList.at(index).at(0);
	waypoint.position.y = phiList.at(index).at(1);
	waypoint.position.z = 0; 
	for(int i = 0; i<phiSize;i++){
		myfile << phiList.at(index).at(i) << " ";
	}
	myfile << "\n";
}

void Pass_Hallway::get_waypoints()
{
	int gridHeight = 5;
	int gridWidth = 5;
	//don't consider waypoints more than one meter past the midpoint
	float hallwayWidth = (x_hallway_max-x_hallway_min)/2 + 1;
	for(int i = 0; i <= gridWidth; i++){
		float x = (i * hallwayWidth/gridWidth) + x_hallway_min;
		for(int j = 0; j <= gridHeight; j++){
			float y = (j * (y_hallway_max-y_hallway_min)/gridHeight) + y_hallway_min;
			GetPlan srv;
			ROS_INFO_STREAM("x: " << x <<"y: "<<y);
		    geometry_msgs::PoseStamped &start = srv.request.start;
		    geometry_msgs::PoseStamped &goal = srv.request.goal;
		    start.header.frame_id = goal.header.frame_id = marvin_fixed_frame;
			start.header.stamp = goal.header.stamp = ros::Time::now();

			start.pose.position.x = x_hallway_min;
			start.pose.position.y = 8.1;
			start.pose.position.z = 0;
			tf::Quaternion start_quat;
			start_quat.setRPY(0.0, 0.0, 0.0);
			tf::quaternionTFToMsg(start_quat, start.pose.orientation);
			tf::Quaternion goal_quat;
			goal_quat.setRPY(0.0, 0.0, -1.309);
			tf::quaternionTFToMsg(goal_quat, goal.pose.orientation);
			goal.pose.position.x = x;
			goal.pose.position.y = y;
			goal.pose.position.z = 0; 
			srv.request.tolerance = 0.5 + 1e-6;

			if (make_plan_client_marvin.call(srv)) {
			    if (srv.response.plan.poses.size() != 0) {
		 			ROS_INFO("valid");
		 			float marvin_distance = sqrt(pow(16-x,2)+pow(8.1-y,2));
		 			float roberto_distance = sqrt(pow(24-x,2)+pow(8.1-y,2));
		 			float ip_distance = sqrt(pow(20-x,2)+pow(8.1-y,2));
					phi.push_back(x);
					phi.push_back(y);
					phi.push_back(-(PI/4));
					phi.push_back(marvin_distance);
					phi.push_back(roberto_distance);
					phi.push_back(ip_distance);
		 			move_base_msgs::MoveBaseGoal marv_goal;

					marv_goal.target_pose.header.frame_id = "marvin/level_mux_map";
					marv_goal.target_pose.header.stamp = ros::Time::now();
					marv_goal.target_pose.pose = goal.pose;
					marvin_ac.sendGoalAndWait(marv_goal);

					marvin_laser = nh_ -> subscribe("marvin/scan", 1, &Pass_Hallway::laser_callback,this);
					ros::Duration(1.5).sleep();
					ros::spinOnce();
					ros::Duration(1.5).sleep();
					phi.push_back(maxes[0]);
					phi.push_back(maxes[1]);
					phi.push_back(maxes[2]);
					goal_quat.setRPY(0.0, 0.0, -(3*PI/4) - 1.309);
					tf::quaternionTFToMsg(goal_quat, goal.pose.orientation);
					marv_goal.target_pose.pose = goal.pose;
					marvin_ac.sendGoalAndWait(marv_goal);
					ros::Duration(1.5).sleep();
					ros::spinOnce();
					ros::Duration(1.5).sleep();
					phi.push_back(maxes[0]);
					phi.push_back(maxes[1]);
					phi.push_back(maxes[2]);
					goal_quat.setRPY(0.0, 0.0, -(6*PI/4) - 1.309);
					tf::quaternionTFToMsg(goal_quat, goal.pose.orientation);
					marv_goal.target_pose.pose = goal.pose;
					marvin_ac.sendGoalAndWait(marv_goal);
					ros::Duration(1.5).sleep();
					ros::spinOnce();
					ros::Duration(1.5).sleep();
					phi.push_back(maxes[0]);
					phi.push_back(maxes[1]);
					for(int k = 0;k<phi.size();k++){
						myfile << phi[k] << " ";
					}
					myfile<<"\n";
					phi[2] = 0;
					for(int k = 0;k<phi.size();k++){
						myfile << phi[k] << " ";
					}
					myfile<<"\n";
					phi[2] = PI/4;
					for(int k = 0;k<phi.size();k++){
						myfile << phi[k] << " ";
					}
					myfile<<"\n";
					phi.clear();
		 			//valid = true;
			    } else {        
			        ROS_INFO_STREAM("not valid");
			        //myfile<<"empty plan\n";
			    }
			} else {
			    ROS_INFO_STREAM("service failed");
			}
		}
	}
/*	bool valid = false;
	while(!valid){
		int gridHeight = 5;
		int gridWidth = 5;
		int randH = rand() % (gridHeight+1);
		int randW = rand() % (gridWidth+1);
		float localY = randH * (y_hallway_max-y_hallway_min)/gridHeight;
		//don't consider waypoints more than one meter past the midpoint
		float hallwayWidth = (x_hallway_max-x_hallway_min)/2 + 1;
		float localX = randW * hallwayWidth/gridWidth;
		float globalY = localY + y_hallway_min;
		float globalX = localX + x_hallway_min;
		GetPlan srv;
	    geometry_msgs::PoseStamped &start = srv.request.start;
	    geometry_msgs::PoseStamped &goal = srv.request.goal;
	    start.header.frame_id = goal.header.frame_id = marvin_fixed_frame;
		start.header.stamp = goal.header.stamp = ros::Time::now();

		start.pose.position.x = x_hallway_min;
		start.pose.position.y = 8.1;
		start.pose.position.z = 0;
		tf::Quaternion start_quat;
		start_quat.setRPY(0.0, 0.0, 0.0);
		tf::quaternionTFToMsg(start_quat, start.pose.orientation);

		//generate random yaw
		tf::Quaternion goal_quat;
		float yaw = (rand()%4) * (3.14159/2);
		goal_quat.setRPY(0.0, 0.0, yaw);
		tf::quaternionTFToMsg(goal_quat, waypoint.orientation);
		waypoint.position.x = globalX;
		waypoint.position.y = globalY;
		waypoint.position.z = 0; 
		goal.pose = waypoint;
		/*tf::quaternionTFToMsg(goal_quat, goal.pose.orientation);

		goal.pose.position.x = globalX;
		goal.pose.position.y = globalY;
		goal.pose.position.z = 0;
		srv.request.tolerance = 0.5 + 1e-6;

		if (make_plan_client_marvin.call(srv)) {
		    if (srv.response.plan.poses.size() != 0) {
	 			ROS_INFO("valid");
	 			valid = true;
		    } else {        
		        ROS_INFO_STREAM("not valid");
		        //myfile<<"empty plan\n";
		    }
		} else {
		    ROS_INFO_STREAM("service failed");
		}
	}
	*/
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

 
  reset_positions();
  //reset again to ensure proper localization
  reset_positions();
  ros::Duration(1).sleep();
  //calculate waypoints, phi
  get_waypoint();

  reset = false;
  robotsClose = false;
  successfulPass = false;
  collision = false;
  goalsReached = false;
  roberto_center = false;
  marvin_center = false;
  time_out = false;
  episodeStartTime = ros::Time::now().toSec();

  ROS_INFO("in run");
  message_filters::Subscriber<nav_msgs::Odometry> marvin_odom(*nh_, "/marvin/odom", 1);
  message_filters::Subscriber<nav_msgs::Odometry> roberto_odom(*nh_, "/roberto/odom", 1);
  TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> sync(roberto_odom, marvin_odom, 10);
  
  sync.registerCallback(boost::bind(&Pass_Hallway::callback, this, _1, _2));
  ros::Rate r(40);
  while(ros::ok() && !reset){
  	ros::spinOnce();
  	r.sleep();
  }
  return;
}


int main(int argc, char**argv) {
  ros::init(argc, argv, "multi_robot_training");
  ROS_INFO("in main");
    // ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandle nh;
  
  Pass_Hallway pass_hallway(&nh,500);
  
  /*for(int i = 0; i<1000;i++){
  	pass_hallway.run();
  }*/
  
  // thread_odom.join();
  
  return 0;
}