#include <actionlib/client/simple_action_client.h>
#include <plan_execution/ExecutePlanAction.h>
#include <bwi_msgs/LogicalActionAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <bwi_msgs/RobotTeleporterInterface.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <bwi_planning_common/structures.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <boost/circular_buffer.hpp>
#include <std_srvs/Empty.h>
//#include <random>
typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> ASPClient;
typedef actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction> LAClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

bool DEBUG = false;

bool REVERSE = true;
string ROBOT_NAME = "/roberto";
string HOST_NAME  = "/brian";
int buffer_size = 10;

class safetyProtocol{
protected:
  ASPClient *client_;
  LAClient *lac_;
  ros::Publisher stop;
  ros::Rate *stop_rate;
  MoveBaseClient roberto_ac;
  ros::NodeHandle nh_;
  vector<geometry_msgs::Pose> myPlan_;
  vector<geometry_msgs::Pose> othersPlan_;
  boost::circular_buffer<geometry_msgs::Pose> othersHistory_;
  ros::Subscriber subscribeMyPlan_;
  ros::Subscriber subscribeOthersPlan_;
  ros::AsyncSpinner *spinner;

  map<string, geometry_msgs::Pose> parkingZones_;
  ros::ServiceClient makePlanService;
  ros::ServiceClient roberto_teleporter_client;
  ros::ServiceClient resetRoberto;
  ros::Publisher roberto_relocalize;
  string global_frame_id_;
  // private functions
  void subscribeMyPlan(const visualization_msgs::MarkerArray plan);
  void subscribeOthersPlan(const visualization_msgs::MarkerArray plan);
  float dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt);
  float distance(const geometry_msgs::Point start_pose, const geometry_msgs::Point goal_pose);
  string findSafeZone();
  bool waitUntilSafe();
  bool distanceBaseAlarm(float warningDistance);
public:
  safetyProtocol():roberto_ac(""){}
  safetyProtocol(string ROBOT_NAME);
  void stopASPPlan();
  void goToSafeZone(string location);
  void resume(string location);
  void run(string location);
  void reset_position(float x, float y, float theta);
  void goTo(float x, float y, float theta);
  void sample_points(int num, float xmin, float xmax, float ymin, float ymax);
  bool isClientArrive(){
    if(this->client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) return true;
    return false;
  }
};
safetyProtocol::safetyProtocol(string ROBOT_NAME):roberto_ac("roberto/move_base"){
  this->client_ = new ASPClient(ROBOT_NAME + "/action_executor/execute_plan", true);
  this->lac_ = new LAClient(ROBOT_NAME + "/execute_logical_goal", true);
  stop = nh_.advertise<actionlib_msgs::GoalID>(ROBOT_NAME + "/move_base/cancel", 1, true);
  this->makePlanService = nh_.serviceClient<nav_msgs::GetPlan>(ROBOT_NAME + "/move_base/NavfnROS/make_plan");
  this->stop_rate = new ros::Rate(100);

  this->othersHistory_ = boost::circular_buffer<geometry_msgs::Pose>(buffer_size);

  this->client_->waitForServer();
  this->lac_->waitForServer();
  this->makePlanService.waitForExistence();

  //services for resetting
  this->roberto_teleporter_client = nh_.serviceClient<bwi_msgs::RobotTeleporterInterface>("roberto/teleport_robot");
  this->resetRoberto = nh_.serviceClient<std_srvs::Empty>("roberto/move_base/clear_costmaps");
  this->roberto_relocalize = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/roberto/initialpose",100,true);
  

  string commonEBand = "/move_base/EBandPlannerROS/eband_visualization_array";
  string myEBandTopic = ROBOT_NAME + commonEBand;

  string othersEBandTopic = (ROBOT_NAME==""?"/others":"/marvin") + commonEBand;
  this->subscribeMyPlan_ = nh_.subscribe(myEBandTopic, 1, &safetyProtocol::subscribeMyPlan, this);
  this->subscribeOthersPlan_ = nh_.subscribe(othersEBandTopic, 1, &safetyProtocol::subscribeOthersPlan, this);
 this->spinner = new ros::AsyncSpinner(2);
  this->spinner->start();

  // Read predefined parking zones
  string loc_file = "/home" + HOST_NAME + "/catkin_ws/src/bwi_common/utexas_gdc/maps/simulation/3ne/objects.yaml";
  bwi_planning_common::readObjectApproachFile(loc_file, parkingZones_);

  ROS_INFO_STREAM("Number of predefined parking zones: " << parkingZones_.size());
  for(map<string, geometry_msgs::Pose>::iterator iter = parkingZones_.begin(); iter != parkingZones_.end(); iter++) ROS_INFO_STREAM(iter->first);

  if(ROBOT_NAME == "") global_frame_id_ = "level_mux_map";
  else global_frame_id_ = "roberto/level_mux_map";
};
void safetyProtocol::subscribeMyPlan(const visualization_msgs::MarkerArray plan){
  this->myPlan_.clear();
  for(int i=0; i<plan.markers.size(); i++){
    this->myPlan_.push_back(plan.markers[i].pose);
  }
};
void safetyProtocol::subscribeOthersPlan(const visualization_msgs::MarkerArray plan){
  this->othersPlan_.clear();
  for(int i=0; i<plan.markers.size(); i++) othersPlan_.push_back(plan.markers[i].pose);
  // Push current other robot's pose to history
  if(this->othersHistory_.size()==0){
    this->othersHistory_.push_back(plan.markers[0].pose);
  }
  else{
    if(this->dist(this->othersHistory_[0].position, plan.markers[0].pose.position)>0.5){
      this->othersHistory_.push_back(plan.markers[0].pose);
    }
  }
};
float safetyProtocol::dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt){
  return hypot(my_plan_pt.x - others_plan_pt.x, my_plan_pt.y - others_plan_pt.y);
};
bool safetyProtocol::distanceBaseAlarm(float warningDistance = 0.6){
  bool danger = false;
  if(myPlan_.size() == 0 || othersPlan_.size() == 0) return danger;

  for(int i=0; i<myPlan_.size()||i<othersPlan_.size(); i++){
    if(i<myPlan_.size()){
      if(dist(myPlan_[i].position, othersPlan_[0].position) < warningDistance){
        ROS_INFO_STREAM("Roberto--->Marvin");
        ROS_INFO_STREAM("my pose : ("<<myPlan_[i].position.x<<", "<<myPlan_[i].position.y<<")");
        ROS_INFO_STREAM("others pose : ("<<othersPlan_[0].position.x<<", "<<othersPlan_[0].position.y<<")");
        danger = true;
        return danger;
      }
    }
    if(i<othersPlan_.size()){
      if(dist(myPlan_[0].position, othersPlan_[i].position) < warningDistance){
        ROS_INFO_STREAM("Roberto<---Marvin");
        ROS_INFO_STREAM("my pose : ("<<myPlan_[0].position.x<<", "<<myPlan_[0].position.y<<")");
        ROS_INFO_STREAM("others pose : ("<<othersPlan_[i].position.x<<", "<<othersPlan_[i].position.y<<")");
        danger = true;
        return danger;
      }
    }
    if(i<myPlan_.size() && i<othersPlan_.size()){
      if(dist(myPlan_[i].position, othersPlan_[i].position) < warningDistance){
        ROS_INFO_STREAM("Roberto-><-Marvin");
        ROS_INFO_STREAM("my pose : ("<<myPlan_[i].position.x<<", "<<myPlan_[i].position.y<<")");
        ROS_INFO_STREAM("others pose : ("<<othersPlan_[i].position.x<<", "<<othersPlan_[i].position.y<<")");
        danger = true;
        return danger;
      }
    }
  }

  return danger;
};
void safetyProtocol::stopASPPlan(){
  // cancel current ASP planner
  this->client_->cancelGoal();
  // Manually stop the move base.
  actionlib_msgs::GoalID stop_msg;
  stop_msg.id = "";
  stop_rate->sleep();
  stop.publish(stop_msg);
};
float safetyProtocol::distance(geometry_msgs::Point start_pt, geometry_msgs::Point goal_pt){
  float distance = 0;
  nav_msgs::GetPlan srv;
  geometry_msgs::PoseStamped &start = srv.request.start;
  geometry_msgs::PoseStamped &goal  = srv.request.goal;
  // set GetPlan service parameters
  start.header.frame_id = goal.header.frame_id = global_frame_id_;
  start.header.stamp    = goal.header.stamp    = ros::Time::now();

  start.pose.position.x = start_pt.x;
  start.pose.position.y = start_pt.y;
  start.pose.position.z = 0;
  start.pose.orientation.w = 1.0;

  goal.pose.position.x = goal_pt.x;
  goal.pose.position.y = goal_pt.y;
  goal.pose.position.z = 0;
  goal.pose.orientation.w = 1.0;

  srv.request.tolerance = 0.5 + 1e-6;

  geometry_msgs::Pose prev_pose = start.pose;
  geometry_msgs::Pose curr_pose = start.pose;

  if(makePlanService.call(srv)) {
    if(srv.response.plan.poses.size() != 0){
      for(int i = 0 ; i<srv.response.plan.poses.size() ; i++){
        curr_pose = srv.response.plan.poses[i].pose;
        if(abs(prev_pose.position.x - curr_pose.position.x) > 1 || abs(prev_pose.position.y - curr_pose.position.y)>1){
          distance = distance + dist(prev_pose.position, curr_pose.position);
          prev_pose = curr_pose;
        }
      }
      distance = distance + dist(prev_pose.position, curr_pose.position);
    }else{
      if(DEBUG) ROS_INFO_STREAM("No plan available. Return Euclidian distance");
      distance = dist(start_pt, goal_pt);
    }
  }
  else{
    ROS_INFO_STREAM("Planning failed!");
  }
  return distance;
}
string safetyProtocol::findSafeZone(){
  string safeZone = "empty";
  string nearZone = "empty";
  float safeDist = 999.f;
  float nearDist = 999.f;

  map<string, geometry_msgs::Pose>::iterator iter;
  for(iter = parkingZones_.begin(); iter != parkingZones_.end(); iter++){
    float my_distance_to_safe_zone = distance(myPlan_.begin()->position, iter->second.position);
    float others_distance_to_safe_zone = distance(othersPlan_.begin()->position, iter->second.position);
    if(my_distance_to_safe_zone < others_distance_to_safe_zone && my_distance_to_safe_zone < safeDist){
      safeDist = my_distance_to_safe_zone;
      safeZone = iter->first;
    }
    if(my_distance_to_safe_zone < nearDist){
      nearDist = my_distance_to_safe_zone;
      nearZone = iter->first;
    }
  }
  if(safeZone == "empty"){
    safeZone = nearZone;
  }
  ROS_INFO_STREAM("nearest Safe Zone : "<<safeZone);
  return safeZone;
};
void safetyProtocol::resume(string location){
  ROS_INFO_STREAM("resume to "<<location);
  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;

  fluent.name = "not facing";
  fluent.variables.push_back(location);
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);

  this->client_->sendGoal(goal);
};
void safetyProtocol::goToSafeZone(string safeZone){
  // safeZone must be defined as object.
  vector<string> parameters;
  bwi_msgs::LogicalActionGoal goal;

  goal.command.name = "goto";
  parameters.push_back(safeZone);
  parameters.push_back("1");
  goal.command.value = parameters;

  this->lac_->sendGoal(goal);
  parameters.clear();
};
// Original waitUntilSafe module
/*
bool safetyProtocol::waitUntilSafe(){
  // return true if it is safe, otherwise return false;
  bool safe = false;
  // if robots are 5 meters away
  vector<geometry_msgs::Pose>::iterator my_iter = myPlan_.begin();
  vector<geometry_msgs::Pose>::iterator others_iter = othersPlan_.end();

  float distance_between_robot = dist(my_iter->position, (--others_iter)->position);
  ////ROS_INFO_STREAM("DEBUG : "<<distance_between_robot);
  ////ROS_INFO_STREAM("DEBUG : "<<othersPlan_.size()<<" - other robot's plan size");
  //ROS_INFO_STREAM("waitUntilSafe init");
  if(distance_between_robot > 8.0f){
    ROS_INFO_STREAM("Robots are 8.0 M away!");
    safe = true;
    return safe;
  }
  //ROS_INFO_STREAM("waitUntilSafe distance between robots");
  if(othersPlan_.size() == 2){
    // Other robot is stopped and planning for next location.
    return safe;
  }
  // if other robot is moving away.
  bool isMovingAway = true;

  others_iter = othersPlan_.begin();
  float prev_dist = dist(my_iter->position, others_iter->position);
  float curr_dist;
  others_iter = ++others_iter;
  //ROS_INFO_STREAM("waitUntilSafe far away init");
  //ROS_INFO_STREAM("Other plan size: "<<othersPlan_.size());
  int index = 1;
  for(; others_iter < othersPlan_.end(); others_iter++){
    // write distance
    //ROS_INFO_STREAM("index : "<<index);
    index = index+1;
    curr_dist = dist(my_iter->position, others_iter->position);
    if(curr_dist<prev_dist){
      isMovingAway = false;
      break;
    }
  }
  //ROS_INFO_STREAM("waitUntilSafe far away finished");
  if(isMovingAway == true){
    ROS_INFO_STREAM("Other robot is definately planning to move away!");
    safe = true;
  }

  return safe;
};
*/
bool safetyProtocol::waitUntilSafe(){
  // return true if it is safe, otherwise return false;
  bool safe = false;
  // if robots are 5 meters away
  vector<geometry_msgs::Pose>::iterator my_iter = myPlan_.begin();
  vector<geometry_msgs::Pose>::iterator others_iter = othersPlan_.end();

  // if distance between me and future plan of other robot is 8.0m away, resume.
  float distance_between_robot = dist(my_iter->position, (--others_iter)->position);
  if(distance_between_robot > 8.0f){
    ROS_INFO_STREAM("Robots are 8.0 M away!");
    safe = true;
    return safe;
  }
  // If other robot is stopped and planning for next location, it is not safe.
  if(othersPlan_.size() == 2){
    return safe;
  }
  // check if other robot is moving away.
  bool isMovingAway = false;

  int idx = 0;
  float prev_dist = dist(my_iter->position, this->othersHistory_[idx].position);
  float curr_dist;
  idx++;

  for(; idx < buffer_size; idx++){
    // write distance
    curr_dist = dist(my_iter->position, this->othersHistory_[idx].position);
    if(curr_dist>prev_dist){
      isMovingAway = true;
      break;
    }
  }
  //ROS_INFO_STREAM("waitUntilSafe far away finished");
  if(isMovingAway == true){
    ROS_INFO_STREAM("Other robot is definately planning to move away!");
    safe = true;
  }

  return safe;
};

void safetyProtocol::run(string location){
  //ROS_INFO_STREAM("Initiate Safety Protocol");
  bool danger = distanceBaseAlarm();
  if(danger == true){
    //stop plan
    //ROS_INFO_STREAM("Stop plan");
    stopASPPlan();
    // find near safezone
    string safeZone;
    safeZone = findSafeZone();
    // move to safe zone
    ROS_INFO_STREAM("Going to safe zone");
    goToSafeZone(safeZone);
    if(DEBUG) ROS_INFO_STREAM("DEBUG 1");
    // Decide when to resume robot.
    //double stime = clock();
    while(ros::ok()){
      //if((clock()-stime)/CLOCKS_PER_SEC > 600) break;
      // Handel ABORTED from bwi logical action
      if(DEBUG) ROS_INFO_STREAM("DEBUE 1-1");
      if(this->lac_->getState() == actionlib::SimpleClientGoalState::ABORTED){
        ROS_INFO_STREAM("Execute Logical Goal failed.");
        ROS_INFO_STREAM("Re-executing.");
        goToSafeZone(safeZone);
      }
      if(DEBUG) ROS_INFO_STREAM("DEBUE 1-2");
      //wait until it is safe.
      if(waitUntilSafe()) break;
      stop_rate->sleep();
    }
    if(DEBUG) ROS_INFO_STREAM("DEBUG 2");
    // resume to original goal
    if(this->lac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
      this->lac_->cancelGoal();
    }
    if(DEBUG) ROS_INFO_STREAM("DEBUG 3");
    resume(location);
    while(ros::ok()){
      if(this->client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) break;
      ROS_INFO_STREAM("waiting for robot to resume");
      this->stop_rate->sleep();
    }
    if(DEBUG) ROS_INFO_STREAM("DEBUG 4");
  }
  for(int i=0; i<10; i++) this->stop_rate->sleep();
  return;
};

void safetyProtocol::goTo(float x, float y, float theta){
  ROS_INFO_STREAM("Going to "<< x << "," << y);
  move_base_msgs::MoveBaseGoal rob_goal;
  rob_goal.target_pose.header.frame_id = global_frame_id_;
  rob_goal.target_pose.header.stamp = ros::Time::now();
  rob_goal.target_pose.pose.position.x = x;
  rob_goal.target_pose.pose.position.y = y;
  tf::Quaternion roberto_quat;
  roberto_quat.setRPY(0.0, 0.0, theta);
  tf::quaternionTFToMsg(roberto_quat, rob_goal.target_pose.pose.orientation);
  roberto_ac.sendGoal(rob_goal);
};

void safetyProtocol::reset_position(float x, float y, float theta){
  bwi_msgs::RobotTeleporterInterface roberto_rti;
  geometry_msgs::Pose roberto_start;
  roberto_start.position.x = x;
  roberto_start.position.y = y;
  roberto_start.position.z = 0;
  tf::Quaternion roberto_quat;
  roberto_quat.setRPY(0.0, 0.0, theta);
  tf::quaternionTFToMsg(roberto_quat, roberto_start.orientation);
  roberto_rti.request.pose = roberto_start;
  roberto_teleporter_client.call(roberto_rti);
  ros::Duration(0.5).sleep();

  geometry_msgs::PoseWithCovarianceStamped roberto_pose;
  roberto_pose.header.frame_id = global_frame_id_;
  roberto_pose.header.stamp = ros::Time::now();
  roberto_pose.pose.pose.position.x = roberto_start.position.x;
  roberto_pose.pose.pose.position.y = roberto_start.position.y;
  roberto_pose.pose.pose.position.z = roberto_start.position.z;
  tf::quaternionTFToMsg(roberto_quat, roberto_pose.pose.pose.orientation);
  roberto_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
  roberto_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
  roberto_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
  std_srvs::Empty empty;
  roberto_relocalize.publish(roberto_pose);
  resetRoberto.call(empty);
  //ros::Duration(0.5).sleep();
};

void safetyProtocol::sample_points(int num, float xmin, float xmax, float ymin, float ymax){
  while(num>0){
    //uniform random, only compiles in C++11
    /*std::default_random_engine generator;
    std::uniform_real_distribution<double> x_distribution(xmin,xmax);
    std::uniform_real_distribution<double> y_distribution(ymin,ymax);
    float x = x_distribution(generator);
    float y = y_distribution(generator);*/

    //close to but not fully uniformly random
    float x = (float)rand() / RAND_MAX;
    x = x * (xmax-xmin) + xmin;
    float y = (float)rand() / RAND_MAX;
    y = y * (ymax-ymin) + ymin;

    nav_msgs::GetPlan srv;
    geometry_msgs::PoseStamped &start = srv.request.start;
    geometry_msgs::PoseStamped &goal = srv.request.goal;
    start.header.frame_id = goal.header.frame_id = global_frame_id_;
    start.header.stamp = goal.header.stamp = ros::Time::now();
    start.pose.position.x = 10; //use current coord instead, maybe from odom if in sim
    start.pose.position.y = 12;
    start.pose.position.z = 0;
    tf::Quaternion start_quat;
    start_quat.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(start_quat, start.pose.orientation);

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0; 
    tf::Quaternion goal_quat;
    goal_quat.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(goal_quat, goal.pose.orientation);
    srv.request.tolerance = 0.5 + 1e-6;

    if(makePlanService.call(srv)) {
      if(srv.response.plan.poses.size() != 0){
        num--;
        ROS_INFO_STREAM("Valid x:" << x << "y: " << y);
      }else{
        ROS_INFO_STREAM("Not valid");
      }
    }
  }    
};
int main(int argc, char** argv){
  ros::init(argc, argv, "sample_visit_door_safely");

  safetyProtocol sp(ROBOT_NAME);
  /**
  // initialize door list
  vector<string> doors;
  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  doors.push_back("d3_414a2");
  doors.push_back("d3_418");
  int door = 0;

  sp.goTo(doors.at(door));

  while(ros::ok()){
    sp.run(doors.at(door));
    if(DEBUG) ROS_INFO_STREAM("main DEBUG 1");
    if(sp.isClientArrive()){
      if(REVERSE) door = (door?door:doors.size())-1;
      else        door = (door+1)%doors.size();
      //ROS_INFO_STREAM("-1 < " << door << " < " << doors.size());

      if(DEBUG) ROS_INFO_STREAM("main DEBUG 2-1");
      sp.goTo(doors.at(door));
    }
    if(DEBUG) ROS_INFO_STREAM("main DEBUG 2-2");
  }
  **/
  sp.reset_position(10,12,0.0);

  //sp.goTo(14.04,14.5,0.0);
  sp.sample_points(20,10,15,9,15);
}
