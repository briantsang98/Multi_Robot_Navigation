
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

string global_frame_id_ = "roberto/level_mux_map";


class Visit_Door {
public:
  Visit_Door(ros::NodeHandle* nh);
  void run();
  void callback(const Odometry::ConstPtr &roberto_odom, const Odometry::ConstPtr &marvin_odom);
  bool checkForInterruption(const Odometry::ConstPtr &roberto_odom, const Odometry::ConstPtr &marvin_odom);
  string getNearestParkingZone(const geometry_msgs::Point &ip);
  void goToParkingZone(string ParkingZone, Client *client, bool &interrupt_flag);
  bool safeInteraction(const Point &pt, const Point &roberto_odom, const Point &marvin_odom);
  vector<geometry_msgs::Point> getInteractionPoints(const Point &start_pt, const Point &goal_pt);
  geometry_msgs::Point getIteractionPoint(const Odometry::ConstPtr &roberto_odom, const Odometry::ConstPtr &marvin_odom);  
  bool checkRobotsMovingCloser(vector<int> &distances);
  bool checkRobotsMovingAway(vector<int> &distances);
  void stopRobot(Client *client);
  void resumeRobot(Client *client, int door);

protected: 
    ros::NodeHandle *nh_;
    Client *roberto_client; 
    Client *marvin_client; 
    bool interrupt_flag;
    std::map<std::string, geometry_msgs::Pose> ParkingZones;
    ros::ServiceClient make_plan_client_roberto;
    ros::ServiceClient make_plan_client_marvin;
    vector<int>robot_distances;
    int marvin_door;
    int roberto_door;
    std::vector<string> doors;
    ros::Publisher stop_roberto;
    ros::Publisher stop_marvin;
    std::vector<geometry_msgs::Point> latestIP;
    bool marvin_parked;
    bool roberto_parked;
    Client *parkedRobot;
    int savedGoal;
    std::vector<string> notSafe;
    ofstream myfile;
    string parkedLocation;
    std::vector<float> robot_odom_distances;
};

Visit_Door::Visit_Door( ros::NodeHandle* nh )
{
  nh_ = nh;
  marvin_client = new Client("/marvin/action_executor/execute_plan", true);
  roberto_client = new Client("/roberto/action_executor/execute_plan", true);
  interrupt_flag = true;
  marvin_parked = false;
  roberto_parked = false;
  myfile.open("testfile.txt");
  if(myfile.is_open()){
    myfile<<"hi\n";
  }else{
    ROS_INFO("not open");
    exit(1);
  }
  //printer parking zones not safe to pass through
  notSafe.push_back("p3_15");
  notSafe.push_back("p3_16");
  stop_roberto = nh_->advertise<actionlib_msgs::GoalID> 
        ("/roberto/move_base/cancel", 1000, true); 
  stop_marvin = nh_->advertise<actionlib_msgs::GoalID> 
      ("/marvin/move_base/cancel", 1000, true); 
  //std::string data_directory = "/home/users/NI2452/catkin_ws/src/bwi_common/utexas_gdc/maps/simulation/3ne";
  std::string data_directory = "/home/brian/catkin_ws/src/bwi_common/utexas_gdc/maps/simulation/3ne";
  std::string loc_file = bwi_planning_common::getObjectsFileLocationFromDataDirectory(data_directory);
  ROS_INFO_STREAM("Reading ParkingZones file: " + loc_file);
  bwi_planning_common::readObjectApproachFile(loc_file, ParkingZones);
  ROS_INFO("ParkingZones size:%d", ParkingZones.size());
  map<string, geometry_msgs::Pose>::iterator it;
  for(it = ParkingZones.begin();it != ParkingZones.end();it++){
    if(it->first.at(0) == 'r'){
      ParkingZones.erase(it);
      --it;
    }else{
      ROS_INFO_STREAM(it->first);
    }
  }
}

std::vector<geometry_msgs::Point> Visit_Door::getInteractionPoints(const geometry_msgs::Point &start_pt, const geometry_msgs::Point &goal_pt){
  std::vector<geometry_msgs::Point> interaction_points;
  GetPlan srv;
  geometry_msgs::PoseStamped &start = srv.request.start;
  geometry_msgs::PoseStamped &goal = srv.request.goal;

  start.header.frame_id = goal.header.frame_id = global_frame_id_;
  start.header.stamp = goal.header.stamp = ros::Time::now();

  start.pose.position.x = start_pt.x;
  start.pose.position.y = start_pt.y;
  start.pose.position.z = 0;
  start.pose.orientation.w = 1.0;

  goal.pose.position.x = goal_pt.x;
  goal.pose.position.y = goal_pt.y;
  goal.pose.position.z = 0;
  goal.pose.orientation.w = 1.0;
  srv.request.tolerance = 0.5 + 1e-6;

  interaction_points.push_back(start.pose.position);

  if (!make_plan_client_roberto) {
    ROS_INFO("Persistent service connection to failed");
  }
  bool make_plan_client_initialized_ = false;
  if (!make_plan_client_initialized_) {    
    make_plan_client_roberto = nh_->serviceClient<nav_msgs::GetPlan>("roberto/move_base/NavfnROS/make_plan");
    make_plan_client_roberto.waitForExistence();  
    make_plan_client_initialized_ = true;
  }
  float old_pt_x, old_pt_y, new_pt_x, new_pt_y;
  if (make_plan_client_roberto.call(srv)) {
    if (srv.response.plan.poses.size() != 0) {
      for(int i = 0 ; i< srv.response.plan.poses.size() ; i++){
        geometry_msgs::PoseStamped p = srv.response.plan.poses[i];        
        old_pt_x = interaction_points.back().x;
        old_pt_y = interaction_points.back().y;
        new_pt_x = p.pose.position.x;
        new_pt_y = p.pose.position.y;
        if(abs(new_pt_x - old_pt_x)>1 || abs(new_pt_y-old_pt_y)>1){
          interaction_points.push_back(p.pose.position);        }        
      }  
    } else {        
      ROS_INFO_STREAM("Empty plan");
      myfile<<"empty plan\n";
    }
  } else {
    ROS_INFO_STREAM("service failed");
  }
  return interaction_points;
}
geometry_msgs::Point Visit_Door::getIteractionPoint(const Odometry::ConstPtr &roberto_odom, const Odometry::ConstPtr &marvin_odom) {
  std::vector<geometry_msgs::Point> interaction_points = getInteractionPoints(roberto_odom->pose.pose.position, marvin_odom->pose.pose.position);
  int size = interaction_points.size();
  // ROS_INFO_STREAM("No. of poses:"<<size);
  return interaction_points[size/2];
}

string Visit_Door::getNearestParkingZone(const geometry_msgs::Point &ip) {
 
  string nearestZone; 
  float min_distance = INT_MAX, distance;
  geometry_msgs::Point safe_zone;
  bool empty = true;
  map<string, geometry_msgs::Pose>::iterator it;
  for(it = ParkingZones.begin(); it != ParkingZones.end(); it++ )
  {
    safe_zone.x = it->second.position.x;
    safe_zone.y = it->second.position.y;
    ROS_INFO_STREAM(it->first<<" "<<safe_zone.x<<" "<<safe_zone.y);
    myfile <<it->first<<" "<<safe_zone.x<<" "<<safe_zone.y<<"\n";
    distance = getInteractionPoints(safe_zone, ip).size(); // returns the number of poses btw interaction point and parking zone
    if (distance < min_distance && distance != 1){
      min_distance = distance;
      nearestZone = it->first;
      empty = false;
    }
  }
  if(empty){
    ROS_INFO_STREAM("All empty plans");
    myfile << "all empty plans\n";
    nearestZone = "empty";
  }
  ROS_INFO_STREAM("nearestZone: "<<nearestZone);
  myfile << "nearestZone: "<<nearestZone<<"\n";
  return nearestZone;
}

void Visit_Door::stopRobot(Client *client){
  //stop robot by canceling goal
  ros::Publisher pub1;
  actionlib_msgs::GoalID msg;
  msg.id = ""; 
  if(client == marvin_client){
    stop_marvin.publish(msg);
  }else if(client == roberto_client){
    stop_roberto.publish(msg);
  }
  client -> cancelGoal();
}

void Visit_Door::resumeRobot(Client *client, int door){
  //resume robot by giving goal of saved door number
  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;
  fluent.name = "not facing";
  if(door >= (int)doors.size()){
    door = 0;
  }
  if(door < 0){
    door = (int)doors.size()-1;
  }
  ROS_INFO_STREAM("resuming" << parkedLocation);
  if(client == parkedRobot && (parkedLocation.compare("p3_15") == 0 || parkedLocation.compare("p3_16") == 0)){
    string tempLocation = "r3_16";
    plan_execution::ExecutePlanGoal tempGoal;
    plan_execution::AspRule tempRule;
    plan_execution::AspFluent tempFluent;
    tempFluent.name = "not facing";
    tempFluent.variables.push_back(tempLocation);
    tempRule.body.push_back(tempFluent);
    tempGoal.aspGoal.push_back(tempRule);
    client->sendGoalAndWait(tempGoal);
    ROS_INFO_STREAM("Going to temp loc: " << tempLocation);
  }
  //ROS_INFO("SavedDoor: %d, door: %d",savedGoal,door);
  if(client == marvin_client){
    ROS_INFO_STREAM("Marvin going to " << doors.at(door));
    myfile << "Marvin going to " << doors.at(door) << "\n";
  }else{
    ROS_INFO_STREAM("Roberto going to" << doors.at(door));
    myfile << "Roberto going to " << doors.at(door) << "\n";
  }
  string location = doors.at(door);
  fluent.variables.push_back(location);
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);
  client->sendGoal(goal);
}


void Visit_Door::goToParkingZone(string ParkingZone, Client *client, bool &interrupt_flag) {
  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;
  fluent.name = "not facing";
  Client* otherClient;
  if(client == marvin_client){
    ROS_INFO_STREAM("Marvin "<<" going to " << ParkingZone);
    myfile << "Marvin "<<" going to " << ParkingZone << "\n";
    otherClient = roberto_client;
  }else if(client == roberto_client){
    ROS_INFO_STREAM("Roberto "<<" going to " <<  ParkingZone);
    myfile << "Roberto "<<" going to " << ParkingZone << "\n";
    otherClient = marvin_client;
  }
  stopRobot(otherClient);
  fluent.variables.push_back(ParkingZone);

  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);
  client->sendGoalAndWait(goal);
  if (client->getState() == actionlib::SimpleClientGoalState::ABORTED) {
    ROS_INFO("Aborted");
  }
  else if (client->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    ROS_INFO("Preempted");
  }
  
  else if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Succeeded!");
    stopRobot(client);
    parkedLocation = ParkingZone;
    ROS_INFO_STREAM("after success" << parkedLocation);
  }
  else
     ROS_INFO("Terminated");  
  /*if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO_STREAM("Moved to safe zone, resume goal");
    interrupt_flag = false;
  }*/
}
bool Visit_Door::checkForInterruption(const Odometry::ConstPtr &robot1_odom, const Odometry::ConstPtr &robot2_odom) {  
  // ROS_INFO_STREAM(robot1_odom->pose.pose.position.x<<" "<<robot2_odom->pose.pose.position.x);  
  // getIteractionPoint(robot1_odom, robot2_odom);
  if(abs(robot1_odom->pose.pose.position.x-robot2_odom->pose.pose.position.x)<5.0){
  // if(robot1_odom->pose.pose.position.x < robot2_odom->pose.pose.position.x && abs(robot1_odom->pose.pose.position.x-robot2_odom->pose.pose.position.x)<5.0){
    ROS_INFO_STREAM("x diff :"<<abs(robot1_odom->pose.pose.position.x - robot2_odom->pose.pose.position.x));
    return true;
  }
  return false;
}
bool Visit_Door::checkRobotsMovingCloser(vector<int> &distances){
  //increasing distance - robots moving away
  int size = distances.size();
  int i = size-35 > 0 ? size-35: 0; // check the latest 10 distances only
  int count = 0;
  float min;
  float max;
  for( ; i<size-1 ; i++) {
    ROS_INFO_STREAM(distances[i]<<" ");
    myfile<<distances[i]<<" ";
    if(distances[i]>distances[i+1] && distances[i]- distances[i+1] < 3 && distances[i] < 12) {
      //to avoid errors, we check if atleast two values are contradicting
      count++;
      if(distances[i]>max && count == 1){
        max = distances[i];
      }
      else if(count==3 && (max-distances[i+1])>2)
        return false;
    }
  }
  myfile<<"\n";
  return true;
}

bool Visit_Door::checkRobotsMovingAway(vector<int> &distances){
  //increasing distance - robots moving away
  //int size = distances.size();
  //int i = size-35 > 0 ? size-35: 0; // check the latest 10 distances only
  int count = 0;
  float min;
  float max;
  /*for( ; i<size-1 ; i++) {
    //ROS_INFO_STREAM(distances[i]<<" ");
    if(distances[i]<distances[i+1] && distances[i+1] - distances[i] < 3) {
      //to avoid errors, we check if atleast two values are contradicting
      count++;
      if(count == 1){
        min = distances[i];
      }    
      else if(count==3 && (distances[i+1]-min) > 2)
        return true;
    }
  }*/

  //odometry usage to check if robots are moving away, need to be within 8 to work
  int odom_size = robot_odom_distances.size();
  
  int i = odom_size - 15 > 0 ? odom_size-15: 0;
  count = 0;
  min = 1000;
  max = 0;
  for( ; i<odom_size-1 ; i++) {
    ROS_INFO_STREAM(robot_odom_distances[i]<<" ");
    myfile<<robot_odom_distances[i]<<" ";
    if(robot_odom_distances[i]<robot_odom_distances[i+1]) {
      //to avoid errors, we check if atleast two values are contradicting
      count++;
      if(count == 1){
        min = robot_odom_distances[i];
      }
      if(count>3 && (robot_odom_distances[i+1]-min) > 0.5){
        myfile<<robot_odom_distances[i+1]<<" ";
        return true;
      }
    }
  }
  myfile<<"\n";
  
  return false;
}

bool Visit_Door::safeInteraction(const Point &ip, const Point &roberto_odom, const Point &marvin_odom) {
  float tolerance = 1.5;
  Point safe_zone;
  //add distance to robot distances if greater than 1, else use previous value
  int distance = getInteractionPoints(roberto_odom, marvin_odom).size();

  float odom_distance = sqrt(pow(marvin_odom.x-roberto_odom.x,2)+pow(marvin_odom.y-roberto_odom.y,2));
  if(odom_distance>0.1 && odom_distance < 3){
    robot_odom_distances.push_back(odom_distance);
  }
  robot_distances.push_back(distance);
  /*if(distance>0){
    robot_distances.push_back(distance);
  }else if(robot_distances.size() > 0){
    robot_distances.push_back(robot_distances.at(robot_distances.size()-1));
  }*/
  //1 - Robots interact in safe zone

  map<string, geometry_msgs::Pose>::iterator it;
  for(it = ParkingZones.begin(); it != ParkingZones.end(); it++ )
  {
    safe_zone.x = it->second.position.x; //(ParkingZones[i].door_corners[0].x + ParkingZones[i].door_corners[1].x ) /2;
    safe_zone.y = it->second.position.y; //(ParkingZones[i].door_corners[0].y + ParkingZones[i].door_corners[1].y ) /2;    

    //check if the interaction point is in parking zone that is safe
    if(abs(safe_zone.x - ip.x)<tolerance && abs(safe_zone.y - ip.y)<tolerance
      && std::find(notSafe.begin(), notSafe.end(), it->first) == notSafe.end()){
      ROS_INFO_STREAM("interaction in safe zone"); 
      myfile << "interaction in safe zone\n"; 
      return true;
    }
  }

  //2 - Robots are moving away from each other
  if(checkRobotsMovingCloser(robot_distances)){
    //false - moving towards each other
    //true - moving away from each other
    ROS_INFO_STREAM("robots moving away from each other");
    return true;
  }

  //Robots are moving towards and they dont interact in safe zone
  ROS_INFO_STREAM("robots moving towards each other");
  return false;
}
void Visit_Door::callback(const Odometry::ConstPtr &roberto_odom, const Odometry::ConstPtr &marvin_odom)
{    
  //robot_distances.clear();
  plan_execution::ExecutePlanGoal marvin_goal;
  plan_execution::AspRule marvin_rule;
  plan_execution::AspFluent marvin_fluent;
  marvin_fluent.name = "not facing";
  plan_execution::ExecutePlanGoal roberto_goal;
  plan_execution::AspRule roberto_rule;
  plan_execution::AspFluent roberto_fluent;
  roberto_fluent.name = "not facing";
  if(marvin_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED && !marvin_parked){
    
    marvin_door += 1;
    if(marvin_door >= (int)doors.size()){
      marvin_door = 0;
    }
    string marvin_location = doors.at(marvin_door);
    ROS_INFO_STREAM("MARVIN going to " << marvin_location);
    myfile << "Marvin going to " << marvin_location << "\n";
    marvin_fluent.variables.push_back(marvin_location);

    marvin_rule.body.push_back(marvin_fluent);
    marvin_goal.aspGoal.push_back(marvin_rule);


    ROS_INFO("sending goal");
    marvin_client->sendGoal(marvin_goal);
  }
  if(roberto_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED && !roberto_parked){
    
    roberto_door -= 1;

    if(roberto_door < 0){
      roberto_door = (int)doors.size()-1;
    }
    string roberto_location = doors.at(roberto_door);

    ROS_INFO_STREAM("ROBERTO going to " << roberto_location);
    myfile << "Roberto going to " << roberto_location << "\n";
    roberto_fluent.variables.push_back(roberto_location);

    roberto_rule.body.push_back(roberto_fluent);
    roberto_goal.aspGoal.push_back(roberto_rule);
    
    ROS_INFO("sending goal");
    roberto_client->sendGoal(roberto_goal);
  }

  Point ip = getIteractionPoint(roberto_odom, marvin_odom);

  /*if(latestIP.size() == 10){
    latestIP.erase(latestIP.begin());
  }
  latestIP.push_back(ip);
  float sumX = 0; 
  float sumY = 0;
  std::vector<geometry_msgs::Point>::iterator it;
  for(it = latestIP.begin(); it < latestIP.end();it++){
    sumX += (*it).x;
    sumY += (*it).y;
  }
  Point avgIP;
  avgIP.x = sumX/latestIP.size();
  avgIP.y = sumY/latestIP.size();*/
  ROS_INFO_STREAM("interaction point::"<<ip.x<<" "<<ip.y);
  //if a robot is parked waiting for the other to pass
  bool safe = safeInteraction(ip, roberto_odom->pose.pose.position, marvin_odom->pose.pose.position);

  if(roberto_parked || marvin_parked){
    if(checkRobotsMovingAway(robot_distances)){
      ROS_INFO("resume parked robot");
      myfile << "resume parked robot";
      resumeRobot(parkedRobot,savedGoal);
      roberto_parked = false;
      marvin_parked = false;
    }
  }
  //check if interaction takes place in ParkingZone
  else if(!safe){
    //ROS_INFO_STREAM("Safe interaction");

    string nearZone = getNearestParkingZone(ip);
    //if can't find nearest zone to interaction point, get nearest zone to Marvin
    if(nearZone.compare("empty") == 0){
      nearZone = getNearestParkingZone(marvin_odom->pose.pose.position);
    }
    Point nearZone_pt;
    nearZone_pt.x = ParkingZones[nearZone].position.x;// (nearZone.door_corners[0].x + nearZone.door_corners[1].x)/2;
    nearZone_pt.y = ParkingZones[nearZone].position.y;//(nearZone.door_corners[0].y + nearZone.door_corners[1].y)/2;
    ROS_INFO_STREAM("nearX:" << nearZone_pt.x << "nearY;" << nearZone_pt.y);
    /*float roberto_x = roberto_odom->pose.pose.position.x;
    float roberto_y = roberto_odom->pose.pose.position.y;
    float marvin_x = marvin_odom->pose.pose.position.x;
    float marvin_y = marvin_odom->pose.pose.position.y;
    ROS_INFO_STREAM("robX:" << roberto_x << "robY;" << roberto_y);
    ROS_INFO_STREAM("marX:" << marvin_x << "marY;" << marvin_y);
    float marvin_zone_dist = sqrt(pow(marvin_x-nearZone_pt.x,2)+pow(marvin_y-nearZone_pt.y,2));
    float roberto_zone_dist = sqrt(pow(roberto_x-nearZone_pt.x,2)+pow(roberto_y-nearZone_pt.y,2));*/
    
    float roberto_zone_dist = getInteractionPoints(roberto_odom->pose.pose.position, nearZone_pt).size();
    float marvin_zone_dist = getInteractionPoints(marvin_odom->pose.pose.position, nearZone_pt).size();
    
    ROS_INFO_STREAM("roberto dis:"<<roberto_zone_dist<<" Marvin dis:"<<marvin_zone_dist);
    myfile << "roberto dis:"<<roberto_zone_dist<<" Marvin dis:"<<marvin_zone_dist << "\n";
    robot_distances.clear();
    robot_odom_distances.clear();
    //roberto closer to parking zone
    if(roberto_zone_dist < marvin_zone_dist) {      
      // clock_t initialTime = clock();
      // while(interrupt_flag)
      // {
      //   // if ((clock() - initialTime) / CLOCKS_PER_SEC >= 10) // time in seconds
      //   //   break;
      //   pub1.publish(msg);
      //   // ROS_INFO_STREAM("Stopping roberto");
      //   // roberto_client->cancelGoal();
      //   goToParkingZone(nearZone, roberto_client, interrupt_flag);
      // }      
      goToParkingZone(nearZone, roberto_client, interrupt_flag);
      resumeRobot(marvin_client,marvin_door);
      parkedRobot = roberto_client;
      savedGoal = roberto_door;
      roberto_parked = true;
    }

    //marvin closer to parking zone
    else {
      // clock_t initialTime = clock();
      // while(interrupt_flag)
      // {
      //   // if ((clock() - initialTime) / CLOCKS_PER_SEC >= 10) // time in seconds
      //   //   break;
      //   pub2.publish(msg);
      //   // ROS_INFO_STREAM("Stopping marvin");
      //   // marvin_client->cancelGoal();
      //   goToParkingZone(nearZone, marvin_client, interrupt_flag);
      // }
      goToParkingZone(nearZone, marvin_client, interrupt_flag);
      resumeRobot(roberto_client,roberto_door);
      parkedRobot = marvin_client;
      savedGoal = marvin_door;
      marvin_parked = true;
    }
    ROS_INFO("resume other");
    myfile << "resume other\n";
  }
}

void Visit_Door::run() 
{
  
  doors.push_back("d3_414a2");
  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");     

  marvin_door = 0;
  roberto_door = (int)doors.size()-1;

  marvin_client->waitForServer();
  roberto_client->waitForServer();

  //ros::Duration(2).sleep();
  
  //goToParkingZone("d3_414a1",roberto_client,interrupt_flag);


  string marvin_location = doors.at(marvin_door);
  string roberto_location = doors.at(roberto_door);

  plan_execution::ExecutePlanGoal marvin_goal;
  plan_execution::ExecutePlanGoal roberto_goal;

  plan_execution::AspRule marvin_rule;
  plan_execution::AspFluent marvin_fluent;
  marvin_fluent.name = "not facing";

  plan_execution::AspRule roberto_rule;
  plan_execution::AspFluent roberto_fluent;
  roberto_fluent.name = "not facing";

  ROS_INFO_STREAM("MARVIN going to " << marvin_location);
  ROS_INFO_STREAM("ROBERTO going to " << roberto_location);
  marvin_fluent.variables.push_back(marvin_location);
  roberto_fluent.variables.push_back(roberto_location);

  marvin_rule.body.push_back(marvin_fluent);
  marvin_goal.aspGoal.push_back(marvin_rule);

  roberto_rule.body.push_back(roberto_fluent);
  roberto_goal.aspGoal.push_back(roberto_rule);

  marvin_client->sendGoal(marvin_goal);
  roberto_client->sendGoal(roberto_goal);
  message_filters::Subscriber<nav_msgs::Odometry> marvin_odom(*nh_, "/marvin/odom", 1);
  message_filters::Subscriber<nav_msgs::Odometry> roberto_odom(*nh_, "/roberto/odom", 1);
  TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> sync(roberto_odom, marvin_odom, 10);
  sync.registerCallback(boost::bind(&Visit_Door::callback, this, _1, _2));
  
  ros::spin();
    
  return;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "multi_robot_navigation");
  
    // ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandle nh;
  
  Visit_Door visit_door(&nh);
  
  
  visit_door.run();
  // thread_odom.join();
  
  return 0;
}