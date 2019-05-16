
#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <bwi_planning_common/structures.h>
#include <bwi_planning_common/utils.h>
#include <bwi_planning_common/interactions.h>

#include <ros/ros.h>
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
  bool checkRobotsDirection(vector<int> &distances);

protected: 
    ros::NodeHandle *nh_;
    Client *roberto_client; 
    Client *marvin_client; 
    bool interrupt_flag;
    std::map<std::string, geometry_msgs::Pose> ParkingZones;
    ros::ServiceClient make_plan_client_roberto;
    ros::ServiceClient make_plan_client_marvin;
    vector<int>robot_distances;
};

Visit_Door::Visit_Door( ros::NodeHandle* nh )
{
  nh_ = nh;
  marvin_client = new Client("/marvin/action_executor/execute_plan", true);
  roberto_client = new Client("/roberto/action_executor/execute_plan", true);
  interrupt_flag = true;
  std::string data_directory = "/home/users/NI2452/catkin_ws/src/bwi_common/utexas_gdc/maps/simulation/3ne";
  std::string loc_file = bwi_planning_common::getObjectsFileLocationFromDataDirectory(data_directory);
  ROS_INFO_STREAM("Reading ParkingZones file: " + loc_file);
  bwi_planning_common::readObjectApproachFile(loc_file, ParkingZones);
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
  goal.pose.orientation.w= 1.0;
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
  std::vector<float> distances;
  float min_distance = INT_MAX, distance;
  geometry_msgs::Point safe_zone;

  map<string, geometry_msgs::Pose>::iterator it;
  for(it = ParkingZones.begin(); it != ParkingZones.end(); it++ )
  {
    safe_zone.x = it->second.position.x;
    safe_zone.y = it->second.position.y;
    ROS_INFO_STREAM(it->first<<" "<<safe_zone.x<<" "<<safe_zone.y);
    distance = getInteractionPoints(safe_zone, ip).size(); // returns the number of poses btw interaction point and parking zone
    if (distance < min_distance){
      min_distance = distance;
      nearestZone = it->first;
    }
  }
  ROS_INFO_STREAM("nearestZone: "<<nearestZone);
  return nearestZone;
}

void Visit_Door::goToParkingZone(string ParkingZone, Client *client, bool &interrupt_flag) {

  string location = ParkingZone;
  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;
  fluent.name = "not facing";

  ROS_INFO_STREAM(client<<" going to " << location);
  fluent.variables.push_back(location);

  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);
  client->sendGoalAndWait(goal);  
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO_STREAM("Moved to safe zone, resume goal");
    interrupt_flag = false;
  }
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
bool Visit_Door::checkRobotsDirection(vector<int> &distances){
  //increasing distance - robots moving away
  int size = distances.size();
  int i = size-10 > 0 ? size-10 : 0; // check the latest 10 distances only
  int count = 0;
  for( ; i<size-1 ; i++) {
    // ROS_INFO_STREAM(distances[i]<<" ");
    if(distances[i]<distances[i+1]) {
      //to avoid errors, we check if atleast two values are contradicting
      count++;
      if(count==3)
        break;
    }
  }
  if(i!=distances.size()-1){
    return false;
  }
  return true;
}
bool Visit_Door::safeInteraction(const Point &ip, const Point &roberto_odom, const Point &marvin_odom) {
  float tolerance = 2.0;
  Point safe_zone;
  robot_distances.push_back(getInteractionPoints(roberto_odom, marvin_odom).size());
  //1 - Robots interact in safe zone

  map<string, geometry_msgs::Pose>::iterator it;
  for(it = ParkingZones.begin(); it != ParkingZones.end(); it++ )
  {
    safe_zone.x = it->second.position.x; //(ParkingZones[i].door_corners[0].x + ParkingZones[i].door_corners[1].x ) /2;
    safe_zone.y = it->second.position.y; //(ParkingZones[i].door_corners[0].y + ParkingZones[i].door_corners[1].y ) /2;    

    //check if the interaction point is in the safe zone
    if(abs(safe_zone.x - ip.x)<tolerance && abs(safe_zone.y - ip.y)<tolerance){
      ROS_INFO_STREAM("interaction in safe zone");  
      return true;
    }
  }

  //2 - Robots are moving away from each other
  if(checkRobotsDirection(robot_distances)){
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
  Point ip = getIteractionPoint(roberto_odom, marvin_odom);
  ROS_INFO_STREAM("interaction point::"<<ip.x<<" "<<ip.y);
  //check if interaction takes place in ParkingZone
  if(safeInteraction(ip, roberto_odom->pose.pose.position, marvin_odom->pose.pose.position)){
    ROS_INFO_STREAM("Safe interaction");
  }
  else{
    string nearZone = getNearestParkingZone(ip);
    Point nearZone_pt;
    nearZone_pt.x = ParkingZones[nearZone].position.x;// (nearZone.door_corners[0].x + nearZone.door_corners[1].x)/2;
    nearZone_pt.y = ParkingZones[nearZone].position.y;//(nearZone.door_corners[0].y + nearZone.door_corners[1].y)/2;
    float roberto_zone_dist = getInteractionPoints(roberto_odom->pose.pose.position, nearZone_pt).size();
    float marvin_zone_dist = getInteractionPoints(marvin_odom->pose.pose.position, nearZone_pt).size();

    ros::Publisher pub1 = nh_->advertise<actionlib_msgs::GoalID> 
        ("/roberto/move_base/cancel", 1000, true); 
    ros::Publisher pub2 = nh_->advertise<actionlib_msgs::GoalID> 
        ("/marvin/move_base/cancel", 1000, true); 
    actionlib_msgs::GoalID msg;
    msg.id = "";
    
    ROS_INFO_STREAM("roberto dis:"<<roberto_zone_dist<<" Marvin dis:"<<marvin_zone_dist);
    
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
    }
  }
}

void Visit_Door::run() 
{
  std::vector<string> doors;

  // doors.push_back("o3_402"); 
  doors.push_back("d3_414a2");  
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  doors.push_back("d3_414b1");      
  // doors.push_back("p3_17");      
  
  int marvin_door = 0;
  int roberto_door = (int)doors.size()-1;

  marvin_client->waitForServer();
  roberto_client->waitForServer();

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
  TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> sync(marvin_odom, roberto_odom, 10);
  sync.registerCallback(boost::bind(&Visit_Door::callback, this, _1, _2));
  while(ros::ok()) {
    // message_filters::Subscriber<nav_msgs::Odometry> marvin_odom(*nh_, "/marvin/odom", 1);
    // message_filters::Subscriber<nav_msgs::Odometry> roberto_odom(*nh_, "/roberto/odom", 1);
    // TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> sync(marvin_odom, roberto_odom, 10);
    // sync.registerCallback(boost::bind(&Visit_Door::callback, this, _1, _2));
    robot_distances.clear();
    if(marvin_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      marvin_location = doors.at(marvin_door);
      marvin_door += 1;
      if(marvin_door >= (int)doors.size()){
        marvin_door = 0;
      }

      ROS_INFO_STREAM("MARVIN going to " << marvin_location);
      plan_execution::ExecutePlanGoal marvin_goal;

      plan_execution::AspRule marvin_rule;
      plan_execution::AspFluent marvin_fluent;
      marvin_fluent.name = "not facing";

      marvin_fluent.variables.push_back(marvin_location);

      marvin_rule.body.push_back(marvin_fluent);
      marvin_goal.aspGoal.push_back(marvin_rule);


      ROS_INFO("sending goal");
      marvin_client->sendGoal(marvin_goal);
    }
    if(roberto_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      roberto_location = doors.at(roberto_door);
      roberto_door -= 1;

      if(roberto_door <= 0){
        roberto_door = (int)doors.size()-1;
      }

      ROS_INFO_STREAM("ROBERTO going to " << roberto_location);
      plan_execution::ExecutePlanGoal roberto_goal;

      plan_execution::AspRule roberto_rule;
      plan_execution::AspFluent roberto_fluent;
      roberto_fluent.name = "not facing";

      roberto_fluent.variables.push_back(roberto_location);

      roberto_rule.body.push_back(roberto_fluent);
      roberto_goal.aspGoal.push_back(roberto_rule);
      
      ROS_INFO("sending goal");
      roberto_client->sendGoal(roberto_goal);
    }
    ros::spin();

  }
  return;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "multi_robot_navigation");

    // ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
  ros::NodeHandle nh;
  ros::NodeHandle privateNode("~");
  Visit_Door visit_door(&nh);
  
  visit_door.run();
  // thread_odom.join();
  
  return 0;
}