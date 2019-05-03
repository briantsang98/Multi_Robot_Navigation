
#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <string>
using namespace nav_msgs;
using namespace message_filters;

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

using namespace std;

class Visit_Door {
public:
  Visit_Door(ros::NodeHandle* nh);
  void run();
  void callback(const nav_msgs::Odometry::ConstPtr &roberto_odom, const nav_msgs::Odometry::ConstPtr &marvin_odom);
  bool checkForInterruption(geometry_msgs::Point robot1, geometry_msgs::Point robot2);
  void task_interrupt(int* publish_rate);
  bool goToSafeZone(const std::string);

protected: 
    ros::NodeHandle *nh_;
    Client *roberto_client; 
    Client *marvin_client; 

};

Visit_Door::Visit_Door( ros::NodeHandle* nh )
{
  nh_ = nh;

  marvin_client = new Client("/marvin/action_executor/execute_plan", true);
  roberto_client = new Client("/roberto/action_executor/execute_plan", true);
  // marvin_client->waitForServer(); 
    
}
bool Visit_Door::goToSafeZone(const std::string robot_name) {
  string location = "ia_1";
  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;
  fluent.name = "not facing";

  ROS_INFO_STREAM(robot_name<<" going to " << location);
  fluent.variables.push_back(location);

  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);

  if(robot_name.compare("roberto")==0){
    roberto_client->sendGoal(goal);  
    if(roberto_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      return true;
  }
  else {
    marvin_client->sendGoal(goal);  
    if(marvin_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      return true;
  }
  ros::spin();
  return false;
}
bool Visit_Door::checkForInterruption(geometry_msgs::Point robot1, geometry_msgs::Point robot2) {  
  ROS_INFO_STREAM(robot1.x<<" "<<robot2.x);
  if(robot1.x < robot2.x && abs(robot1.x-robot2.x)<3.0){
    ROS_INFO_STREAM("x diff :"<<abs(robot1.x - robot2.x));
    return true;
  }
  return false;
}
void Visit_Door::callback(const nav_msgs::Odometry::ConstPtr &roberto_odom, const nav_msgs::Odometry::ConstPtr &marvin_odom)
{  
  // ROS_INFO_STREAM("Marvin " << marvin_odom->pose.pose.position.x);
  // ROS_INFO_STREAM("interaction_point" << min(roberto_odom->pose.pose.position.x,marvin_odom->pose.pose.position.x) + (abs(roberto_odom->pose.pose.position.x - marvin_odom->pose.pose.position.x) / 2));
  // if((int)marvin_odom->pose.pose.position.x > 19 && (int)marvin_odom->pose.pose.position.x < 22) {    
  ROS_INFO_STREAM("Diff"<<abs(marvin_odom->pose.pose.position.x - roberto_odom->pose.pose.position.x));
  if(abs(marvin_odom->pose.pose.position.x - roberto_odom->pose.pose.position.x) < 5.0) {        
    ros::Publisher pub1 = nh_->advertise<actionlib_msgs::GoalID> 
        ("/roberto/move_base/cancel", 1000, true); 
        ros::Publisher pub2 = nh_->advertise<actionlib_msgs::GoalID> 
        ("/marvin/move_base/cancel", 1000, true); 
    actionlib_msgs::GoalID msg;
    msg.id = "";
    static bool interrupt_flag = true;
    if(marvin_odom->pose.pose.position.x > roberto_odom->pose.pose.position.x){
      if(checkForInterruption(marvin_odom->pose.pose.position, roberto_odom->pose.pose.position)){
        interrupt_flag = false;
      }  
    } 
    else{
      if(checkForInterruption(roberto_odom->pose.pose.position, marvin_odom->pose.pose.position)){
        interrupt_flag = false;
      }  
    }
    ROS_INFO_STREAM("Flag"<<interrupt_flag);

    while(interrupt_flag){    
      // goToSafeZone("roberto");  
      pub1.publish(msg);   
      // pub2.publish(msg);   
      // ROS_INFO_STREAM(marvin_odom->pose.pose.position.x<<" "<<roberto_odom->pose.pose.position.x);            
    }    
  
  }
}

void Visit_Door::task_interrupt(int* publish_rate) {
  
  // ros::Rate loop_rate(*publish_rate);
  message_filters::Subscriber<nav_msgs::Odometry> marvin_odom(*nh_, "/marvin/odom", 1);
  message_filters::Subscriber<nav_msgs::Odometry> roberto_odom(*nh_, "/roberto/odom", 1);
  TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> sync(marvin_odom, roberto_odom, 10);
  ROS_INFO_STREAM("task_interrupt function"); 
  sync.registerCallback(boost::bind(&Visit_Door::callback, this, _1, _2));  
  ROS_INFO_STREAM("task_interrupt function111111"); 
  // sync.registerCallback(&Visit_Door::callback, this, _1, _2);  
}

void Visit_Door::run() 
{
  std::vector<string> doors;

  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  doors.push_back("d3_414a2");
     
  
  int marvin_door = 0;
  int roberto_door = (int)doors.size()-1;

  // Client marvin_client("marvin/action_executor/execute_plan", true);
  marvin_client->waitForServer();
  
  // Client roberto_client("roberto/action_executor/execute_plan", true);
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