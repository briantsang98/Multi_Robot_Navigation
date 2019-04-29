
#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
using namespace nav_msgs;
using namespace message_filters;

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

using namespace std;

// ros::NodeHandle n;
// ros::NodeHandle privateNode("~");

// void reportMarvin(Client& marvin_client) {
//   if (marvin_client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
//     ROS_INFO("marvin: Aborted");
//   }
//   else if (marvin_client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
//     ROS_INFO("marvin: Preempted");
//   }
  
//   else if (marvin_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//     ROS_INFO("marvin: Succeeded!");
//   }
//   else
//      ROS_INFO("marvin: Terminated");
// }

// void reportRoberto(Client& roberto_client) {
//   if (roberto_client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
//     ROS_INFO("roberto: Aborted");
//   }
//   else if (roberto_client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
//     ROS_INFO("roberto: Preempted");
//   }
  
//   else if (roberto_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//     ROS_INFO("roberto: Succeeded!");
//   }
//   else
//      ROS_INFO("roberto: Terminated");
// }

class Visit_Door {
public:
  Visit_Door(ros::NodeHandle* nh);
  void run();
  void callback(const nav_msgs::Odometry::ConstPtr &roberto_odom, const nav_msgs::Odometry::ConstPtr &marvin_odom);
  bool checkForInterruption(geometry_msgs::Point robot1, geometry_msgs::Point robot2);

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
bool Visit_Door::checkForInterruption(geometry_msgs::Point robot1, geometry_msgs::Point robot2) {  
  if(abs(robot1.x - robot2.x)>2.0){
    ROS_INFO_STREAM("x diff :"<<abs(robot1.x - robot2.x));
    return false;
  }
  return true;
}
void Visit_Door::callback(const nav_msgs::Odometry::ConstPtr &roberto_odom, const nav_msgs::Odometry::ConstPtr &marvin_odom)
{  
  // ROS_INFO_STREAM("Marvin " << marvin_odom->pose.pose.position.x);
  ROS_INFO_STREAM("interaction_point" << min(roberto_odom->pose.pose.position.x,marvin_odom->pose.pose.position.x) + (abs(roberto_odom->pose.pose.position.x - marvin_odom->pose.pose.position.x) / 2));
  // if((int)marvin_odom->pose.pose.position.x > 19 && (int)marvin_odom->pose.pose.position.x < 22) {    
  if(abs(marvin_odom->pose.pose.position.x -marvin_odom->pose.pose.position.x) < 2.0) {    
    ROS_INFO_STREAM("Stopping roberto and marvin");    
    ros::Publisher pub1 = nh_->advertise<actionlib_msgs::GoalID> 
        ("/roberto/move_base/cancel", 1000); 
        ros::Publisher pub2 = nh_->advertise<actionlib_msgs::GoalID> 
        ("/marvin/move_base/cancel", 1000); 
    actionlib_msgs::GoalID msg;
    msg.id = "";
    bool interrupt_flag = true;
    while(interrupt_flag){
      pub1.publish(msg);   
      pub2.publish(msg);   
      // if(checkForInterruption(marvin_odom->pose.pose.position, roberto_odom->pose.pose.position)){
      //   interrupt_flag = false;
      // }
    }    
  
  }

}

void Visit_Door::run() 
{
  std::vector<string> doors;

  doors.push_back("d3_414b1");    
  doors.push_back("d3_414a2");
  doors.push_back("d3_414b2");
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

  while(ros::ok()) {
    message_filters::Subscriber<nav_msgs::Odometry> marvin_odom(*nh_, "/marvin/odom", 1);
    message_filters::Subscriber<nav_msgs::Odometry> roberto_odom(*nh_, "/roberto/odom", 1);
    TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> sync(marvin_odom, roberto_odom, 10);
    sync.registerCallback(boost::bind(&Visit_Door::callback, this, _1, _2));
    ROS_INFO_STREAM("in main");
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
}
int main(int argc, char**argv) {
  ros::init(argc, argv, "multi_robot_navigation");

  ros::NodeHandle nh;
  ros::NodeHandle privateNode("~");
  Visit_Door visit_door(&nh);
  visit_door.run();
  
  return 0;
}