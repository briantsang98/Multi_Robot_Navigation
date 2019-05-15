#include <boost/filesystem.hpp>
#include <fstream>
#include <libgen.h>
#include <stdexcept>
#include <stdio.h>
#include <yaml-cpp/yaml.h>

#include <bwi_planning_common/interactions.h>
#include <tf/transform_datatypes.h> 

namespace bwi_planning_common {
  std::vector<geometry_msgs::Point> getInteractionPoints(const geometry_msgs::Point &start_pt, const geometry_msgs::Point &goal_pt){
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
            interaction_points.push_back(p.pose.position);
            // ROS_INFO("x = %f, y = %f", p.pose.position.x, p.pose.position.y);
          }        
        }  
      } else {        
        ROS_INFO_STREAM("Empty plan");
      }
    } else {
      ROS_INFO_STREAM("service failed");
    }
    return interaction_points;
  }
  geometry_msgs::Point getIteractionPoint(const Odometry::ConstPtr &roberto_odom, const Odometry::ConstPtr &marvin_odom) {
    std::vector<geometry_msgs::Point> interaction_points = getInteractionPoints(roberto_odom->pose.pose.position, marvin_odom->pose.pose.position);
    int size = interaction_points.size();
    ROS_INFO_STREAM("Size:"<<size);
    return interaction_points[size/2];
  }

  Door getNearestSafeZone(const geometry_msgs::Point &ip) {
   
    Door nearestZone; 
    std::vector<float> distances;
    float min_distance = INT_MAX, distance;
    geometry_msgs::Point safe_zone;
    for(int i = 0 ; i < safeZones.size() ; i++) {
      safe_zone.x = (safeZones[i].door_corners[0].x + safeZones[i].door_corners[1].x) /2 ;
      safe_zone.y = (safeZones[i].door_corners[0].y + safeZones[i].door_corners[1].y) /2 ;

      distance = getInteractionPoints(safe_zone, ip).size(); // returns the number of poses btw interaction point and parking zone
      if (distance < min_distance){
        min_distance = distance;
        nearestZone = safeZones[i];
      }
    }
    ROS_INFO_STREAM("nearestZone: "<<nearestZone.name);
    return nearestZone;
  }  

} /* bwi_common */
