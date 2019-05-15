#ifndef BWI_PLANNING_COMMON_INTERACTIONS_H_
#define BWI_PLANNING_COMMON_INTERACTIONS_H_

#include <bwi_tools/point.h>
#include <stdint.h>
#include <string>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace geometry_msgs;
using namespace nav_msgs;

namespace bwi_planning_common {

  Door getNearestSafeZone(const geometry_msgs::Point &ip);
  vector<geometry_msgs::Point> getInteractionPoints(const Point &start_pt, const Point &goal_pt);
  geometry_msgs::Point getIteractionPoint(const Odometry::ConstPtr &roberto_odom, const Odometry::ConstPtr &marvin_odom);  

  void readLocationFile(const std::string& filename, 
      std::vector<std::string>& locations, std::vector<int32_t>& location_map);

} /* bwi_planning_common */

#endif /* end of include guard: BWI_PLANNING_COMMON_INTERACTIONS_H_ */
