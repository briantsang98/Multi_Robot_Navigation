// #include <cstdlib>

// #include <ros/ros.h>
// #include <nav_msgs/Odometry.h>
// struct Point2f{
// 	float x;
// 	float y;
// };
// class InteractionPoint {
// 	private:
// 		// Point2f robot_loc(robot_x_, robot_y_);
// 		Point2f position1;
// 		Point2f position2;
// 		char intersecting_axis;

// 	public:
// 		void InteractionPoint(const nav_msgs::Odometry::ConstPtr &roberto_odom, const nav_msgs::Odometry::ConstPtr &marvin_odom);
// 		void findIntersectingAxis();
// 		bool validCorner(Point2f c1);
// 		void findInteractionPoint();
// };
// void InteractionPoint::InteractionPoint(Point2f pos1, Point2f pos2) {
// 	roberto_odom->pose.pose.position.y
// 	position1 = pos1;
// 	position2 = pos2;
// 	intersecting_axis = NULL;
// }

// void InteractionPoint::findIntersectingAxis() {
// 	x_axis_diff = abs(position1[0] - position2[0]);
// 	y_axis_diff = abs(position1[1] - position2[1]);
// 	if(x_axis_diff < y_axis_diff) {		
// 		intersecting_axis='x';
// 	}
// 	else{
// 		intersecting_axis='y';
// 	}			
// }

// bool InteractionPoint::validCorner(Point2f corner) {
// 	return true;
// }

// void InteractionPoint::findInteractionPoint() {
// 	Point2f corner1 = new Point2f(position1[0],position2[1]);
// 	Point2f corner2 = new Point2f(position2[0],position1[1]);
// 	Point2f ip; 
// 	length = abs(position1[1] - position2[1]);
// 	breadth = abs(position1[0] - position2[0]);
// 	if(validCorner(corner1)){
// 		if(intersecting_axis=='x')
// 			ip = new Point2f((corner1[0]+position2[0] - breadth)/2 , corner1[1]);
// 		else if(intersecting_axis=='y')
// 			ip = new Point2f(corner1[0] , (corner1[1]+position2[1] - breadth)/2);
// 	}
// 	ROS_INFO_STREAM("Point"<<ip[0]<<" "<<ip[1]);
		
// }
// int main(int argc, char**argv) {
// 	Point2f p1 = new Point2f(0.0,10.0);
// 	Point2f p2 = new Point2f(2.0,0.0);
// 	InteractionPoint ip_obj = new InteractionPoint(p1,p2);
// 	intPt = InteractionPoint(p1, p2)
  
//   return 0;
// }
// // class InteractionPoint():
// // 	def __init__(self, position1, position2):
// // 		self.position1 = position1
// // 		self.position2 = position2
// // 		self.intersecting_axis = {'x':0, 'y':1}
// // 	def findIntersectingAxis():
// // 		x_axis_diff = abs(position1[0] - position2[0])
// // 		y_axis_diff = abs(position1[1] - position2[1])
// // 		if x_axis_diff < y_axis_diff:
// // 			self.intersecting_axis='x'
// // 		else:
// // 			self.intersecting_axis='y'
// // 	def validCorner(corner):
// // 		return True
// // 	def findInteractionPoint():
// // 		xmax = 50
// // 		xmin = -20
// // 		ymax = 25
// // 		ymin = 0
// // 		# while position1[0]!=position2[0] and position1[1] != position2[1]:
// // 		corner1 = [position1[0],position2[1]]
// // 		corner2 = [position2[0],position1[1]]
// // 		length = abs(position1[1] - position2[1])
// // 		breadth = abs(position1[0] - position2[0])
// // 		if liesInCorridor(corner1):
// // 			if self.intersecting_axis=='x':		
// // 				ip = [(corner1[0]+position2[0] - breadth)/2 , corner1[1]]
// // 			elif self.intersecting_axis=='y':
// // 				ip = [corner1[0] , (corner1[1]+position2[1] - breadth)/2]
// // 		print(ip[0], ip[1])
// // 		# axis = findIntersectingAxis()
// // 		# if self.intersecting_axis=='x':
// // 		# 	if position1[1] > position2[1]:
// // 		# 		new_position1 = [position1[0], position1[1] - diff]
// // 		# 		new_position2 = [position2[0] - diff, position2[1]]
// // 		# 	elif position1[1] <= position2[1]:
// // 		# 		new_position1 = [position1[0] - diff, position1[1]]
// // 		# 		new_position2 = [position2[0], position2[1] - diff]
// // 		# elif self.intersecting_axis=='y':			
// // 		# 	if position1[0] > position2[0]:
// // 		# 		new_position1 = [position1[0] - diff, position1[1]]
// // 		# 		new_position2 = [position2[0], position2[1] - diff]
// // 		# 	elif position1[0] <= position2[0]:
// // 		# 		new_position1 = [position1[0], position1[1] - diff]
// // 		# 		new_position2 = [position2[0] - diff, position2[1]]
// // def main():
// // 	p1 = ['x': 0, 'y':10]
// // 	p2 = ['x': 2, 'y':0]
// // 	intPt = InteractionPoint(p1, p2)	
// // if __name__ == '__main__':
// // 	main()