#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <boost/thread/thread.hpp>

void do_stuff(int* publish_rate)
{
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::Publisher pub_b = node->advertise<std_msgs::Empty>("topic_b", 10);

  ros::Rate loop_rate(*publish_rate);  
  while (ros::ok())
  {
    int cnt = 0;
    while(cnt < 5){
      ROS_INFO_STREAM("in function");
      std_msgs::Empty msg;
      pub_b.publish(msg);
      loop_rate.sleep();  
      cnt++;
    }    
  }
}

int main(int argc, char** argv)
{
  int rate_b = 1; // 1 Hz

  ros::init(argc, argv, "mt_node");

  // spawn another thread
  boost::thread thread_b(do_stuff, &rate_b);

  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  ros::Publisher pub_a = node->advertise<std_msgs::Empty>("topic_a", 10);

  ros::Rate loop_rate(10); // 10 Hz  
  while (ros::ok())
  {
    int count = 0;
    while(count < 5){
      ROS_INFO_STREAM("in main");
      std_msgs::Empty msg;
      pub_a.publish(msg);
      loop_rate.sleep();
      count++;
    }
    
    // process any incoming messages in this thread
    ros::spinOnce();
  }

  // wait the second thread to finish
  thread_b.join();

  return 0;
}