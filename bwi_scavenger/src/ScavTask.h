#ifndef TASKRESULT_H
#define TASKRESULT_H

#include <ros/ros.h>
#include <string>

enum TaskResult{ TIMEOUT, SUCCEEDED, FAILED}; 

class ScavTask {

public:

    ros::NodeHandle *nh; 

    std::string task_description; 
    std::string task_name; 
    const static float tolerance = 0.5; 
    std::vector<std::string> task_parameters; 

    virtual void executeTask(int timeout, TaskResult &result, std::string &record) {}

    virtual void stopEarly() {}


}; 

#endif
