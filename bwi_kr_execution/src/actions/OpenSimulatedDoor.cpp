#include "OpenSimulatedDoor.h"

#include "plan_execution/CurrentStateQuery.h"
#include "bwi_msgs/DoorHandlerInterface.h"

#include "ActionFactory.h"
#include "plan_execution/LogicalAction.h"

#include "actasp/AspFluent.h"

#include <ros/ros.h>

using namespace std;
using namespace ros;

namespace bwi_krexec {


OpenSimulatedDoor::OpenSimulatedDoor() : door(), done(false),requestSent(false) {}

void OpenSimulatedDoor::run() {
  NodeHandle n;

  if (!requestSent) {
    ServiceClient doorClient = n.serviceClient<bwi_msgs::DoorHandlerInterface> ("/update_doors");
    doorClient.waitForExistence();

    bwi_msgs::DoorHandlerInterface dhi;

    dhi.request.all_doors = false;
    dhi.request.door = door;
    dhi.request.open = true;

    doorClient.call(dhi);

    requestSent = true;

    vector<string> params;
    params.push_back(door);
    senseDoor = new plan_exec::LogicalAction ("sensedoor",params);
  }

  senseDoor->run();

  ros::ServiceClient currentClient = n.serviceClient<plan_execution::CurrentStateQuery> ("current_state_query");
  plan_execution::AspFluent openFluent;
  openFluent.name = "open";
  openFluent.timeStep = 0;
  openFluent.variables.push_back(door);

  plan_execution::AspRule rule;
  rule.head.push_back(openFluent);

  plan_execution::CurrentStateQuery csq;
  csq.request.query.push_back(rule);

  currentClient.call(csq);

  done = csq.response.answer.satisfied;

}


actasp::Action* OpenSimulatedDoor::cloneAndInit(const actasp::AspFluent& fluent) const {
  OpenSimulatedDoor *newAction = new OpenSimulatedDoor();
  newAction->door = fluent.getParameters().at(0);

  return newAction;
}

std::vector<std::string> OpenSimulatedDoor::getParameters() const {
  vector<string> param;
  param.push_back(door);
  return param;
}

ActionFactory simulatedOpenDoor(new OpenSimulatedDoor(), true);

}
