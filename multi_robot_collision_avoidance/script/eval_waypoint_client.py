#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import Pose
from multi_robot_collision_avoidance.srv import *

def add_two_ints_client(cpos, bpos, wpos):
    rospy.wait_for_service('eval_waypoint')
    try:
        add_two_ints = rospy.ServiceProxy('eval_waypoint', EvalWaypoint)
        resp1 = add_two_ints(cpos, bpos)
        print(resp1)
        return resp1.E_reward
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "cx cy bx by wx wy"

if __name__ == "__main__":
    if len(sys.argv) == 7:
        cpos = Pose()
        cpos.position.x = float(sys.argv[1])
        cpos.position.y = float(sys.argv[2])

        bpos = Pose()
        bpos.position.x = float(sys.argv[3])
        bpos.position.y = float(sys.argv[4])

        wpos = Pose()
        wpos.position.x = float(sys.argv[5])
        wpos.position.y = float(sys.argv[6])
    else:
        print usage()
        sys.exit(1)
    print("Requesting Evaluation of point ({:.3f},{:.3f})".format(wpos.position.x, wpos.position.y))
    print "%s + %s = %s"%(wpos.position.x, wpos.position.y, add_two_ints_client(cpos, bpos, wpos))
