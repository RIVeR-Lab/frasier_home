#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from frasier_planner_server.srv import PlanWaypoints, PlanWaypointsRequest
from std_srvs.srv import Empty, EmptyRequest

PLANNER_SERVER = '/frasier_planner_server/plan'
EXECUTION_SERVER = '/frasier_planner_server/execute'


def single_waypoint(planner_srv, execution_srv):
    waypoints = []

    w_1 = Pose()
    w_1.position.x = 0.7
    w_1.position.y = 0.3
    w_1.position.z = 0.7
    w_1.orientation.w = 0.0
    w_1.orientation.x = 0.707
    w_1.orientation.y = 0.0
    w_1.orientation.z = 0.707

    waypoints.append(w_1)
    req = PlanWaypointsRequest()
    req.waypoints = waypoints

    res = planner_srv(req)
    rospy.loginfo(res.trajectory)

    exec_req = EmptyRequest()
    res = execute_srv(exec_req)


def multi_waypoints(planner_srv):
    waypoints = [Pose()] * 3

    waypoints[0].orientation.w = 0.0
    waypoints[0].orientation.x = 0.707
    waypoints[0].orientation.y = 0.0
    waypoints[0].orientation.z = 0.707

    waypoints[1].orientation = waypoints[0].orientation
    waypoints[2].orientation = waypoints[0].orientation

    waypoints[0].position.x = 1.0
    waypoints[0].position.y = 0.0
    waypoints[0].position.z = 0.5

    waypoints[1].position.x = 1.5
    waypoints[1].position.y = 0.0
    waypoints[1].position.z = 0.8

    waypoints[2].position.x = 2.0
    waypoints[2].position.y = 0.4
    waypoints[2].position.z = 1.0

    req = PlanWaypointsRequest()
    req.waypoints = waypoints

    res = planner_srv(req)
    rospy.loginfo(res.trajectory)




if __name__ == "__main__":
    rospy.init_node('test_plan_req')
    rospy.wait_for_service(PLANNER_SERVER)
    planner_srv = rospy.ServiceProxy(PLANNER_SERVER, PlanWaypoints)
    execute_srv = rospy.ServiceProxy(EXECUTION_SERVER, Empty)

    single_waypoint(planner_srv, execute_srv)
    # multi_waypoints(planner_srv)




