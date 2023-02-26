#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from frasier_planner_server.srv import PickAndPlace
from frasier_planner_server.srv import PickAndPlaceRequest
from frasier_planner_server.msg import DetectedObjects
from point_cloud_proc.srv import TabletopClustering
from point_cloud_proc.srv import TabletopClusteringRequest
from point_cloud_proc.srv import TabletopClusteringResponse
import copy

PICK_PLACE_SERVER = '/frasier_planner_server/pick_and_place'
DETECTION_TOPIC = '/frasier_planner_server/detected_objects'
CLUSTER_SERVER = '/frasier_planner_server/cluster_objects'


class PickAndPlaceTest():
    def __init__(self):
        rospy.init_node('test_plan_req')
        rospy.wait_for_service(PICK_PLACE_SERVER)
        self.planner_srv = rospy.ServiceProxy(PICK_PLACE_SERVER, PickAndPlace)
        self.cluster_srv = rospy.ServiceProxy(CLUSTER_SERVER, TabletopClustering)
        rospy.Subscriber(DETECTION_TOPIC, DetectedObjects, self.object_detection_cb)
        self.detected_objects = False
                
    def object_detection_cb(self, data):
        self.objects = data
        self.detected_objects = True
        print(data)

    def pick_and_place(self):
        while not self.detected_objects and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        pick_place_req = PickAndPlaceRequest()
        pick_place_req.pick_pose = copy.deepcopy(self.objects.object_poses[0])
        pick_place_req.place_pose = copy.deepcopy(self.objects.object_poses[0])
        pick_place_req.place_pose.position.y += 0.4
        pick_place_req.place_pose.position.z += 0.3
        res = self.planner_srv.call(pick_place_req)

    def demo(self):
        cluster_req = TabletopClusteringRequest()
        cluster_res = self.cluster_srv.call(cluster_req)
        if not cluster_res.success:
            print('No object found on table')
            exit(1)

        object = cluster_res.objects[0]
        pick_place_req = PickAndPlaceRequest()
        place_pose = Pose()
        place_pose.position.x = 0
        place_pose.position.y = -1.0
        place_pose.position.z = 0.4

        place_pose.orientation.x = 0.5
        place_pose.orientation.y = -0.5
        place_pose.orientation.z = 0.5
        place_pose.orientation.w = 0.5

        pick_place_req.pick_pose = copy.deepcopy(object.pose)
        pick_place_req.place_pose = place_pose
        res = self.planner_srv.call(pick_place_req)


if __name__ == "__main__":
    test = PickAndPlaceTest()
    test.demo()
