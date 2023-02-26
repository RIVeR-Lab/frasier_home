#include <ros/package.h>
#include <point_cloud_proc/point_cloud_proc.h>
#include <frasier_openrave/frasier_openrave.h>
#include <frasier_openrave/frasier_openrave_controller.h>
#include <frasier_planner_server/PlanWaypoints.h>
#include <std_srvs/SetBool.h>
#include <detectron/GetDetections.h>
#include <frasier_planner_server/DetectObjects.h>
#include <frasier_planner_server/DetectedObjects.h>
#include <frasier_planner_server/PickAndPlace.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/bind.hpp>

class FRASIERPlannerServer{
    public:
        explicit FRASIERPlannerServer(ros::NodeHandle n);

        ~FRASIERPlannerServer   ();

        bool plannerServer(
            frasier_planner_server::PlanWaypoints::Request& req,
            frasier_planner_server::PlanWaypoints::Response& res
        );
        bool pickAndPlaceServer(
            frasier_planner_server::PickAndPlace::Request& req,
            frasier_planner_server::PickAndPlace::Response& res
        );
        bool homeServer(
            std_srvs::Empty::Request& req,
            std_srvs::Empty::Response& res
        );
        bool executeServer(
            std_srvs::Empty::Request& req,
            std_srvs::Empty::Response& res
        );
        bool graspReleaseServer(
            std_srvs::SetBool::Request& req,
            std_srvs::SetBool::Response& res
        );
        bool graspObjectServer(
            std_srvs::Empty::Request& req,
            std_srvs::Empty::Response& res
        );
        bool tableSegmentationServer(
            point_cloud_proc::SinglePlaneSegmentation::Request& req,
            point_cloud_proc::SinglePlaneSegmentation::Response& res
        );
        bool clusterObjectsServer(
            point_cloud_proc::TabletopClustering::Request& req,
            point_cloud_proc::TabletopClustering::Response& res
        );
        bool detectObjectsServer(
            frasier_planner_server::DetectObjects::Request& req,
            frasier_planner_server::DetectObjects::Response& res
        );

        void jointSensorCb(const sensor_msgs::JointState::ConstPtr &msg);

        void detectionThread();
        void downsampleThread();

    private:
        ros::NodeHandle nh;
        FRASIEROpenRAVE* rave_;
        FRASIERController* ctrl_;
        PointCloudProc* pcp_;
        ros::ServiceClient detect_objects_;
        std::vector<point_cloud_proc::Object> tabletop_objects_;
        trajectory_msgs::JointTrajectory planned_trajectory;
        sensor_msgs::JointState joints_;
        ros::Publisher object_poses_pub_;
        ros::Publisher filtered_cloud_pub;
        ros::Publisher detected_object_pub_;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> nav_cli;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_cli;
        boost::thread detection_thread_, downsample_thread_;
        frasier_planner_server::DetectedObjects detected_objects_;
};

FRASIERPlannerServer::FRASIERPlannerServer(ros::NodeHandle n) : nh(n),
                                                                nav_cli("/move_base/move", true),
                                                                arm_cli("/hsrb/arm_trajectory_controller/follow_joint_trajectory", true){

    std::string pkg_path = ros::package::getPath("frasier_planner_server");
    std::string pcp_config = pkg_path + "/config/table.yaml";
    detect_objects_ = nh.serviceClient<detectron::GetDetections>("/mask_rcnn/get_detections");
    nav_cli.waitForServer();
    arm_cli.waitForServer();

    rave_ = new FRASIEROpenRAVE(nh, true, true);
    ctrl_ = new FRASIERController(nh);
    pcp_ = new PointCloudProc(nh, true, pcp_config);

    ros::Subscriber joint_state_sub_ = nh.subscribe("/hsrb/robot_state/joint_states", 1, &FRASIERPlannerServer::jointSensorCb, this);
    ros::ServiceServer s_plan = nh.advertiseService("/frasier_planner_server/plan", &FRASIERPlannerServer::plannerServer, this);
    ros::ServiceServer s_home = nh.advertiseService("/frasier_planner_server/home", &FRASIERPlannerServer::homeServer, this);
    ros::ServiceServer s_exec = nh.advertiseService("/frasier_planner_server/execute", &FRASIERPlannerServer::executeServer, this);
    ros::ServiceServer s_grasp = nh.advertiseService("/frasier_planner_server/grasp_release", &FRASIERPlannerServer::graspReleaseServer, this);
    ros::ServiceServer s_grasp_object = nh.advertiseService("/frasier_planner_server/grasp_object", &FRASIERPlannerServer::graspObjectServer, this);
    ros::ServiceServer s_segment = nh.advertiseService("/frasier_planner_server/segment_plane", &FRASIERPlannerServer::tableSegmentationServer, this);
    ros::ServiceServer s_cluster = nh.advertiseService("/frasier_planner_server/cluster_objects", &FRASIERPlannerServer::clusterObjectsServer, this);
    ros::ServiceServer s_detect = nh.advertiseService("/frasier_planner_server/detect_objects", &FRASIERPlannerServer::detectObjectsServer, this);
    ros::ServiceServer s_pick_place = nh.advertiseService("/frasier_planner_server/pick_and_place", &FRASIERPlannerServer::pickAndPlaceServer, this);

    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/frasier_planner_server/cloud", 10);
    object_poses_pub_ = nh.advertise<geometry_msgs::PoseArray>("/frasier_planner_server/object_poses", 10);
    detected_object_pub_ = nh.advertise<frasier_planner_server::DetectedObjects>("/frasier_planner_server/detected_objects", 10);


    ctrl_->moveToKnownState(MOVE_STATE::PICK);
    ctrl_->moveHeadToKnownState(HEAD_STATE::LOOK_TABLE_FRONT);
    ctrl_->graspOrRelease(GRIPPER_STATE::RELEASE);
    ros::Duration(1.0).sleep();

    point_cloud_proc::SinglePlaneSegmentation::Request req;
    point_cloud_proc::SinglePlaneSegmentation::Response res;
    this->tableSegmentationServer(req, res);

    std::cout << "Starting threads..." << std::endl;

    detection_thread_ = boost::thread(boost::bind(&FRASIERPlannerServer::detectionThread, this));
    downsample_thread_ = boost::thread(boost::bind(&FRASIERPlannerServer::downsampleThread, this));

    ros::waitForShutdown();

}

FRASIERPlannerServer::~FRASIERPlannerServer(){
    detection_thread_.join();
    downsample_thread_.join();
}

void FRASIERPlannerServer::jointSensorCb(const sensor_msgs::JointState::ConstPtr &msg) {
    joints_ = *msg;
}

void FRASIERPlannerServer::detectionThread(){

    geometry_msgs::PoseArray object_poses_rviz;
    object_poses_rviz.header.frame_id = "map";

    ros::Rate rate(1);
    while(ros::ok()){
        detectron::GetDetections mask_rcnn_req;
        detect_objects_.call(mask_rcnn_req);

        int num_objects = mask_rcnn_req.response.detections.size();
        frasier_planner_server::DetectedObjects detected_objects;
        detected_objects.object_ids.resize(num_objects);
        detected_objects.object_poses.resize(num_objects);
        detected_objects.object_scores.resize(num_objects);
        object_poses_rviz.poses.clear();

        for (int i = 0; i < num_objects; i++){
            auto object = mask_rcnn_req.response.detections[i];

            point_cloud_proc::Object obj;
            bool got_object = pcp_->getObjectFromContour(object.contour_y, object.contour_x, obj);
            if(!got_object){
                continue;
            }
            object_poses_rviz.poses.push_back(obj.pose);
            detected_objects.object_ids[i] = object.class_name;
            detected_objects.object_poses[i] = obj.pose;
            detected_objects.object_scores[i] = object.score;

        }

        detected_objects_ = detected_objects;
        detected_object_pub_.publish(detected_objects);
        object_poses_pub_.publish(object_poses_rviz);
        rate.sleep();
    }

}

void FRASIERPlannerServer::downsampleThread(){
    ros::Rate rate(10);
    while(ros::ok()){
        sensor_msgs::PointCloud2 cloud;
        pcp_->getFilteredCloud(cloud);
        filtered_cloud_pub.publish(cloud);
        ros::spinOnce();
        rate.sleep();
    }
}


bool FRASIERPlannerServer::plannerServer(
    frasier_planner_server::PlanWaypoints::Request& req,
    frasier_planner_server::PlanWaypoints::Response& res){
    int num_waypoints = req.waypoints.size();

    std::cout << "Planning for " << num_waypoints << " end-effector goals..." << std::endl;
    EEFPoseGoals eef_goals(num_waypoints);

    eef_goals.wrt_world = true;

    for (int i = 0; i < num_waypoints; i++ ){
        OpenRAVE::Transform goal_pose(
            OpenRAVE::Vector(
                req.waypoints[i].orientation.w,
                req.waypoints[i].orientation.x,
                req.waypoints[i].orientation.y,
                req.waypoints[i].orientation.z
            ),
            OpenRAVE::Vector(
                req.waypoints[i].position.x,
                req.waypoints[i].position.y,
                req.waypoints[i].position.z
            )
        );

        eef_goals.poses[i] = goal_pose;
        float pos_diff = 10000.0;
        if (i == 0){
            OpenRAVE::Transform eef_pose = rave_->getEEFTransform();
            pos_diff = sqrt(
                pow(goal_pose.trans.x - eef_pose.trans.x, 2) +
                pow(goal_pose.trans.y - eef_pose.trans.y, 2) +
                pow(goal_pose.trans.z - eef_pose.trans.z, 2)
            );
        } else{
            pos_diff = sqrt(
                pow(goal_pose.trans.x - eef_goals.poses[i-1].trans.x, 2) +
                pow(goal_pose.trans.y - eef_goals.poses[i-1].trans.y, 2) +
                pow(goal_pose.trans.z - eef_goals.poses[i-1].trans.z, 2)
            );
        }

        if (pos_diff < 20.0){
            eef_goals.no_waypoints = 5 * num_waypoints;
            eef_goals.timesteps[i] = (i + 1) * 5 - 1;
        } else{
            eef_goals.no_waypoints = 10 * num_waypoints;
            eef_goals.timesteps[i] = (i + 1) * 10 - 1;
        }

    }

    res.trajectory = rave_->computeTrajectory(eef_goals, false);
    rave_->smoothTrajectory(res.trajectory, planned_trajectory);

    return true;
}

bool FRASIERPlannerServer::pickAndPlaceServer(
    frasier_planner_server::PickAndPlace::Request& req,
    frasier_planner_server::PickAndPlace::Response& res){

    EEFPoseGoals pick_goal(1);
    pick_goal.wrt_world = true;
    pick_goal.no_waypoints = 10;
    pick_goal.timesteps[0] = 9;

    EEFPoseGoals place_goal(1);
    place_goal.wrt_world = true;
    place_goal.no_waypoints = 10;
    place_goal.timesteps[0] = 9;

    OpenRAVE::Transform pick_pose(
        OpenRAVE::Vector(0.0, 0.707, 0.0, 0.707),
        OpenRAVE::Vector(
            req.pick_pose.position.x - 0.05,
            req.pick_pose.position.y,
            req.pick_pose.position.z
        )
    );
    pick_goal.poses[0] = pick_pose;

    OpenRAVE::Transform place_pose(
        OpenRAVE::Vector(
                req.place_pose.orientation.w,
                req.place_pose.orientation.x,
                req.place_pose.orientation.y,
                req.place_pose.orientation.z
                ),
        OpenRAVE::Vector(
            req.place_pose.position.x,
            req.place_pose.position.y,
            req.place_pose.position.z
        )
    );
    place_goal.poses[0] = place_pose;

    trajectory_msgs::JointTrajectory trajectory_pick, trajectory_place;
    trajectory_msgs::JointTrajectory smooth_trajectory_pick, smooth_trajectory_place;
    trajectory_pick = rave_->computeTrajectory(pick_goal, false);
    rave_->smoothTrajectory(trajectory_pick, smooth_trajectory_pick);
    ctrl_->executeWholeBodyTraj(smooth_trajectory_pick);
    std_srvs::SetBool grasp_release;
    grasp_release.request.data = true;
    this->graspReleaseServer(grasp_release.request, grasp_release.response);
    std::string object_name = "tabletop_1";
    rave_->grabObject(object_name);

    trajectory_place = rave_->computeTrajectory(place_goal, false);
    rave_->smoothTrajectory(trajectory_place, smooth_trajectory_place);
    ctrl_->executeWholeBodyTraj(smooth_trajectory_place);
    grasp_release.request.data = false;
    this->graspReleaseServer(grasp_release.request, grasp_release.response);
    rave_->releaseObject(object_name);

    return true;
    }

bool FRASIERPlannerServer::homeServer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    OpenRAVE::Transform eef_pose = rave_->getEEFTransform();

    sensor_msgs::JointState joints = rave_->getWholeBodyState();

    control_msgs::FollowJointTrajectoryGoal arm_goal;
    trajectory_msgs::JointTrajectory arm_traj;

    arm_traj.joint_names.push_back("arm_lift_joint");
    arm_traj.joint_names.push_back("arm_flex_joint");
    arm_traj.joint_names.push_back("arm_roll_joint");
    arm_traj.joint_names.push_back("wrist_flex_joint");
    arm_traj.joint_names.push_back("wrist_roll_joint");

    arm_traj.points.resize(1);
    arm_traj.points[0].positions.resize(5);

    arm_traj.points[0].positions[0] = joints_.position[14] + 0.20;
    arm_traj.points[0].positions[1] = joints_.position[15];
    arm_traj.points[0].positions[2] = joints_.position[16];
    arm_traj.points[0].positions[3] = joints_.position[17];
    arm_traj.points[0].positions[4] = joints_.position[18];

    arm_traj.points[0].time_from_start = ros::Duration(3.0);

    arm_goal.trajectory = arm_traj;
    arm_cli.sendGoal(arm_goal);
    arm_cli.waitForResult();

    geometry_msgs::PoseStamped home_pose_nav;
    home_pose_nav.header.frame_id = "map";
    home_pose_nav.header.stamp = ros::Time::now();
    home_pose_nav.pose.position.x = -0.1;
    home_pose_nav.pose.position.y = 0;
    tf::Quaternion home_q;
    home_q.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(home_q, home_pose_nav.pose.orientation);
    move_base_msgs::MoveBaseGoal home_goal;
    home_goal.target_pose = home_pose_nav;

    nav_cli.sendGoal(home_goal);
    nav_cli.waitForResult();
    ctrl_->moveToKnownState(MOVE_STATE::PICK);
    ctrl_->moveHeadToKnownState(HEAD_STATE::LOOK_TABLE_FRONT);

    return true;

}

bool FRASIERPlannerServer::executeServer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    ctrl_->executeWholeBodyTraj(planned_trajectory);
    return true;
}

bool FRASIERPlannerServer::graspReleaseServer(
    std_srvs::SetBool::Request& req,
    std_srvs::SetBool::Response& res){

    auto gripper_joint_it = std::find(joints_.name.begin(), joints_.name.end(), "hand_motor_joint");
    int gripper_joint_idx = std::distance(joints_.name.begin(), gripper_joint_it);

    if (req.data){
        ctrl_->graspOrRelease(GRIPPER_STATE::GRASP);
        if (joints_.position[gripper_joint_idx] > -0.7){
                res.success = true;
            }
            else{
                res.success = false;
            }
    } else{
        ctrl_->graspOrRelease(GRIPPER_STATE::RELEASE);
        res.success = true;
    }
    std::cout << joints_.position[gripper_joint_idx] << std::endl;

    return true;
}

bool FRASIERPlannerServer::graspObjectServer(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res){

    if (tabletop_objects_.empty()){
        std::cout << "Call object clustering first!" << std::endl;
        return true;
    }

    float min_object_distance = 10000;
    int min_object_index = 0;

    for (int i=0; i < tabletop_objects_.size(); i++){
        auto object = tabletop_objects_[i];
        float object_distance = std::sqrt(
            std::pow(object.pose.position.x, 2) +
            std::pow(object.pose.position.y, 2) +
            std::pow(object.pose.position.z, 2)
        );
        if (object_distance < min_object_distance){
            min_object_distance = object_distance;
            min_object_index = i;
        }
    }

    EEFPoseGoals eef_goals(1);
    eef_goals.wrt_world = true;

    auto closest_object = tabletop_objects_[min_object_index];
    OpenRAVE::Transform goal_pose(
        OpenRAVE::Vector(
            closest_object.pose.orientation.w,
            closest_object.pose.orientation.x,
            closest_object.pose.orientation.y,
            closest_object.pose.orientation.z
        ),
        OpenRAVE::Vector(
            closest_object.pose.position.x,
            closest_object.pose.position.y,
            closest_object.pose.position.z + 0.11
        )
    );

    eef_goals.poses[0] = goal_pose;
    eef_goals.no_waypoints = 10;
    eef_goals.timesteps[0] = 9;

    trajectory_msgs::JointTrajectory trajectory, smooth_trajectory;
    trajectory = rave_->computeTrajectory(eef_goals, false);
    rave_->smoothTrajectory(trajectory, smooth_trajectory);
    ctrl_->executeWholeBodyTraj(smooth_trajectory);

    std_srvs::SetBool grasp_release;
    grasp_release.request.data = true;
    this->graspReleaseServer(grasp_release.request, grasp_release.response);
    std::string object_name = "tabletop_" + std::to_string(min_object_index + 1);
    rave_->grabObject(object_name);

    return true;
}


bool FRASIERPlannerServer::tableSegmentationServer(
            point_cloud_proc::SinglePlaneSegmentation::Request& req,
            point_cloud_proc::SinglePlaneSegmentation::Response& res){

    point_cloud_proc::Plane table;
    bool table_segmented = pcp_->segmentSinglePlane(table);
    if (table_segmented) {
        // Creating table collision object
        std::string table_name = "table";
        OpenRAVE::Vector table_size((table.max.x - table.min.x), (table.max.y - table.min.y), table.max.z);
        OpenRAVE::Transform table_pose(OpenRAVE::Vector(1, 0, 0, 0), OpenRAVE::Vector((table.max.x + table.min.x) / 2,
                                                                                      (table.max.y + table.min.y) / 2,
                                                                                      (table.max.z) / 2));

        rave_->addBoxCollObj(table_size, table_pose, table_name);
    }

    res.success = table_segmented;
    res.plane_object = table;

    return table_segmented;
}

bool FRASIERPlannerServer::clusterObjectsServer(
            point_cloud_proc::TabletopClustering::Request& req,
            point_cloud_proc::TabletopClustering::Response& res){

    rave_->removeTableObjects();
    tabletop_objects_.clear();
    std::vector<point_cloud_proc::Object> tabletop_objects;
    bool tabletop_clustered = pcp_->clusterObjects(tabletop_objects);
    tabletop_objects_ = tabletop_objects;
    if (tabletop_clustered) {
        // Create tabletop collision objects
        int object_count = 1;
        for (auto object : tabletop_objects) {
            std::string obj_name = "tabletop_" + std::to_string(object_count);
            OpenRAVE::Vector obj_size((object.max.x - object.min.x), (object.max.y - object.min.y),
                                        (object.max.z - object.min.z));
            // OpenRAVE::VEctor obj_rot()
            OpenRAVE::Transform obj_pose(
                OpenRAVE::Vector(object.pose.orientation.w,
                                 object.pose.orientation.x,
                                 object.pose.orientation.y,
                                 object.pose.orientation.z),
                OpenRAVE::Vector(
                    (object.max.x + object.min.x) / 2,
                    (object.max.y + object.min.y) / 2,
                    (object.max.z + object.min.z) / 2
                                )
                            );
            rave_->addBoxCollObj(obj_size, obj_pose, obj_name);
            object_count++;
        }

    res.success = tabletop_clustered;
    res.objects = tabletop_objects;

    return true;

    }
}

bool FRASIERPlannerServer::detectObjectsServer(
            frasier_planner_server::DetectObjects::Request& req,
            frasier_planner_server::DetectObjects::Response& res){

    detectron::GetDetections mask_rcnn_req;
    geometry_msgs::PoseArray object_poses_rviz;

    object_poses_rviz.header.frame_id = "map";

    detect_objects_.call(mask_rcnn_req);
    int num_objects = mask_rcnn_req.response.detections.size();
    res.object_poses.resize(num_objects);
    res.object_ids.resize(num_objects);
    res.object_scores.resize(num_objects);
    for (int i = 0; i < num_objects; i++){
        auto detected_object = mask_rcnn_req.response.detections[i];
        std::cout << detected_object.class_name << std::endl;

        point_cloud_proc::Object obj;
        pcp_->getObjectFromContour(detected_object.contour_y, detected_object.contour_x, obj);
        object_poses_rviz.poses.push_back(obj.pose);
        res.object_ids[i] = detected_object.class_name;
        res.object_poses[i] = obj.pose;
        res.object_scores[i] = detected_object.score;
    }

    object_poses_pub_.publish(object_poses_rviz);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "frasier_planner_server");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    FRASIERPlannerServer server(nh);
    // ros::spin();

}

