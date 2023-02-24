#include <frasier_openrave/frasier_openrave.h>
#include <frasier_openrave/frasier_openrave_controller.h>
#include <frasier_manipulation_msgs/GraspObject.h>
#include <point_cloud_proc/point_cloud_proc.h>
#include <darknet_ros/GetDetections.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>

class GraspGiveService {

public:
    GraspGiveService(ros::NodeHandle n) : nh_(n){

      controller = new FRASIERController(n);
      rave = new FRASIEROpenRAVE(n, false, true);
      pcp = new PointCloudProc(n, true);

      detect_objects_cli_ = nh_.serviceClient<darknet_ros::GetDetections>("/frasier/darknet_ros/detect");

      grasp_srv_ = nh_.advertiseService ("/frasier/gpsr/grasp", &GraspGiveService::grasp, this);
      give_srv_ = nh_.advertiseService ("/frasier/gpsr/give", &GraspGiveService::give, this);
      drop_srv_ = nh_.advertiseService ("/frasier/gpsr/drop", &GraspGiveService::drop, this);
      wrist_sensor_sub_ = nh_.subscribe("/hsrb/wrist_wrench/compensated", 1, &GraspGiveService::wristSensorCb, this);

      ros::AsyncSpinner spinner(4);
      spinner.start();

      rave->loadHSR();
      rave->startThreads();
      ros::Duration(1.0).sleep();

      ros::waitForShutdown();
      drop_count = 1;
      grasp_count = 1;

    }

    void wristSensorCb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
      wrench_ = *msg;
    }

    bool grasp(frasier_manipulation_msgs::GraspObject::Request &req, frasier_manipulation_msgs::GraspObject::Response &res){

      std::cout << "GPSR: starting grasp service..." << std::endl;
      controller->graspOrRelease(GRIPPER_STATE::RELEASE);
//      controller->moveArmToKnownState(ARM_STATE::GRASP_CONF);
      controller->moveArmToKnownState(ARM_STATE::GO_CONF);
//      controller->moveHeadToKnownState(HEAD_STATE::LOOK_TABLE_FRONT);
      ros::Duration(2.0).sleep();
//    ASK FOR OBJECT HERE
      ros::Rate rate(20);
      while(true){
        if (wrench_.wrench.force.x > 15){
          controller->graspOrRelease(GRIPPER_STATE::GRASP);
          break;
        }
        rate.sleep();
      }
	
//      std::string table_name = "table_grasp_" + std::to_string(grasp_count);
//      grasp_count++;
//      point_cloud_proc::Plane table;
//      bool table_segmented = pcp->segmentSinglePlane(table);
//      if (table_segmented){
//        // Creating table collision object
//
//        OpenRAVE::Vector table_size((table.max.x-table.min.x), (table.max.y-table.min.y), table.max.z);
//        OpenRAVE::Transform table_pose(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((table.max.x + table.min.x)/2,
//                                                                                   (table.max.y + table.min.y)/2,
//                                                                                   (table.max.z)/2));
//
//        rave->addBoxCollObj(table_size, table_pose, table_name);
//      }

//      sensor_msgs::PointCloud2 cloud;
//      pcp->getFilteredCloud(cloud);
//      pcl_msgs::PolygonMesh cloud_mesh;
//      pcp->trianglePointCloud(cloud, cloud_mesh);
//      std::string cloud_name = "cloud";
//      rave->addMeshCollObj(cloud_mesh, cloud_name);
//
//      darknet_ros::GetDetections object_detection;
//      detect_objects_cli_.call(object_detection);
//      bool found_object = false;
//      std::string obj_name;
//      for(auto object : object_detection.response.objects){
//        if (object.label == req.object_name){
//          point_cloud_proc::Object object_shape;
//          if(!pcp->getObjectFromBBox(object.bbox.data(), object_shape)){
//            res.success = 0;
//            std::cout << "GPSR: couldn't get object shape!!" << std::endl;
//            return true;
//          }
//
//          obj_name = "gpsr_object_" + object.label;
//          OpenRAVE::Vector obj_size((object_shape.max.x-object_shape.min.x),
//                                    (object_shape.max.y-object_shape.min.y),
//                                    (object_shape.max.z-object_shape.min.z));
//          OpenRAVE::Transform obj_pose(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((object_shape.max.x + object_shape.min.x)/2,
//                                                                                   (object_shape.max.y + object_shape.min.y)/2,
//                                                                                   (object_shape.max.z + object_shape.min.z)/2));
//
//          rave->addBoxCollObj(obj_size, obj_pose, obj_name);
//
//          found_object = true;
//
//        }
//
//      }

//      if(!found_object){
//        res.success = 0;
//        std::cout << "GPSR: couldn't find the object!" << std::endl;
//        rave->removeCollisionObj(cloud_name);
//        return true;
//      }
//
//
//      Grasp grasp = rave->generateGraspPose(0, obj_name);
//
//      geometry_msgs::Pose2D hsr_pose = rave->getRobotPose();
//
//      /// PICKING ///
//      EEFPoseGoals eef_grasp_goal(1);
//      eef_grasp_goal.wrt_world = true;
//      eef_grasp_goal.no_waypoints = 5;
//      eef_grasp_goal.timesteps[0] = 4;
//      eef_grasp_goal.poses[0] = grasp.pose;
//
//      trajectory_msgs::JointTrajectory traj_grasp = rave->computeTrajectory(eef_grasp_goal);
//      trajectory_msgs::JointTrajectory traj_grasp_smooth;
//      rave->smoothTrajectory(traj_grasp, traj_grasp_smooth);
//      controller->executeWholeBodyTraj(traj_grasp_smooth);
//
//      controller->graspOrRelease(GRIPPER_STATE::GRASP);
//
//      rave->grabObject(obj_name);
//
//      JointPosGoals pick_joint_goal;
//      pick_joint_goal.no_waypoints = 10;
//      pick_joint_goal.q = {hsr_pose.x, hsr_pose.y, hsr_pose.theta, 0.0, 0.0, -M_PI/2, -M_PI/2, 0.0};
//      trajectory_msgs::JointTrajectory traj_pick = rave->computeTrajectory(pick_joint_goal);
//      trajectory_msgs::JointTrajectory traj_pick_smooth;
//      rave->smoothTrajectory(traj_pick, traj_pick_smooth);
//      controller->executeWholeBodyTraj(traj_pick_smooth);
//
//      rave->removeCollisionObj(cloud_name);
////      rave->removeCollisionObj(obj_name);
      res.success = 1;
      return true;
    }

    bool give(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
      controller->moveArmToKnownState(ARM_STATE::GIVE_CONF);
      ros::Rate rate(50);
      while(ros::ok()){
        if (wrench_.wrench.force.z < -10){
          controller->graspOrRelease(GRIPPER_STATE::RELEASE);
          break;
        }
        rate.sleep();
      }

      ros::Duration(1.0).sleep();
      controller->moveArmToKnownState(ARM_STATE::GO_CONF);

      return true;
    }

    bool drop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

      controller->moveArmToKnownState(ARM_STATE::GRASP_CONF);
      controller->moveHeadToKnownState(HEAD_STATE::LOOK_TABLE_FRONT);
      ros::Duration(2.0).sleep();

      std::string table_name = "table_drop_" + std::to_string(drop_count);
      drop_count++;
      point_cloud_proc::Plane table;
      bool table_segmented = pcp->segmentSinglePlane(table);
      if (table_segmented){
        // Creating table collision object

        OpenRAVE::Vector table_size((table.max.x-table.min.x), (table.max.y-table.min.y), table.max.z);
        OpenRAVE::Transform table_pose(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((table.max.x + table.min.x)/2,
                                                                                   (table.max.y + table.min.y)/2,
                                                                                   (table.max.z)/2));

        rave->addBoxCollObj(table_size, table_pose, table_name);
      }

      OpenRAVE::Transform drop_pose(OpenRAVE::Vector(0.0, 0.707, 0.0, 0.707), OpenRAVE::Vector((table.max.x + table.min.x)/2,
                                                                                 (table.max.y + table.min.y)/2,
                                                                                 (table.max.z + 0.15)));

      geometry_msgs::Pose2D hsr_pose = rave->getRobotPose();

      /// PICKING ///
      EEFPoseGoals eef_grasp_goal(1);
      eef_grasp_goal.wrt_world = true;
      eef_grasp_goal.no_waypoints = 5;
      eef_grasp_goal.timesteps[0] = 4;
      eef_grasp_goal.poses[0] = drop_pose;

      trajectory_msgs::JointTrajectory traj_grasp = rave->computeTrajectory(eef_grasp_goal);
      trajectory_msgs::JointTrajectory traj_grasp_smooth;
      rave->smoothTrajectory(traj_grasp, traj_grasp_smooth);
      controller->executeWholeBodyTraj(traj_grasp_smooth);

      controller->graspOrRelease(GRIPPER_STATE::RELEASE);

      JointPosGoals pick_joint_goal;
      pick_joint_goal.no_waypoints = 10;
      pick_joint_goal.q = {hsr_pose.x, hsr_pose.y, hsr_pose.theta, 0.0, 0.0, -M_PI/2, -M_PI/2, 0.0};
      trajectory_msgs::JointTrajectory traj_pick = rave->computeTrajectory(pick_joint_goal);
      trajectory_msgs::JointTrajectory traj_pick_smooth;
      rave->smoothTrajectory(traj_pick, traj_pick_smooth);
      controller->executeWholeBodyTraj(traj_pick_smooth);

      return true;
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer grasp_srv_, give_srv_, drop_srv_;
    ros::Subscriber wrist_sensor_sub_;
    ros::ServiceClient detect_objects_cli_;
    geometry_msgs::WrenchStamped wrench_;

    FRASIEROpenRAVE *rave;
    FRASIERController *controller;
    PointCloudProc *pcp;

    int grasp_count, drop_count;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "frasier_grasp_give_service");
  ros::NodeHandle nh;
  GraspGiveService run(nh);
  return 0;
}
