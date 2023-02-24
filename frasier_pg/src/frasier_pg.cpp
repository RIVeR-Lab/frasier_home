#include <ros/ros.h>
//#include <gpd/detect_grasps.h>
#include <gpd/CloudIndexed.h>
#include <gpd/GraspConfigList.h>
#include <frasier_openrave/frasier_openrave.h>
#include <frasier_openrave/frasier_openrave_controller.h>
#include <point_cloud_proc/point_cloud_proc.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class GPDClient{

public:
  GPDClient(ros::NodeHandle n) : nh_(n){
//    gpd_detect_grasps_ = nh_.serviceClient<gpd::detect_grasps> ("/detect_grasps_server/detect_grasps");
    gpd_pub_ = nh_.advertise<gpd::CloudIndexed>("/frasier/gpd_cloud", 10);
    gpd_sub_ = nh_.subscribe("/detect_grasps/clustered_grasps", 10, &GPDClient::gpdCallback, this);
    grasp_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/frasier/grasp_pose", 10);

    grasps_receieved_ = false;

    ros::Duration(1.0).sleep(); // setlle down
  }

    void gpdCallback(const gpd::GraspConfigListPtr grasps_msg){
      detected_grasps_ = *grasps_msg;
      grasps_receieved_ = true;
    }


    void detectGraspPoses(pcl::PointIndices::Ptr sample_indicies, pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole_cloud) {
    gpd::CloudIndexed cloud_indexed;
    geometry_msgs::Point camera_viewpoint;
    camera_viewpoint.x = 0;
    camera_viewpoint.y = 0;
    camera_viewpoint.z = 0;
    std_msgs::Int64 camera_source, point;
    camera_source.data = 0;
    sensor_msgs::PointCloud2 whole_cloud_ros;
    pcl::toROSMsg(*whole_cloud, whole_cloud_ros);
    cloud_indexed.cloud_sources.view_points.push_back(camera_viewpoint);
    cloud_indexed.cloud_sources.cloud =  whole_cloud_ros;
    for (int i = 0; i < whole_cloud->width; i++) {
      cloud_indexed.cloud_sources.camera_source.push_back(camera_source);
    }

    for (int i = 0; i < sample_indicies->indices.size(); i++) {
      point.data = sample_indicies->indices[i];
      cloud_indexed.indices.push_back(point);
    }

    gpd_pub_.publish(cloud_indexed);



  }

  OpenRAVE::Transform getGraspPose(){
    ros::Rate rate(30);

    while (ros::ok()){
      if (grasps_receieved_){
        std::cout << "TASK: received grasps from GPD!" << std::endl;
        gpd::GraspConfig grasp_config = detected_grasps_.grasps[0];

        tf::Matrix3x3 grasp_rot(-grasp_config.axis.x, grasp_config.binormal.x, grasp_config.approach.x,
                                            -grasp_config.axis.y, grasp_config.binormal.y, grasp_config.approach.y,
                                            -grasp_config.axis.z, grasp_config.binormal.z, grasp_config.approach.z);

        tf::Quaternion grasp_q;
        grasp_rot.getRotation(grasp_q);
        OpenRAVE::Transform grasp_pose(OpenRAVE::Vector(grasp_q.w(), grasp_q.x(), grasp_q.y(), grasp_q.z()),
                                       OpenRAVE::Vector(grasp_config.bottom.x, grasp_config.bottom.y, grasp_config.bottom.z));

        grasps_receieved_ = false;
        visualizeGraspPose(grasp_pose);
        return grasp_pose;
      }
      rate.sleep();
    }

  }

    void visualizeGraspPose(OpenRAVE::Transform T){

      geometry_msgs::PoseStamped pose;

      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "/base_link";
      pose.pose.position.x = T.trans.x;
      pose.pose.position.y = T.trans.y;
      pose.pose.position.z = T.trans.z;

      pose.pose.orientation.w = T.rot[0];
      pose.pose.orientation.x = T.rot[1];
      pose.pose.orientation.y = T.rot[2];
      pose.pose.orientation.z = T.rot[3];

      grasp_pose_pub_.publish(pose);
    }

//  OpenRAVE::Transform detectGraspPoses(pcl::PointIndices::Ptr sample_indicies, pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole_cloud) {
//    gpd::detect_grasps detect_grasps;
//    geometry_msgs::Point camera_viewpoint;
//    camera_viewpoint.x = 0;
//    camera_viewpoint.y = 0;
//    camera_viewpoint.z = 0;
//    std_msgs::Int64 camera_source, point;
//    camera_source.data = 0;
//    sensor_msgs::PointCloud2 whole_cloud_ros;
//    pcl::toROSMsg(*whole_cloud, whole_cloud_ros);
//    detect_grasps.request.cloud_indexed.cloud_sources.view_points.push_back(camera_viewpoint);
//    detect_grasps.request.cloud_indexed.cloud_sources.cloud =  whole_cloud_ros;
//    for (int i = 0; i < whole_cloud->width; i++) {
//      detect_grasps.request.cloud_indexed.cloud_sources.camera_source.push_back(camera_source);
//    }
//
//    for (int i = 0; i < sample_indicies->indices.size(); i++) {
//      point.data = sample_indicies->indices[i];
//      detect_grasps.request.cloud_indexed.indices.push_back(point);
//    }
//
//    gpd_detect_grasps_.call(detect_grasps);
//
//    gpd::GraspConfigList grasp_configs = detect_grasps.response.grasp_configs;
//    gpd::GraspConfig grasp_config = grasp_configs.grasps[0];
//
//    Eigen::Matrix3d grasp_rot;
//    grasp_rot << -grasp_config.axis.x, grasp_config.binormal.x, grasp_config.approach.x,
//      -grasp_config.axis.y, grasp_config.binormal.y, grasp_config.approach.y,
//      -grasp_config.axis.z, grasp_config.binormal.z, grasp_config.approach.z;
//    Eigen::Quaterniond grasp_q(grasp_rot);
//
//    OpenRAVE::Transform grasp_pose(OpenRAVE::Vector(grasp_q.w(), grasp_q.x(), grasp_q.y(), grasp_q.z()),
//                                   OpenRAVE::Vector(grasp_config.bottom.x, grasp_config.bottom.y, grasp_config.bottom.z));
//
//    return grasp_pose;
//
//  }


private:
  ros::NodeHandle nh_;
  ros::ServiceClient gpd_detect_grasps_;
  ros::Publisher gpd_pub_, grasp_pose_pub_;
  ros::Subscriber gpd_sub_;
  gpd::GraspConfigList detected_grasps_;
  bool grasps_receieved_;

};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "frasier_pg_challenge");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);

  FRASIEROpenRAVE rave(nh, false, true);
  FRASIERController controller(nh);
  PointCloudProc pcp(nh, true);
  GPDClient gpd(nh);

  spinner.start();

  rave.loadHSR();
  rave.startThreads();
  ros::Duration(1.0).sleep();
  controller.moveToKnownState(MOVE_STATE::SHELF);
  controller.graspOrRelease(GRIPPER_STATE::RELEASE);

  ////////// SEGMENT TABLE //////////
  controller.moveHeadToKnownState(HEAD_STATE::LOOK_TABLE_FRONT);
  ros::Duration(2.0).sleep();

  // Segment table
  point_cloud_proc::Plane table;
  bool table_segmented = pcp.segmentSinglePlane(table);
  if (table_segmented){
    // Creating table collision object
    std::string table_name = "table";
    OpenRAVE::Vector table_size((table.max.x-table.min.x), (table.max.y-table.min.y), table.max.z);
    OpenRAVE::Transform table_pose(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((table.max.x + table.min.x)/2,
                                                                               (table.max.y + table.min.y)/2,
                                                                               (table.max.z)/2));

    rave.addBoxCollObj(table_size, table_pose, table_name);
  }
  //////////////////////////////////////////////////

  ///////// CLUSTER TABLETOP OBJECTS //////////
  std::vector<point_cloud_proc::Object> tabletop_objects;
  bool tabletop_clustered = pcp.clusterObjects(tabletop_objects);
  std::cout << "TASK: number of objects: " << tabletop_objects.size() << std::endl;
  if (tabletop_clustered){
    // Create tabletop collision objects
    int object_count = 1;
    for (auto object : tabletop_objects) {
      std::string obj_name = "table_obj_" + std::to_string(object_count);
      OpenRAVE::Vector obj_size((object.max.x-object.min.x), (object.max.y-object.min.y), (object.max.z-object.min.z));
      OpenRAVE::Transform obj_pose(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((object.max.x + object.min.x)/2,
                                                                               (object.max.y + object.min.y)/2,
                                                                               (object.max.z + object.min.z)/2));
      rave.addBoxCollObj(obj_size, obj_pose, obj_name);
      object_count++;
    }

  }
  else{
    std::cout << "TASK: there is no object left on the table!!!" << std::endl;
    ros::shutdown();
    return 0;
  }

  pcl::PointIndices::Ptr tabletop_indicies(new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  tabletop_indicies  = pcp.getTabletopIndicies();
//  sensor_msgs::PointCloud2::Ptr filtered_cloud;
  filtered_cloud = pcp.getFilteredCloud();

  gpd.detectGraspPoses(tabletop_indicies, filtered_cloud);
  OpenRAVE::Transform grasp_pose = gpd.getGraspPose();

  /// PICKING ///
  EEFPoseGoals eef_grasp_goal(1);
  eef_grasp_goal.wrt_world = false;
  eef_grasp_goal.no_waypoints = 5;
  eef_grasp_goal.timesteps[0] = 4;
  eef_grasp_goal.poses[0] = grasp_pose;

  trajectory_msgs::JointTrajectory traj_grasp = rave.computeTrajectory(eef_grasp_goal);
  trajectory_msgs::JointTrajectory traj_grasp_smooth;
  rave.smoothTrajectory(traj_grasp, traj_grasp_smooth);
  controller.executeWholeBodyTraj(traj_grasp_smooth);

  controller.graspOrRelease(GRIPPER_STATE::GRASP);


  ros::waitForShutdown();
  return 0;
}
