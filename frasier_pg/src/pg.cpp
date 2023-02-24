#include <ros/ros.h>
//#include <gpd/detect_grasps.h>
#include <gpd/CloudIndexed.h>
#include <gpd/GraspConfigList.h>
#include <frasier_openrave/frasier_openrave.h>
#include <frasier_openrave/frasier_openrave_controller.h>
#include <point_cloud_proc/point_cloud_proc.h>
#include <frasier_nav_client/NavigateTo.h>
#include <tmc_msgs/Voice.h>
#include <geometry_msgs/WrenchStamped.h>

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
        OpenRAVE::Transform grasp_pose_closest;
          gpd::GraspConfig grasp_config = detected_grasps_.grasps[0];
          std::cout << "TASK: received grasps from GPD!" << std::endl;

          tf::Matrix3x3 grasp_rot(-grasp_config.axis.x, grasp_config.binormal.x, grasp_config.approach.x,
                                  -grasp_config.axis.y, grasp_config.binormal.y, grasp_config.approach.y,
                                  -grasp_config.axis.z, grasp_config.binormal.z, grasp_config.approach.z);

          tf::Quaternion grasp_q;
          grasp_rot.getRotation(grasp_q);
          OpenRAVE::Transform grasp_pose(OpenRAVE::Vector(grasp_q.w(), grasp_q.x(), grasp_q.y(), grasp_q.z()),
                                         OpenRAVE::Vector(grasp_config.bottom.x, grasp_config.bottom.y, grasp_config.bottom.z));


        grasps_receieved_ = false;
//        visualizeGraspPose(grasp_pose_closest);
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

geometry_msgs::WrenchStamped wrench_;
void wristSensorCb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
  wrench_ = *msg;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "frasier_pg_challenge");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  ros::ServiceClient nav_cli = nh.serviceClient<frasier_nav_client::NavigateTo>("/frasier_navigation/nav_cmd");
  ros::Subscriber wrist_sensor_sub_ = nh.subscribe("/hsrb/wrist_wrench/compensated", 1, wristSensorCb);
  ros::Publisher talk_pub = nh.advertise<tmc_msgs::Voice>("/talk_request", 10);
  FRASIEROpenRAVE rave(nh, false, true);
  FRASIERController controller(nh);
  PointCloudProc pcp(nh, true);
  GPDClient gpd(nh);

  spinner.start();

  rave.loadHSR();
  rave.startThreads();
  ros::Duration(2.0).sleep();
  // controller.moveToKnownState(MOVE_STATE::SHELF);
  // controller.graspOrRelease(GRIPPER_STATE::RELEASE);


  controller.moveArmToKnownState(ARM_STATE::GO_CONF);

  tmc_msgs::Voice door_talk;
  door_talk.language = door_talk.kEnglish;
  door_talk.sentence = "Please open the door and touch my hand to start the task.";
  talk_pub.publish(door_talk);

  ros::Rate rate(50);

  while(ros::ok()){
    if (wrench_.wrench.force.x > 10){
      std::cout << "TASK: starting the task !" << std::endl;
      break;
    }
    rate.sleep();
  }
  ros::Duration(1.0).sleep();

  tmc_msgs::Voice door_talk_;
  door_talk_.language = door_talk_.kEnglish;
  door_talk_.sentence = "I will now go to dining table.";
  talk_pub.publish(door_talk_);

  int count = 0;
  while (true) {
    // GOTO TABLE
    frasier_nav_client::NavigateTo go_table;
    go_table.request.location = "dining_table_front";
    nav_cli.call(go_table);

    tmc_msgs::Voice table_talk;
    table_talk.language = table_talk.kEnglish;
    table_talk.sentence = "I have arrived. I will pick an object from the table.";
    talk_pub.publish(table_talk);

    controller.moveHeadToKnownState(HEAD_STATE::LOOK_TABLE_FRONT);
    controller.moveArmToKnownState(ARM_STATE::GRASP_CONF);
    controller.graspOrRelease(GRIPPER_STATE::RELEASE);
    ros::Duration(2.0).sleep();

    // Segment table
    point_cloud_proc::Plane table;
    bool table_segmented = pcp.segmentSinglePlane(table);
    if (table_segmented){
      // Creating table collision object
      std::string table_name = "table_" + std::to_string(count);
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
      break;
    }

    pcl::PointIndices::Ptr tabletop_indicies(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    tabletop_indicies  = pcp.getTabletopIndicies();
  //  sensor_msgs::PointCloud2::Ptr filtered_cloud;
    filtered_cloud = pcp.getFilteredCloud();

    gpd.detectGraspPoses(tabletop_indicies, filtered_cloud);
    OpenRAVE::Transform grasp_pose = gpd.getGraspPose();

    /// PICKING ///
    geometry_msgs::Pose2D hsr_pose = rave.getRobotPose();
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

    JointPosGoals pick_joint_goal;
    pick_joint_goal.no_waypoints = 10;
    pick_joint_goal.q = {hsr_pose.x, hsr_pose.y, hsr_pose.theta, 0.0, 0.0, 0.0, -M_PI/2, 0.0};
    trajectory_msgs::JointTrajectory traj_pick = rave.computeTrajectory(pick_joint_goal);
    trajectory_msgs::JointTrajectory traj_pick_smooth;
    rave.smoothTrajectory(traj_pick, traj_pick_smooth);
    controller.executeWholeBodyTraj(traj_pick_smooth);

    tmc_msgs::Voice open_dish_talk;
    open_dish_talk.language = open_dish_talk.kEnglish;
    open_dish_talk.sentence = "I am going to dishwasher. Can you open the door for me?";
    talk_pub.publish(open_dish_talk);

    // GO TO DISHWASHER
    frasier_nav_client::NavigateTo go_dish;
    go_dish.request.location = "dishwasher_place";
    nav_cli.call(go_dish);
    
    controller.moveArmToKnownState(ARM_STATE::DISH);

    frasier_nav_client::NavigateTo go_dish_place;
    go_dish_place.request.location = "dishwasher_place_turn";
    nav_cli.call(go_dish_place);
    controller.graspOrRelease(GRIPPER_STATE::RELEASE);

    nav_cli.call(go_dish);
    controller.moveArmToKnownState(ARM_STATE::GO_CONF);

    rave.removeTableObjects();
    std::string table_name = "table_" + std::to_string(count);
    rave.removeCollisionObj(table_name);
    count++;
    ros::Duration(2.0).sleep();
    tabletop_objects.clear();
    tabletop_clustered = false;
  }


  ros::shutdown();
  return 0;
}
