
#include <frasier_openrave/frasier_openrave.h>
#include <frasier_openrave/frasier_openrave_controller.h>
#include <point_cloud_proc/point_cloud_proc.h>
#include <darknet_ros/GetDetections.h>
#include <tmc_msgs/Voice.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/WrenchStamped.h>

#include <yaml-cpp/yaml.h>

geometry_msgs::WrenchStamped wrench_;
void wristSensorCb(const geometry_msgs::WrenchStamped::ConstPtr &msg){
  wrench_ = *msg;
}

void talk(ros::Publisher& talk_pub, std::string& sentence){
  tmc_msgs::Voice msg;
  msg.language = msg.kEnglish;
  msg.interrupting = 0;
  msg.interrupting = 0;
  msg.sentence = sentence;

  talk_pub.publish(msg);


}

int main(int argc, char **argv) {
  ros::init(argc, argv, "storing_groceries");
  ros::NodeHandle nh; 
  ros::AsyncSpinner spinner(4);

  ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("object_center", 10);
  ros::Publisher talk_pub = nh.advertise<tmc_msgs::Voice>("/talk_request", 10);
  ros::Subscriber wrist_sensor_sub_ = nh.subscribe("/hsrb/wrist_wrench/compensated", 1, wristSensorCb);

  bool OPEN_DOOR = false;
  bool RUN_VIEWER = false;
  bool HAND = true;
  HEAD_STATE TABLE_POSE = HEAD_STATE::LOOK_TABLE_LEFT;

  if(argc > 1){
    for (int i = 1; i < argc; i ++){
      if(std::string(argv[i]) == "-v" || std::string(argv[i]) == "--view"){
        std::cout << "TASK: running the viewer!" << std::endl;
        RUN_VIEWER = true;
      }
      if(std::string(argv[i]) == "-d" || std::string(argv[i]) == "--door"){
        std::cout << "TASK: opening door!" << std::endl;
        OPEN_DOOR = true;
      }
      if(std::string(argv[i]) == "-h" || std::string(argv[i]) == "--hand"){
        std::cout << "TASK: don't touch hand" << std::endl;
        OPEN_DOOR = false;
      }

    }

  }

  FRASIEROpenRAVE rave(nh, RUN_VIEWER, true);
  FRASIERController controller(nh);
  PointCloudProc pcp(nh, true);
  spinner.start();

  rave.loadHSR();
  rave.startThreads();
  ros::Duration(1.0).sleep();

  std::string start_sentence = "Please touch my hand to start the task.";
  std::string door_open_sentence = "I am starting the task. Please open the door.";
  std::string shelf_inspect_sentence = "I am inspecting the shelf.";
  std::string table_inspect_sentence = "I am inspecting the table";
  std::string not_same = "I couldn't find the same category. I am going to place this object to a new space";
  talk(talk_pub, start_sentence);

  ros::Rate rate(50);
  
  while(ros::ok()){
    if (wrench_.wrench.force.x > 15){
      std::cout << "TASK: starting the task !" << std::endl;
       break;
      }
      rate.sleep();
    }
  
  ros::Duration(1.0).sleep();

//  controller.moveToKnownState(MOVE_STATE::SHELF);
  controller.moveArmToKnownState(ARM_STATE::GRASP_CONF);
  controller.graspOrRelease(GRIPPER_STATE::RELEASE);
  controller.moveHeadToKnownState(HEAD_STATE::LOOK_SHELF);

  talk(talk_pub, door_open_sentence);
  ros::Duration(7.0).sleep();

  if(OPEN_DOOR){
    controller.moveHeadToKnownState(HEAD_STATE::LOOK_SHELF);
    controller.graspOrRelease(GRIPPER_STATE::GRASP);
    point_cloud_proc::Plane plane;
    pcp.segmentSinglePlane(plane, 'z');

//    std::string door_name = "door";
//    OpenRAVE::Vector door_size(0.05, (plane.max.y-plane.min.y), (plane.max.z-plane.min.z));
//    OpenRAVE::Transform door_pose(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((plane.max.x + plane.min.x)/2,
//                                                                               (plane.max.y + plane.min.y)/2,
//                                                                               (plane.max.z + plane.min.z)/2));
//
//    rave.addBoxCollObj(door_size, door_pose, door_name);

    sensor_msgs::PointCloud2 remaining_cloud;
//    pcp.getRemainingCloud(remaining_cloud);
    pcp.getFilteredCloud(remaining_cloud);
    pcl_msgs::PolygonMesh remaining_mesh;
    pcp.trianglePointCloud(remaining_cloud, remaining_mesh);
    std::string body_name = "remaining_cloud";
    rave.addMeshCollObj(remaining_mesh, body_name);

//    double x = plane.max.x + 0.05;
//    double y = plane.max.y - 0.05;
//    double z = (plane.max.z + plane.min.z)/2;

    double x = plane.min.x+0.10;
    double y = plane.min.y-0.05;
    double z = plane.max.x+0.15;

    OpenRAVE::Transform open_door_pose(OpenRAVE::Vector(0.707, 0.707, 0.0, 0.0), OpenRAVE::Vector(x, y, z));

    EEFPoseGoals door_open_goal(1);
    door_open_goal.wrt_world = false;
    door_open_goal.no_waypoints = 10;
    door_open_goal.timesteps[0] = 9;
    door_open_goal.poses[0] = open_door_pose;

    trajectory_msgs::JointTrajectory traj_grasp = rave.computeTrajectory(door_open_goal, true);
    trajectory_msgs::JointTrajectory traj_grasp_smooth;
    rave.smoothTrajectory(traj_grasp, traj_grasp_smooth);
    controller.executeWholeBodyTraj(traj_grasp_smooth);

  }

  talk(talk_pub, shelf_inspect_sentence);
  ////////// SEGMENT SHELF //////////

  ros::Duration(2.0).sleep();
  std::vector<point_cloud_proc::Plane> shelves;
  bool shelf_segmented = pcp.segmentMultiplePlane(shelves, 1);

  if (shelf_segmented){
    // Create shelf collision objects
    int shelf_count = 1;
    int mesh_count = 1;
    std::string mesh_name = "mesh_";

    double SHELF_X = 0.28;
    double SHELF_Y = 0.78;
    double shelf_x_cloud, shelf_y_cloud;
    for (auto shelf : shelves) {
      if(shelf.orientation == shelf.ZAXIS){
        if((shelf.max.x-shelf.min.x) > 0.28){
          shelf_x_cloud = SHELF_X;
        }else{
          shelf_x_cloud = (shelf.max.x-shelf.min.x);
        }
        if((shelf.max.y-shelf.min.y) > SHELF_Y){
          shelf_y_cloud = SHELF_Y;
          std::cout << "FIXING SHELF WIDTH!!!!!" << std::endl;
        }else{
          shelf_y_cloud = (shelf.max.y-shelf.min.y);
          std::cout << "NOT FIXING THE SHELF WIDTH: " << (shelf.max.y-shelf.min.y) << std::endl;
        }

        std::string shelf_name = "rack_" + std::to_string(shelf_count);
        OpenRAVE::Vector shelf_size(shelf_x_cloud, shelf_y_cloud, 0.05);
        OpenRAVE::Transform shelf_pose(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((shelf.max.x + shelf.min.x)/2,
                                                                                   (shelf.max.y + shelf.min.y)/2,
                                                                                   (shelf.max.z + shelf.min.z)/2));

        rave.addBoxCollObj(shelf_size, shelf_pose, shelf_name);


        if(shelf_count == 1){
          std::string side_panel_name = "side_panel_1";
          OpenRAVE::Vector panel_size((shelf.max.x-shelf.min.x)+0.1, 0.05, 1.5);
          OpenRAVE::Transform panel_pose(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((shelf.max.x + shelf.min.x)/2,
                                                                                      shelf.max.y, 0.75));

          rave.addBoxCollObj(panel_size, panel_pose, side_panel_name);

          side_panel_name = "side_panel_2";
          OpenRAVE::Vector panel_size_2((shelf.max.x-shelf.min.x)+0.1, 0.05, 1.5);
          OpenRAVE::Transform panel_pose_2(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((shelf.max.x + shelf.min.x)/2,
                                                                                        shelf.min.y, 0.75));

          rave.addBoxCollObj(panel_size_2, panel_pose_2, side_panel_name);

          std::string bottom_panel_name = "bottom_panel";
          OpenRAVE::Vector panel_size_3((shelf.max.x-shelf.min.x)+0.1, (shelf.max.y-shelf.min.y), 0.05);
          OpenRAVE::Transform panel_pose_3(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((shelf.max.x + shelf.min.x)/2,
                                                                                       (shelf.max.y + shelf.min.y)/2, 0.05));

          rave.addBoxCollObj(panel_size_3, panel_pose_3, bottom_panel_name);

          std::string door_name = "door_panel";
          OpenRAVE::Vector panel_size_4((shelf.max.y-shelf.min.y)/2+0.15, 0.05, 1.5);
          OpenRAVE::Transform panel_pose_4(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((shelf.min.x-(shelf.max.y-shelf.min.y)/4),
                                                                                       shelf.min.y, 0.75));

          rave.addBoxCollObj(panel_size_4, panel_pose_4, door_name);

        }
        shelf_count++;
      }
//      else{
//
//        pcl_msgs::PolygonMesh mesh;
//        pcp.trianglePointCloud(shelf.cloud, mesh);
//        std::string body_name = mesh_name + std::to_string(mesh_count);
//        rave.addMeshCollObj(mesh, body_name);
//        mesh_count++;
//
//      }
    }
//    sensor_msgs::PointCloud2 remaining_cloud;
//    pcp.getRemainingCloud(remaining_cloud);
//    pcl_msgs::PolygonMesh remaining_mesh;
//    pcp.trianglePointCloud(remaining_cloud, remaining_mesh);
//    std::string body_name = "remaining_cloud";
//    rave.addMeshCollObj(remaining_mesh, body_name);
  }
  //////////////////////////////////////////////////

  talk(talk_pub, table_inspect_sentence);
  ////////// SEGMENT TABLE //////////
  controller.moveHeadToKnownState(TABLE_POSE);
  ros::Duration(2.0).sleep();

  // Segment table
  point_cloud_proc::Plane table;
  bool table_segmented = pcp.segmentSinglePlane(table, 'z', 2);
  if (table_segmented){
    // Creating table collision object
    std::string table_name = "table";
    OpenRAVE::Vector table_size((table.max.x-table.min.x), (table.max.y-table.min.y), table.max.z);
    OpenRAVE::Transform table_pose(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((table.max.x + table.min.x)/2,
                                                                               (table.max.y + table.min.y)/2,
                                                                               (table.max.z)/2));

    rave.addBoxCollObj(table_size, table_pose, table_name);
  }
  table_segmented = false;
  //////////////////////////////////////////////////


  ////////// PICK AND PLACE //////////
  geometry_msgs::Pose2D hsr_pose = rave.getRobotPose();
  std::vector<OpenRAVE::Transform> place_poses = rave.generatePlacePoses();
  std::cout << "TASK: # of place poses: " << place_poses.size() << std::endl;
  std::vector<point_cloud_proc::Object> tabletop_objects;
  int place_count = 0;
  while (ros::ok()){

    ///////// CLUSTER TABLETOP OBJECTS //////////
    table_segmented = pcp.segmentSinglePlane(table, 'z', 2);
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
    
    Grasp grasp = rave.generateGraspPose(TABLE_POSE);
//    if(!grasp.graspable){
//      std::cout << "TASK: clustering the table again..." << std::endl;
//      rave.removeTableObjects();
//      ros::Duration(1.0).sleep();
//      tabletop_objects.clear();
//      tabletop_clustered = false;
//      continue;
//    }

    /// PICKING ///
    EEFPoseGoals eef_grasp_goal(1);
    eef_grasp_goal.wrt_world = true;
    eef_grasp_goal.no_waypoints = 5;
    eef_grasp_goal.timesteps[0] = 4;
    eef_grasp_goal.poses[0] = grasp.pose;

    trajectory_msgs::JointTrajectory traj_grasp = rave.computeTrajectory(eef_grasp_goal, true);
    trajectory_msgs::JointTrajectory traj_grasp_smooth;
    rave.smoothTrajectory(traj_grasp, traj_grasp_smooth);
    controller.executeWholeBodyTraj(traj_grasp_smooth);

    controller.graspOrRelease(GRIPPER_STATE::GRASP);

    rave.grabObject(grasp.obj_name);

    // MIGHT NEED TO GO PICK POSE BEFORE PLACING
//    JointPosGoals pick_joint_goal;
//    pick_joint_goal.no_waypoints = 10;
//    pick_joint_goal.q = {0.0, 0.0, 0.0, 0.69, -2.60, 0.0, -0.54, 0.0};
//    trajectory_msgs::JointTrajectory traj_pick = rave.computeTrajectory(pick_joint_goal);
//    trajectory_msgs::JointTrajectory traj_pick_smooth;
//    rave.smoothTrajectory(traj_pick, traj_pick_smooth);
//    controller.executeWholeBodyTraj(traj_pick_smooth);


    /// PLACING ///
    EEFPoseGoals eef_place_goal(1);
    eef_place_goal.wrt_world = true;
    eef_place_goal.no_waypoints = 5;
    eef_place_goal.timesteps[0] = 4;
    eef_place_goal.poses[0] = place_poses[place_count];
    std::cout << "TASK: place pos-rot : " << place_poses[place_count].trans << " " << place_poses[place_count].rot << std::endl;


    trajectory_msgs::JointTrajectory traj_place = rave.computeTrajectory(eef_place_goal);
    trajectory_msgs::JointTrajectory traj_place_smooth;
    rave.smoothTrajectory(traj_place, traj_place_smooth);
    controller.executeWholeBodyTraj(traj_place_smooth);


    controller.graspOrRelease(GRIPPER_STATE::RELEASE);
    rave.releaseObject(grasp.obj_name);


    JointPosGoals pick_joint_goal;
    pick_joint_goal.no_waypoints = 10;
    pick_joint_goal.q = {hsr_pose.x, hsr_pose.y, hsr_pose.theta, 0.69, -2.60, 0.0, -0.54, 0.0};
    trajectory_msgs::JointTrajectory traj_pick = rave.computeTrajectory(pick_joint_goal);
    trajectory_msgs::JointTrajectory traj_pick_smooth;
    rave.smoothTrajectory(traj_pick, traj_pick_smooth);
    controller.executeWholeBodyTraj(traj_pick_smooth);

    rave.removeTableObjects();
    ros::Duration(2.0).sleep();
    tabletop_objects.clear();
    tabletop_clustered = false;
    place_count++;
  }

  ros::waitForShutdown();

  return 0;
}
