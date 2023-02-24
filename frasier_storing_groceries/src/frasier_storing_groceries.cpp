#include <ros/package.h>
#include <point_cloud_proc/point_cloud_proc.h>
#include <frasier_openrave/frasier_motion.h>
#include <darknet_ros/GetDetections.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tmc_msgs/Voice.h>
#include <yaml-cpp/yaml.h>

using namespace frasier;

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

  ros::ServiceClient detect_objects_cli = nh.serviceClient<darknet_ros::GetDetections>("/frasier/darknet_ros/detect");
  detect_objects_cli.waitForExistence();
  ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("object_center", 10);
  ros::Publisher talk_pub = nh.advertise<tmc_msgs::Voice>("/talk_request", 10);
  ros::Subscriber wrist_sensor_sub_ = nh.subscribe("/hsrb/wrist_wrench/compensated", 1, wristSensorCb);

  bool OPEN_DOOR = false;
  bool RUN_VIEWER = false;

  HEAD_STATE TABLE_POSE = HEAD_STATE_TABLE_LEFT;

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
    }
  }


  std::string pkg_path = ros::package::getPath("frasier_storing_groceries");
  std::string config_path = pkg_path + "/config/categories.yaml";

  YAML::Node categories = YAML::LoadFile(config_path);

  FRASIERMotion motion(nh);
  PointCloudProc pcp(nh, true);
  spinner.start();

  motion.loadHSR();
  // no equivalent command for rave.startThreads() ??

  ros::Duration(1.0).sleep();

  std::string start_sentence = "Please touch my hand to start the task.";
  std::string door_open_sentence = "I am starting the task. Please open the door.";
  std::string shelf_inspect_sentence = "I am inspecting the shelf.";
  std::string table_inspect_sentence = "I am inspecting the table";
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

  motion.moveArmToKnownState(motion.ARM_STATE_GRASP);
  motion.grasp(false));
  motion.moveHeadToKnownState(motion.HEAD_STATE_SHELF_FRONT;

  talk(talk_pub, door_open_sentence);
  ros::Duration(7.0).sleep();

  talk(talk_pub, shelf_inspect_sentence);

  ////////// SEGMENT SHELF //////////

  ros::Duration(2.0).sleep();
  std::vector<point_cloud_proc::Plane> shelves;
  bool shelf_segmented = pcp.segmentMultiplePlane(shelves, 1);
  if (shelf_segmented){
    // Create shelf collision objects
    int shelf_count = 1;
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
        
        Eigen::Vector3d shelf_size((shelf_x_cloud, shelf_y_cloud, 0.05);
        Eigen::Vector3d shelf_pose(Eigen::Vector3d(1,0,0,0), Eigen::Vector3d((shelf.max.x + shelf.min.x)/2,
                                                                             (shelf.max.y + shelf.min.y)/2,
                                                                             (shelf.max.z + shelf.min.z)/2)));
        motion.addBoxCollObj(shelf_size, shelf_pose, shelf_name);
        
        if(shelf_count == 1){
          std::string side_panel_name = "side_panel_1";
          
          Eigen::Vector3d panel_size((shelf.max.x-shelf.min.x)+0.1, 0.05, 1.5);
          Eigen::Vector3d panel_pose(Eigen::Vector3d(1,0,0,0), Eigen::Vector3d((shelf.max.x + shelf.min.x)/2,
                                                                                shelf.max.y,
                                                                                0.75));
          motion.addBoxCollObj(panel_size, panel_pose, side_panel_name);
        
          side_panel_name = "side_panel_2";
          
          Eigen::Vector3d panel_size_2((shelf.max.x-shelf.min.x)+0.1, 0.05, 1.5);
          Eigen::Vector3d panel_pose_2(Eigen::Vector3d(1,0,0,0), Eigen::Vector3d((shelf.max.x + shelf.min.x)/2,
                                                                                  shelf.min.y,
                                                                                  0.75));
          motion.addBoxCollObj(panel_size_2, panel_pose_2, side_panel_name);
        
          std::string bottom_panel_name = "bottom_panel";
         
          Eigen::Vector3d panel_size_3((shelf.max.x-shelf.min.x)+0.1, (shelf.max.y-shelf.min.y), 0.05);
          Eigen::Vector3d panel_pose_3(Eigen::Vector3d(1,0,0,0), Eigen::Vector3d((shelf.max.x + shelf.min.x)/2,
                                                                                 (shelf.max.y + shelf.min.y)/2,
                                                                                  0.05));
          motion.addBoxCollObj(panel_size_3, panel_pose_3, bottom_panel_name);
        
          std::string door_name = "door_panel";
       
          Eigen::Vector3d panel_size_4((shelf.max.y-shelf.min.y)/2+0.15, 0.05, 1.5);
          Eigen::Vector3d panel_pose_4(Eigen::Vector3d(1,0,0,0), Eigen::Vector3d((shelf.min.x-(shelf.max.y-shelf.min.y)/4),
                                                                                  shelf.min.y,
                                                                                  0.75));
          motion.addBoxCollObj(panel_size_4, panel_pose_4, door_name);
        }
        shelf_count++;
      }
    }
  }
  //////////////////////////////////////////////////

  ////////// DETECT SHELF OBJECTS //////////
  darknet_ros::GetDetections inspect_shelf_srv;
  detect_objects_cli.call(inspect_shelf_srv);
  std::vector<std::string> shelf_object_names;

  for (auto object : inspect_shelf_srv.response.objects) {

    point_cloud_proc::Object object_shape;
    if(!pcp.getObjectFromBBox(object.bbox.data(), object_shape)){
      continue;
    }

    shelf_object_names.push_back(object.label);
    std::string obj_name = "shelf_obj_" + object.label;
   
    Eigen::Vector3d obj_size((object_shape.max.x-object_shape.min.x),
                             (object_shape.max.y-object_shape.min.y),
                             (object_shape.max.z-object_shape.min.z));
    Eigen::Vector3d obj_pose(Eigen::Vector3d(1,0,0,0), Eigen::Vector3d((object_shape.max.x + object_shape.min.x)/2,
                                                                       (object_shape.max.y + object_shape.min.y)/2,
                                                                       (object_shape.max.z + object_shape.min.z)/2));
    motion.addBoxCollObj(obj_size, obj_pose, obj_name);  
  }
  //////////////////////////////////////////////////

  ////////// SEGMENT TABLE //////////
  
  motion.moveHeadToKnownState(TABLE_POSE);

  ros::Duration(2.0).sleep();

  // Segment table
  point_cloud_proc::Plane table;
  bool table_segmented = pcp.segmentSinglePlane(table, 'z', 2);
  if (table_segmented){
    // Creating table collision object
    std::string table_name = "table";
    
    Eigen::Vector3d table_size((table.max.x-table.min.x), (table.max.y-table.min.y), table.max.z);
    Eigen::Vector3d table_pose(Eigen::Vector3d(1,0,0,0), Eigen::Vector3d((table.max.x + table.min.x)/2,
                                                                         (table.max.y + table.min.y)/2,
                                                                         (table.max.z)/2));
    motion.addBoxCollObj(table_size, table_pose, table_name);
  }
  table_segmented = false;
  //////////////////////////////////////////////////


  ////////// PICK AND PLACE //////////
  
  //geometry_msgs::Pose2D hsr_pose = motion.getRobotPose(); // is the function implemented ?
  //Eigen::Vector3d place_poses = // is it still necessary ?
  // TODO : verify that place_poses has member size()
  //std::cout << "TASK: # of place poses: " << place_poses.size() << std::endl;

  std::vector<point_cloud_proc::Object> tabletop_objects;
  int place_count = 0;
  while (ros::ok()){

    ////////// CLUSTER TABLETOP OBJECTS //////////
    table_segmented = pcp.segmentSinglePlane(table, 'z', 2);
    bool tabletop_clustered = pcp.clusterObjects(tabletop_objects);
    std::cout << "TASK: number of objects: " << tabletop_objects.size() << std::endl;
    int object_count = 0;
    if (tabletop_clustered){
      // Create tabletop collision objects

      for (auto object : tabletop_objects) {
        std::string obj_name = "table_obj_" + std::to_string(object_count);
        
        Eigen::Vector3d obj_size((object.max.x-object.min.x), (object.max.y-object.min.y), (object.max.z-object.min.z));
        Eigen::Vector3d obj_pose(Eigen::Vector3d(1,0,0,0), Eigen::Vector3d((object.max.x + object.min.x)/2,
                                                                           (object.max.y + object.min.y)/2,
                                                                           (object.max.z + object.min.z)/2));
        motion.addBoxCollObj(obj_size, obj_pose, obj_name);
    
        object_count++;
      }
    }
    else{
      std::cout << "TASK: there is no object left on the table!!!" << std::endl;
      ros::shutdown();
      return 0;
    }

    ////////// DETECT TABLE OBJECTS //////////
    darknet_ros::GetDetections inspect_table_srv;
    detect_objects_cli.call(inspect_table_srv);
    
    geometry_msgs::Pose place_pose; // Is it the right type ?
    geometry_msgs::Pose grasp_pose; 

    std::string object_grasp, object_place;
    bool same_category_found = false;
    for (auto object : inspect_table_srv.response.objects) {

      for(auto const shelf_object_name: shelf_object_names) {
        if (categories[shelf_object_name].as<std::string>() == categories[object.label].as<std::string>()){
          object_grasp = object.label;
          object_place = shelf_object_name;
          same_category_found = true;
          break;
        }
      }

      if(same_category_found){
        point_cloud_proc::Object object_shape;
        if(!pcp.getObjectFromBBox(object.bbox.data(), object_shape)){
          continue;
        }

        std::string obj_name = "table_obj_" + object.label;
        
        Eigen::Vector3d obj_size((object_shape.max.x-object_shape.min.x), 
                                 (object_shape.max.y-object_shape.min.y),
                                 (object_shape.max.z-object_shape.min.z));
        Eigen::Vector3d obj_pose(Eigen::Vector3d(1,0,0,0), Eigen::Vector3d((object_shape.max.x + object_shape.min.x)/2,
                                                                           (object_shape.max.y + object_shape.min.y)/2,
                                                                           (object_shape.max.z + object_shape.min.z)/2));
        motion.addBoxCollObj(obj_size, obj_pose, obj_name);
    
        std::cout << "STORING_GROCERIES: FOUND SAME CATEGORY!!!"
        << " table: : " << object_grasp <<  " shelf: " << object_place << std::endl;
        std::string same_category_sentence = "I found the " + object.label + " on the table which belongs to "
                                           + categories[object.label].as<std::string>() + " category.";
        talk(talk_pub, same_category_sentence);

        object_grasp = "table_obj_" + object_grasp;
        grasp_pose = motion.generateGraspPose(object_grasp);

        object_place = "shelf_obj_" + object_place;
        place_pose = motion.generatePlacePose(object_place);
        
        break;
      }
    }


    if(!same_category_found){
      std::cout << "STORING_GROCERIES: NOT FOUND SAME CATEGORY!!! : "  << std::endl;
      grasp_pose = motion.generateGraspPoses(TABLE_POSE); //to be changed
//  if(!grasp.graspable){
//        break;
//      }
      std::string not_same_category = "I couldn't find the same category. I am going to create a new space.";
      talk(talk_pub, not_same_category);
      place_pose = place_poses[place_count];
      place_count++;
    }


    /// PICKING ///
   
    FRASIERMotion::EEFPoseGoals eef_grasp_goal;
    Eigen::Isometry3d grasp_pose_eigen;
    motion.toEigenPose(grasp_pose.pose, grasp_pose_eigen);
    eef_grasp_goal.goal_pose = grasp_pose_eigen;
    
    motion.planAndExecuteTrajectory(eef_grasp_goal);
    
    motion.grasp(true);

    /// PLACING ///
    
    FRASIERMotion::EEFPoseGoals eef_place_goal;
    Eigen::Isometry3d place_pose_eigen;
    motion.toEigenPose(place_pose.pose, place_pose_eigen);
    eef_place_goal.goal_pose = place_pose_eigen;

    motion.planAndExecuteTrajectory(eef_place_goal);
    
    motion.grasp(false);

    // HSR comes back in front of the table
    motion.moveBase(motion.BASE_STATE_HOME); // may want to change that (use initial HSR pose #getRobotPose() ?)

    
    motion.removeTableObjects();

    ros::Duration(2.0).sleep();
    tabletop_objects.clear();
    tabletop_clustered = false;
  }

  ros::waitForShutdown();

  return 0;
}