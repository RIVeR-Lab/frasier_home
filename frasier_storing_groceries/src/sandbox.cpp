//double MAX_RACK_X = 0.30;
//double MAX_RACK_Y = 0.80;
//double RACK_Z = 0.05;
//double RACK_X, RACK_Y;
//std::vector<point_cloud_proc::Plane> shelves;
//bool shelf_segmented = pcp.segmentMultiplePlane(shelves);
//
//if (shelf_segmented){
//// Create shelf collision objects
//int shelf_count = 1;
//std::string mesh_name = "mesh_";
//for (auto shelf : shelves) {
//if(shelf.orientation == shelf.ZAXIS){
//
//if ((shelf.max.x-shelf.min.x) > MAX_RACK_X){
//RACK_X = MAX_RACK_X;
//} else{
//RACK_X = (shelf.max.x-shelf.min.x);
//}
//
//if ((shelf.max.y-shelf.min.y) > MAX_RACK_Y){
//RACK_Y = MAX_RACK_Y;
//} else{
//RACK_Y = (shelf.max.y-shelf.min.y);
//}




////////// DETECT TABLE OBJECTS //////////
//  darknet_ros::GetDetections inspect_table_srv;
//  detect_objects_cli.call(inspect_table_srv);
//  std::vector<std::string> table_object_names;
//
//  for (auto object : inspect_table_srv.response.objects) {
//
//    point_cloud_proc::Object object_shape;
//    if(!pcp.getObjectFromBBox(object.bbox.data(), object_shape)){
//      continue;
//    }
//    table_object_names.push_back(object.label);
//    std::string obj_name = "table_obj_" + object.label;
//    OpenRAVE::Vector obj_size((object_shape.max.x-object_shape.min.x),
//                              (object_shape.max.y-object_shape.min.y),
//                              (object_shape.max.z-object_shape.min.z));
//    OpenRAVE::Transform obj_pose(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((object_shape.max.x + object_shape.min.x)/2,
//                                                                             (object_shape.max.y + object_shape.min.y)/2,
//                                                                             (object_shape.max.z + object_shape.min.z)/2));
//
//    rave.addBoxCollObj(obj_size, obj_pose, obj_name);
//
//  }
//////////////////////////////////////////////////

// Cluster tabletop objects
//  std::vector<point_cloud_proc::Object> tabletop_objects;
//  bool tabletop_clustered = pcp.clusterObjects(tabletop_objects);
//  if (tabletop_clustered){
//    // Create tabletop collision objects
//    for (int i = 0; i <tabletop_objects.size() ; i++) {
//      point_cloud_proc::Object object = tabletop_objects[i];
//      std::string obj_name = "obj_" + std::to_string(i+1);
//      OpenRAVE::Vector obj_size((object.max.x-object.min.x), (object.max.y-object.min.y), (object.max.z-object.min.z));
//      OpenRAVE::Transform obj_pose(OpenRAVE::Vector(1,0,0,0), OpenRAVE::Vector((object.max.x + object.min.x)/2, (object.max.y + object.min.y)/2, (object.max.z + object.min.z)/2));
//
//      rave.addBoxCollObj(obj_size, obj_pose, obj_name);
//    }
//
//  }

//    std::string object_grasp, object_place;
//    bool same_category_found = false;
//    for(auto const table_object_name : table_object_names){
//      for(auto const shelf_object_name: shelf_object_names){
//        if (categories[shelf_object_name].as<std::string>() == categories[table_object_name].as<std::string>()){
//          object_grasp = table_object_name;
//          object_place = shelf_object_name;
//          same_category_found = true;
//          break;
//        }
//      }
//      if(same_category_found){
//        std::cout << "STORING_GROCERIES: found same category..." << std::endl;
//        break;
//      }
//    }


std::vector<OpenRAVE::Transform> place_poses = rave.generatePlacePoses();

//  tmc_msgs::Voice enter_object_name;
//  enter_object_name.language = enter_object_name.kEnglish;
//  enter_object_name.sentence = "Please enter the name of the object from the table.";
//
//  talk_pub.publish(enter_object_name);
//
//  // Recognise tabletop objects
//
//  Grasp grasp;
//  bool is_found_object = false;
//  while(ros::ok()){
//    std::string object_name;
//    std::cout << "Enter the name of the object: ";
//    std::getline(std::cin, object_name);
//    darknet_ros::GetDetections detection_srv;
//    detect_objects_cli.call(detection_srv);
//
//    for (int i = 0; i < detection_srv.response.objects.size(); i++) {
//      int cx = detection_srv.response.objects[i].center[0];
//      int cy = detection_srv.response.objects[i].center[1];
//
//      geometry_msgs::PointStamped obj_center;
//      if (detection_srv.response.objects[i].label == object_name){
//        pcp.get3DPoint(cx, cy, obj_center);
//        grasp.pose.trans.x = obj_center.point.x;
//        grasp.pose.trans.y = obj_center.point.y - 0.03;
//        grasp.pose.trans.z = obj_center.point.z;
//        grasp.pose.rot = OpenRAVE::Vector(0.5, -0.5, -0.5, -0.5);
//        grasp.obj_name = detection_srv.response.objects[i].label;
//        tmc_msgs::Voice object_found;
//        object_found.language = object_found.kEnglish;
//        object_found.sentence = "I found " + object_name +" on the table.";
//        is_found_object = true;
//        talk_pub.publish(object_found);
//        break;
//      }
//
//    }
//    if (is_found_object){
//      break;
//    }else{
//      tmc_msgs::Voice object_not_found;
//      object_not_found.language = object_not_found.kEnglish;
//      object_not_found.sentence = "I couldn't find the object you asked. Please try again";
//      talk_pub.publish(object_not_found);
//    }
//  ros::Duration(0.5).sleep();
//  }
//
//  tmc_msgs::Voice object_picking;
//  object_picking.language = object_picking.kEnglish;
//  object_picking.sentence = "I am picking up the " + grasp.obj_name;
//
//  talk_pub.publish(object_picking);
//
//
//  /// PICKING ///
//
////  Grasp grasp = rave.generateGraspPose();
////  rave.drawTransform(grasp_pose);
//
//  EEFPoseGoals eef_grasp_goal(1);
//  eef_grasp_goal.wrt_world = false;
//  eef_grasp_goal.no_waypoints = 5;
//  eef_grasp_goal.timesteps[0] = 4;
//  eef_grasp_goal.poses[0] = grasp.pose;
//
//  trajectory_msgs::JointTrajectory traj_grasp = rave.computeTrajectory(eef_grasp_goal);
//  trajectory_msgs::JointTrajectory traj_grasp_smooth;
//  rave.smoothTrajectory(traj_grasp, traj_grasp_smooth);
//  controller.executeWholeBodyTraj(traj_grasp_smooth);
//
//  controller.graspOrRelease(GRIPPER_STATE::GRASP);
//
////  rave.grabObject(grasp.obj_name);
//
//  /// PLACING ///
//
//  if (!place_poses.empty()){
//    EEFPoseGoals eef_place_goal(1);
//    eef_place_goal.wrt_world = true;
//    eef_place_goal.no_waypoints = 5;
//    eef_place_goal.timesteps[0] = 4;
//    eef_place_goal.poses[0] = place_poses[0];
//
//
//    trajectory_msgs::JointTrajectory traj_place = rave.computeTrajectory(eef_place_goal, true);
//    trajectory_msgs::JointTrajectory traj_place_smooth;
//    rave.smoothTrajectory(traj_place, traj_place_smooth);
//    controller.executeWholeBodyTraj(traj_place_smooth);
//  }
//
//  controller.graspOrRelease(GRIPPER_STATE::RELEASE);
////  rave.releaseObject(grasp.obj_name);
//
//  JointPosGoals pick_joint_goal;
//  pick_joint_goal.no_waypoints = 10;
//  pick_joint_goal.q = {0.0, 0.0, 0.0, 0.69, -2.60, 0.0, -0.54, 0.0, 0.0};
//  trajectory_msgs::JointTrajectory traj_pick = rave.computeTrajectory(pick_joint_goal);
//  trajectory_msgs::JointTrajectory traj_pick_smooth;
//  rave.smoothTrajectory(traj_pick, traj_pick_smooth);
//  controller.executeWholeBodyTraj(traj_pick_smooth);
