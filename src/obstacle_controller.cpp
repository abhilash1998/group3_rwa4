#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_controller");

  // decide whether to publish the full scene
  bool full_scene = false;

  // the index of the argument with the filename
  int filename_index = 1;
  if (argc > 2)
    if (strncmp(argv[1], "--scene", 7) == 0)
    {
      full_scene = true;
      filename_index = 2;
    }
ROS_INFO("Wait");
  if (argc > 1)
  {
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::NodeHandle nh;
    ros::Publisher pub_scene;
    ROS_INFO("Wait");
    if (full_scene)
      pub_scene = nh.advertise<moveit_msgs::PlanningScene>("/ariac/gantry/planning_scene_world", 1);
    else
      pub_scene = nh.advertise<moveit_msgs::PlanningScene>("/ariac/gantry/monitored_planning_scene", 1);
    
    // pub_scene = nh.advertise<moveit_msgs::PlanningSceneWorld>("/ariac/gantry/move_group/monitored_planning_scene", 1);
    // /ariac/gantry/move_group/planning_scene_monitor/parameter_descriptions   /ariac/gantry/move_group/planning_scene_monitor/parameter_updates
    // /ariac/gantry/move_group/sense_for_plan/parameter_descriptions /ariac/gantry/move_group/sense_for_plan/parameter_updates /ariac/gantry/move_group/monitored_planning_scene
    
    robot_model_loader::RobotModelLoader::Options opt;
    opt.robot_description_ = "/ariac/gantry/robot_description";
    opt.load_kinematics_solvers_ = false;
    robot_model_loader::RobotModelLoaderPtr rml(new robot_model_loader::RobotModelLoader(opt));
    planning_scene::PlanningScene ps(rml->getModel());

    std::ifstream f(argv[filename_index]);
    if (ps.loadGeometryFromStream(f))
    {
      ROS_INFO("Publishing geometry from '%s' ...", argv[filename_index]);
      moveit_msgs::PlanningScene ps_msg;
      ps.getPlanningSceneMsg(ps_msg);
      ps_msg.is_diff = true;

      ros::WallDuration dt(0.5);
      unsigned int attempts = 0;
      while (pub_scene.getNumSubscribers() < 1 && ++attempts < 100)
        dt.sleep();

      if (full_scene)
        pub_scene.publish(ps_msg);
      else
        pub_scene.publish(ps_msg.world);

      ros::Duration(1).sleep();
    }
  }
  else
    ROS_WARN("A filename was expected as argument. That file should be a text representation of the geometry in a "
             "planning scene.");

  ros::shutdown();
  return 0;
}