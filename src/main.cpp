#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>

#include <geometry_msgs/msg/point_stamped.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;
using namespace std::chrono_literals;

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_tutorial");
constexpr double TURTLE_POS_X = 0.0;
constexpr double TURTLE_POS_Y = 0.4;
constexpr double TURTLE_POS_Z = 0.0;
constexpr double TURTLE_RADIUS = 0.1;
constexpr auto VISUALIZATION_STEP_DURATION = 300ms;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  RCLCPP_INFO(LOGGER, "Initialize node");

  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("moveit_examples", "", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "manipulator";
  static const std::string LOGNAME = "moveit_examples";
  static const std::vector<std::string> CONTROLLERS(1, "arm_controller");

  RCLCPP_INFO(LOGGER, "Starting MoveIt Examples...");

  auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

  auto planning_component = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp);
  auto robot_model = moveit_cpp->getRobotModel();
  auto robot_state = moveit_cpp->getCurrentState();
  auto arm_group = robot_model->getJointModelGroup(PLANNING_GROUP);
  auto ik_link = arm_group->getOnlyOneEndEffectorTip();

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  moveit_visual_tools::MoveItVisualTools visual_tools(
      node,
      robot_model->getModelFrame(),
      "moveit_examples",
      moveit_cpp->getPlanningSceneMonitorNonConst());
  visual_tools.loadRobotStatePub();
  visual_tools.loadTrajectoryPub();

  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveItCpp_Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Publish Turtle
  geometry_msgs::msg::PoseStamped turtle_pose;
  turtle_pose.header.frame_id = "world";
  turtle_pose.pose.position.x = TURTLE_POS_X;
  turtle_pose.pose.position.y = TURTLE_POS_Y;
  turtle_pose.pose.position.z = TURTLE_POS_Z;
  visual_tools.publishSphere(turtle_pose.pose,
		             rvt::GREEN,
			     2 * TURTLE_RADIUS);
  visual_tools.trigger();

  // 1. IK on fully defined surface pose
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  moveit::core::RobotState target_state(*robot_state);
  target_state.update();

  Eigen::Isometry3d target_pose;
  tf2::fromMsg(turtle_pose.pose, target_pose);
  target_pose.translate(Eigen::Vector3d(0, 0, TURTLE_RADIUS));
  target_pose.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  for (size_t i = 0; i < 10; ++i)
  {
    if (!target_state.setFromIK(arm_group, target_pose))
    {
        RCLCPP_ERROR(LOGGER, "Failed to solve IK!");
	continue;
    }
    target_state.update();
    
    visual_tools.publishRobotState(target_state);
    visual_tools.trigger();
    rclcpp::sleep_for(VISUALIZATION_STEP_DURATION);
  }

  // 2. Pose Constraint Sampler
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  geometry_msgs::msg::Point ik_link_offset;
  ik_link_offset.z = TURTLE_RADIUS;

  geometry_msgs::msg::PointStamped target_point;
  target_point.header = turtle_pose.header;
  target_point.point = turtle_pose.pose.position;

  auto constraints = kinematic_constraints::constructGoalConstraints(ik_link->getName(), ik_link_offset, target_point, 1e-3 /* tolerance */);
  constraint_samplers::ConstraintSamplerManager sampler_manager;
  planning_scene::PlanningScenePtr scene;
  {
    auto psm = moveit_cpp->getPlanningSceneMonitorNonConst();
    psm->updateFrameTransforms();
    planning_scene_monitor::LockedPlanningSceneRO ro_scene(psm);
    scene = planning_scene::PlanningScene::clone(ro_scene);
  }

  auto goal_sampler = sampler_manager.selectSampler(scene, arm_group->getName(), constraints);

  for (size_t i = 0; i < 10; ++i)
  {
    if (!goal_sampler->sample(target_state, 10 /* attempts */))
    {
      RCLCPP_ERROR(LOGGER, "Failed to sample goal state");
      continue;
    }

    visual_tools.publishRobotState(target_state);
    visual_tools.trigger();
    rclcpp::sleep_for(VISUALIZATION_STEP_DURATION);
  }

  // 3. Cost function with IK
  visual_tools.prompt("Press 'next' to run \"Cost function with IK\"");

  auto ik_options = kinematics::KinematicsQueryOptions();
  ik_options.return_approximate_solution = true;

  // Bump up IK thresholds to allow approximate solutions
  const std::string ik_param_ns = "robot_description_kinematics." + arm_group->getName() + ".";
  node->set_parameters({rclcpp::Parameter(ik_param_ns + "position_threshold", 0.1),
		        rclcpp::Parameter(ik_param_ns + "orientation_threshold", 0.1),
			rclcpp::Parameter(ik_param_ns + "cost_threshold", 0.1)});

  kinematic_constraints::KinematicConstraintSet constraints_validator(robot_model);
  constraints_validator.add(constraints, scene->getTransforms());

  auto validity_callback = [&](moveit::core::RobotState* sample_state, const moveit::core::JointModelGroup* group,
                               const double* joint_positions)
  {
    sample_state->setJointGroupPositions(group, joint_positions);
    sample_state->updateLinkTransforms();
    return constraints_validator.decide(*sample_state).satisfied;
  };

  auto constraints_cost_fn = [&](const geometry_msgs::msg::Pose& /* target_pose */, const moveit::core::RobotState& sample_state,
                                moveit::core::JointModelGroup const* /* group */, const std::vector<double>& /* joint_positions */)
  {
    return constraints_validator.decide(sample_state).distance;
  };

  for (size_t i = 0; i < 10; ++i)
  {
    target_state.setToRandomPositions(arm_group);
    if (!target_state.setFromIK(arm_group, target_pose, 0.05, validity_callback, ik_options, constraints_cost_fn))
    {
        RCLCPP_ERROR(LOGGER, "Failed to solve IK!");
	continue;
    }
    target_state.update();
    
    visual_tools.publishRobotState(target_state);
    visual_tools.trigger();
    rclcpp::sleep_for(VISUALIZATION_STEP_DURATION);
  }

  // 4. Cost function with CartesianInterpolator
  visual_tools.prompt("Press 'next' to run \"Cost function with CartesianInterpolator\"");

  moveit::core::RobotState start_state(target_state);
  // start_state.setFromIK(arm_group, target_pose);

  node->set_parameters({rclcpp::Parameter(ik_param_ns + "mode", "local"),
			rclcpp::Parameter(ik_param_ns + "position_threshold", 0.02),
		        rclcpp::Parameter(ik_param_ns + "orientation_threshold", 0.1),
			rclcpp::Parameter(ik_param_ns + "cost_threshold", 0.05),
			rclcpp::Parameter(ik_param_ns + "minimal_displacement_weight", 0.1)});

  std::vector<std::shared_ptr<moveit::core::RobotState>> waypoints;
  auto succeeded_distance = moveit::core::CartesianInterpolator::computeCartesianPath(
      &start_state,
      arm_group,
      waypoints,
      ik_link,
      Eigen::Vector3d::UnitX(),
      false /* use local reference frame */,
      0.05 /* cm distance */,
      moveit::core::MaxEEFStep(0.005, 0.01),
      moveit::core::JumpThreshold(0.1),
      validity_callback,
      ik_options,
      constraints_cost_fn);

  RCLCPP_INFO_STREAM(LOGGER, "Planned Cartesian Path for distance " << succeeded_distance << " and waypoints " << waypoints.size());
  visual_tools.publishArrow(start_state.getGlobalLinkTransform(ik_link), rvt::GREEN, rvt::MEDIUM, 0.05 /* length */);
  if (succeeded_distance > 0.0 && !waypoints.empty())
    visual_tools.publishTrajectoryPath(waypoints, arm_group);

  visual_tools.trigger();

  // 5. Constrained Planning
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  ik_options.return_approximate_solution = false;
  node->set_parameters({rclcpp::Parameter(ik_param_ns + "mode", "global"),
			rclcpp::Parameter(ik_param_ns + "position_threshold", 0.005),
		        rclcpp::Parameter(ik_param_ns + "orientation_threshold", 0.01),
			rclcpp::Parameter(ik_param_ns + "cost_threshold", 0.01),
			rclcpp::Parameter(ik_param_ns + "minimal_displacement_weight", 0.1)});

  start_state.setToRandomPositions(arm_group);
  start_state.setFromIK(arm_group, target_pose, 0.05, validity_callback, ik_options, constraints_cost_fn);

  ik_options.return_approximate_solution = true;
  node->set_parameters({rclcpp::Parameter(ik_param_ns + "mode", "global"),
			rclcpp::Parameter(ik_param_ns + "position_threshold", 0.1),
		        rclcpp::Parameter(ik_param_ns + "orientation_threshold", 0.1),
			rclcpp::Parameter(ik_param_ns + "cost_threshold", 0.1),
			rclcpp::Parameter(ik_param_ns + "minimal_displacement_weight", 0.1)});
  auto goal_pose = start_state.getGlobalLinkTransform(ik_link) * Eigen::Translation3d(0.1, 0, 0);
  target_state.setToRandomPositions(arm_group);
  target_state.setFromIK(arm_group, goal_pose, 0.05, validity_callback, ik_options, constraints_cost_fn);

  moveit_cpp::PlanningComponent::PlanRequestParameters plan_request_params;
  plan_request_params.load(node);
  plan_request_params.planning_pipeline = "ompl";
  plan_request_params.planner_id = "RRTConnectkConfigDefault";
  plan_request_params.planning_time = 10.0;
  planning_component->setStartState(start_state);
  planning_component->setGoal(target_state);
  planning_component->setPathConstraints(constraints);
  const auto ompl_result = planning_component->plan();
  if (ompl_result)
  {
    visual_tools.publishTrajectoryPath(ompl_result.trajectory, arm_group);
    visual_tools.trigger();
  }

  // 6. Plan with STOMP
  visual_tools.prompt("Press 'next' to \"Plan with STOMP \"");

  plan_request_params.planning_pipeline = "stomp";
  plan_request_params.planner_id = "STOMP";
  plan_request_params.planning_time = 10.0;
  const auto stomp_result = planning_component->plan(plan_request_params);

  if (stomp_result)
  {
    visual_tools.publishTrajectoryPath(stomp_result.trajectory, arm_group);
    visual_tools.trigger();
  }

  visual_tools.prompt("Press 'next' to shutdown node");

  RCLCPP_INFO(LOGGER, "Shutting down.");
  rclcpp::shutdown();
  return 0;
}
