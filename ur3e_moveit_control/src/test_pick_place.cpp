#include <cmath>
#include <thread>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/constraints.hpp>

struct TestPosition {
  std::string name;
  double x, y, z;
  double qw, qx, qy, qz;
};

// 5 pick-and-place positions in base_link frame.
// Orientation: x=-0.707, y=0.707, z=0.0, w=0.0 (consistent with previously tested poses).
const std::vector<TestPosition> TEST_POSITIONS = {
  {"pos1_center_y",   0.00, 0.35, 0.10,  0.0, -0.707, 0.707, 0.0},
  {"pos2_front_left", 0.25, 0.30, 0.10,  0.0, -0.707, 0.707, 0.0},
  {"pos3_front_mid",  0.30, 0.25, 0.10,  0.0, -0.707, 0.707, 0.0},
  {"pos4_near_left",  0.15, 0.25, 0.10,  0.0, -0.707, 0.707, 0.0},
  {"pos5_mid_left",   0.20, 0.30, 0.10,  0.0, -0.707, 0.707, 0.0},
};

static void unwrap_wrist3(moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  auto & traj = plan.trajectory_.joint_trajectory;
  int idx = -1;
  for (size_t j = 0; j < traj.joint_names.size(); ++j) {
    if (traj.joint_names[j].find("wrist_3") != std::string::npos) {
      idx = static_cast<int>(j);
      break;
    }
  }
  if (idx < 0) return;
  for (size_t pt = 1; pt < traj.points.size(); ++pt) {
    double prev = traj.points[pt - 1].positions[idx];
    double & curr = traj.points[pt].positions[idx];
    while (curr - prev >  M_PI) curr -= 2 * M_PI;
    while (curr - prev < -M_PI) curr += 2 * M_PI;
  }
}

static bool go_home(moveit::planning_interface::MoveGroupInterface & mg, rclcpp::Logger log)
{
  mg.setStartStateToCurrentState();
  mg.setNamedTarget("home");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = false;
  for (int attempt = 1; attempt <= 10 && !success; ++attempt) {
    success = (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  }
  if (!success) {
    RCLCPP_ERROR(log, "Failed to plan home trajectory");
    return false;
  }
  unwrap_wrist3(plan);
  return mg.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

static bool go_to_pose(
  moveit::planning_interface::MoveGroupInterface & mg,
  const TestPosition & pos,
  rclcpp::Logger log)
{
  geometry_msgs::msg::Pose target;
  target.position.x = pos.x;
  target.position.y = pos.y;
  target.position.z = pos.z;
  target.orientation.w = pos.qw;
  target.orientation.x = pos.qx;
  target.orientation.y = pos.qy;
  target.orientation.z = pos.qz;

  // Constrain wrist_3 to [-π, π] so IK never picks a full-rotation-away solution
  for (const auto & name : mg.getJointNames()) {
    if (name.find("wrist_3") != std::string::npos) {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = name;
      jc.position = 0.0;
      jc.tolerance_above = 2 * M_PI;
      jc.tolerance_below = 2 * M_PI;
      jc.weight = 1.0;
      moveit_msgs::msg::Constraints constraints;
      constraints.joint_constraints.push_back(jc);
      mg.setPathConstraints(constraints);
      break;
    }
  }

  mg.setStartStateToCurrentState();
  mg.setPoseTarget(target, mg.getEndEffectorLink());

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = false;
  for (int attempt = 1; attempt <= 10 && !success; ++attempt) {
    success = (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
      RCLCPP_WARN(log, "  Planning attempt %d/10 failed, retrying...", attempt);
  }
  if (!success) {
    RCLCPP_ERROR(log, "  Planning failed after 10 attempts");
    return false;
  }
  unwrap_wrist3(plan);
  bool exec_ok = mg.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  if (!exec_ok)
    RCLCPP_ERROR(log, "  Execution failed");
  return exec_ok;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "pick_place_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  // Spin in a background thread so MoveGroupInterface can receive joint states
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  auto mg = moveit::planning_interface::MoveGroupInterface(node, "ur_onrobot_manipulator");
  mg.setPlannerId("RRTConnectkConfigDefault");
  mg.setPlanningTime(2.0);
  mg.setNumPlanningAttempts(1);
  mg.setMaxVelocityScalingFactor(0.1);
  mg.setMaxAccelerationScalingFactor(0.1);
  mg.setGoalPositionTolerance(0.005);
  mg.setGoalOrientationTolerance(0.05);
  mg.setWorkspace(-0.6, -0.6, -0.1, 0.6, 0.6, 0.9);

  RCLCPP_INFO(logger, "=== Pick and Place Position Test ===");
  RCLCPP_INFO(logger, "Testing %zu positions\n", TEST_POSITIONS.size());

  struct Result { std::string name; bool home_ok; bool pose_ok; };
  std::vector<Result> results;

  for (const auto & pos : TEST_POSITIONS) {
    RCLCPP_INFO(logger, "--- [%s] (%.2f, %.2f, %.2f) ---",
      pos.name.c_str(), pos.x, pos.y, pos.z);

    RCLCPP_INFO(logger, "  Moving to home...");
    bool home_ok = go_home(mg, logger);
    RCLCPP_INFO(logger, "  Home: %s", home_ok ? "PASS" : "FAIL");

    bool pose_ok = false;
    if (home_ok) {
      RCLCPP_INFO(logger, "  Moving to position...");
      pose_ok = go_to_pose(mg, pos, logger);
      RCLCPP_INFO(logger, "  Position: %s", pose_ok ? "PASS" : "FAIL");
    } else {
      RCLCPP_WARN(logger, "  Skipping position (home failed)");
    }

    results.push_back({pos.name, home_ok, pose_ok});
  }

  // Summary
  RCLCPP_INFO(logger, "\n=== RESULTS ===");
  RCLCPP_INFO(logger, "%-20s  %-6s  %-8s", "Position", "Home", "Pose");
  RCLCPP_INFO(logger, "%-20s  %-6s  %-8s", "--------", "----", "----");
  int passed = 0;
  for (const auto & r : results) {
    bool ok = r.home_ok && r.pose_ok;
    if (ok) passed++;
    RCLCPP_INFO(logger, "%-20s  %-6s  %-8s",
      r.name.c_str(),
      r.home_ok ? "PASS" : "FAIL",
      r.pose_ok ? "PASS" : "FAIL");
  }
  RCLCPP_INFO(logger, "\n%d / %zu positions PASSED", passed, results.size());

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return passed == static_cast<int>(results.size()) ? 0 : 1;
}
