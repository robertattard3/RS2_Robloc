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

const std::vector<TestPosition> TEST_POSITIONS = {
  {"pos1_center_y",   0.00, 0.35, 0.10,  0.0, -0.707, 0.707, 0.0},
  {"pos2_front_left", 0.25, 0.30, 0.10,  0.0, -0.707, 0.707, 0.0},
  {"pos3_front_mid",  0.30, 0.25, 0.10,  0.0, -0.707, 0.707, 0.0},
  {"pos4_near_left",  0.15, 0.25, 0.10,  0.0, -0.707, 0.707, 0.0},
  {"pos5_mid_left",   0.20, 0.30, 0.10,  0.0, -0.707, 0.707, 0.0},
};

const double PASS_THRESHOLD_M = 0.025;  // 25 mm

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

static void set_wrist3_constraint(moveit::planning_interface::MoveGroupInterface & mg)
{
  for (const auto & name : mg.getJointNames()) {
    if (name.find("wrist_3") != std::string::npos) {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = name;
      jc.position = 0.0;
      jc.tolerance_above = 2 * M_PI;
      jc.tolerance_below = 2 * M_PI;
      jc.weight = 1.0;
      moveit_msgs::msg::Constraints c;
      c.joint_constraints.push_back(jc);
      mg.setPathConstraints(c);
      break;
    }
  }
}

static bool go_home(moveit::planning_interface::MoveGroupInterface & mg, rclcpp::Logger log)
{
  mg.setStartStateToCurrentState();
  mg.setNamedTarget("home");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = false;
  for (int attempt = 1; attempt <= 10 && !success; ++attempt)
    success = (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) { RCLCPP_ERROR(log, "  Failed to plan to home"); return false; }
  unwrap_wrist3(plan);
  return mg.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "accuracy_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto log = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  auto mg = moveit::planning_interface::MoveGroupInterface(node, "ur_onrobot_manipulator");
  mg.setPlannerId("RRTConnectkConfigDefault");
  mg.setPlanningTime(2.0);
  mg.setMaxVelocityScalingFactor(0.1);
  mg.setMaxAccelerationScalingFactor(0.1);
  mg.setGoalPositionTolerance(0.005);
  mg.setGoalOrientationTolerance(0.05);
  mg.setWorkspace(-0.6, -0.6, -0.1, 0.6, 0.6, 0.9);

  RCLCPP_INFO(log, "=== End-Effector Accuracy Test ===");
  RCLCPP_INFO(log, "Pass threshold: %.0f mm\n", PASS_THRESHOLD_M * 1000.0);

  struct Result {
    std::string name;
    bool reached;
    double error_mm;
    double tx, ty, tz;   // target
    double ax, ay, az;   // actual
  };
  std::vector<Result> results;

  for (const auto & pos : TEST_POSITIONS) {
    RCLCPP_INFO(log, "--- [%s] target (%.3f, %.3f, %.3f) ---",
      pos.name.c_str(), pos.x, pos.y, pos.z);

    RCLCPP_INFO(log, "  Moving to home...");
    if (!go_home(mg, log)) {
      RCLCPP_ERROR(log, "  Home failed — skipping");
      results.push_back({pos.name, false, -1.0, pos.x, pos.y, pos.z, 0, 0, 0});
      continue;
    }

    // Set wrist_3 constraint based on current (home) position
    set_wrist3_constraint(mg);

    // Plan to target
    geometry_msgs::msg::Pose target;
    target.position.x    = pos.x;
    target.position.y    = pos.y;
    target.position.z    = pos.z;
    target.orientation.w = pos.qw;
    target.orientation.x = pos.qx;
    target.orientation.y = pos.qy;
    target.orientation.z = pos.qz;

    mg.setStartStateToCurrentState();
    mg.setPoseTarget(target, mg.getEndEffectorLink());

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= 10 && !planned; ++attempt) {
      planned = (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!planned)
        RCLCPP_WARN(log, "  Planning attempt %d/10 failed, retrying...", attempt);
    }

    if (!planned) {
      RCLCPP_ERROR(log, "  Planning failed after 10 attempts");
      results.push_back({pos.name, false, -1.0, pos.x, pos.y, pos.z, 0, 0, 0});
      continue;
    }

    unwrap_wrist3(plan);
    bool exec_ok = mg.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;

    if (!exec_ok) {
      RCLCPP_ERROR(log, "  Execution failed");
      results.push_back({pos.name, false, -1.0, pos.x, pos.y, pos.z, 0, 0, 0});
      continue;
    }

    // Read actual end-effector pose
    auto actual = mg.getCurrentPose(mg.getEndEffectorLink()).pose;
    double ax = actual.position.x;
    double ay = actual.position.y;
    double az = actual.position.z;

    double error = std::sqrt(
      std::pow(ax - pos.x, 2) +
      std::pow(ay - pos.y, 2) +
      std::pow(az - pos.z, 2));

    bool pass = error <= PASS_THRESHOLD_M;
    RCLCPP_INFO(log, "  Actual:  (%.4f, %.4f, %.4f)", ax, ay, az);
    RCLCPP_INFO(log, "  Error:   %.2f mm  [%s]", error * 1000.0, pass ? "PASS" : "FAIL");

    results.push_back({pos.name, pass, error * 1000.0, pos.x, pos.y, pos.z, ax, ay, az});
  }

  // Summary table
  RCLCPP_INFO(log, "\n=== RESULTS ===");
  RCLCPP_INFO(log, "%-20s  %-20s  %-20s  %-10s  %-6s",
    "Position", "Target (x,y,z)", "Actual (x,y,z)", "Error(mm)", "Result");
  RCLCPP_INFO(log, "%-20s  %-20s  %-20s  %-10s  %-6s",
    "--------", "--------------", "--------------", "---------", "------");

  int passed = 0;
  for (const auto & r : results) {
    char target_buf[32], actual_buf[32];
    snprintf(target_buf, sizeof(target_buf), "(%.2f,%.2f,%.2f)", r.tx, r.ty, r.tz);
    snprintf(actual_buf, sizeof(actual_buf), "(%.2f,%.2f,%.2f)", r.ax, r.ay, r.az);
    const char * result_str = !r.reached ? "NO_EXEC" : (r.error_mm <= PASS_THRESHOLD_M * 1000.0 ? "PASS" : "FAIL");
    if (r.reached && r.error_mm <= PASS_THRESHOLD_M * 1000.0) passed++;
    RCLCPP_INFO(log, "%-20s  %-20s  %-20s  %-10.2f  %-6s",
      r.name.c_str(), target_buf, actual_buf,
      r.error_mm < 0 ? 0.0 : r.error_mm, result_str);
  }
  RCLCPP_INFO(log, "\n%d / %zu positions within %.0f mm threshold",
    passed, results.size(), PASS_THRESHOLD_M * 1000.0);
  RCLCPP_INFO(log, "Overall: %s", passed == static_cast<int>(results.size()) ? "PASS" : "FAIL");

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return passed == static_cast<int>(results.size()) ? 0 : 1;
}
