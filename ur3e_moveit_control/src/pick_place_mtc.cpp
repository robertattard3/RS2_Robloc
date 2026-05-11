#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace mtc = moveit::task_constructor;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_mtc");

static const double CUBE_SIDE    = 0.05;   // 5 cm
static const char*  ARM_GROUP    = "ur_onrobot_manipulator";
static const char*  HAND_GROUP   = "ur_onrobot_gripper";
static const char*  HAND_FRAME   = "gripper_tcp";
static const char*  WORLD_FRAME  = "base_link";

// Wipes the entire planning scene and rebuilds it from scratch.
// Called before every pick so stale ACM entries or residual objects left by
// MTC never carry over and corrupt subsequent OMPL planning.
static void resetScene(const geometry_msgs::msg::Pose& cube_pose)
{
  moveit::planning_interface::PlanningSceneInterface psi;

  // Detach any cube still attached from a previous run
  moveit_msgs::msg::AttachedCollisionObject detach;
  detach.object.id        = "cube";
  detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  psi.applyAttachedCollisionObject(detach);

  // Remove all world objects so nothing stale remains
  psi.removeCollisionObjects({"table", "cube"});
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Rebuild table
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = WORLD_FRAME;
  table.id = "table";
  shape_msgs::msg::SolidPrimitive tp;
  tp.type = shape_msgs::msg::SolidPrimitive::BOX;
  tp.dimensions = {1.0, 1.0, 0.1};
  geometry_msgs::msg::Pose table_pose;
  table_pose.position.z = -0.065;
  table_pose.orientation.w = 1.0;
  table.primitives.push_back(tp);
  table.primitive_poses.push_back(table_pose);
  table.operation = moveit_msgs::msg::CollisionObject::ADD;
  psi.applyCollisionObject(table);

  // Rebuild cube
  moveit_msgs::msg::CollisionObject cube;
  cube.header.frame_id = WORLD_FRAME;
  cube.id = "cube";
  shape_msgs::msg::SolidPrimitive cp;
  cp.type = shape_msgs::msg::SolidPrimitive::BOX;
  cp.dimensions = {CUBE_SIDE, CUBE_SIDE, CUBE_SIDE};
  cube.primitives.push_back(cp);
  cube.primitive_poses.push_back(cube_pose);
  cube.operation = moveit_msgs::msg::CollisionObject::ADD;
  psi.applyCollisionObject(cube);

  RCLCPP_INFO(LOGGER, "Scene reset: cube at (%.3f, %.3f, %.3f)",
    cube_pose.position.x, cube_pose.position.y, cube_pose.position.z);
}

// ── Task 1: approach the cube and close the gripper ──────────────────────────
// Deliberately does NOT include the lift — the lift is a separate step so the
// gripper has time to fully engage before the robot starts moving upward.
static bool runPickGrasp(const rclcpp::Node::SharedPtr& node,
                         const geometry_msgs::msg::Pose& cube_pose)
{
  mtc::Task task;
  task.stages()->setName("pick_grasp");
  task.loadRobotModel(node);

  // Build a fresh planning scene from the robot model so the task never
  // inherits a stale world state from the PlanningSceneMonitor.  After the
  // first run the monitor may hold the cube at the placement height (which
  // can be inside or touching the table), making every approach pose appear
  // to be in collision so OMPL completely fails.  Constructing from the robot
  // model also re-initialises the ACM from the SRDF, clearing any residual
  // allowed-collision entries left by previous MTC task executions.
  auto fresh_scene = std::make_shared<planning_scene::PlanningScene>(task.getRobotModel());
  {
    // Inject the actual robot joint state so FixedState reports the real pose.
    auto mg_tmp = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, ARM_GROUP);
    mg_tmp->startStateMonitor(2.0);
    auto rs = mg_tmp->getCurrentState(2.0);
    if (rs) fresh_scene->setCurrentState(*rs);

    // Normalise continuous joints to (-π, π].  After a full pick-place cycle
    // a continuous joint like wrist_3 can accumulate to ±2π (physically the
    // same position but a different number).  OMPL's SO2StateSpace does NOT
    // auto-normalise when values are written in, so a start value of -2π is
    // treated as out-of-bounds and RRTConnect fails immediately every time.
    {
      auto& norm = fresh_scene->getCurrentStateNonConst();
      for (const auto* jm : task.getRobotModel()->getJointModels()) {
        if (jm->getType() != moveit::core::JointModel::REVOLUTE) continue;
        const auto& bounds = jm->getVariableBounds();
        if (bounds.empty() || bounds[0].position_bounded_) continue;
        double v = norm.getVariablePosition(jm->getFirstVariableIndex());
        while (v >  M_PI) v -= 2.0 * M_PI;
        while (v <= -M_PI) v += 2.0 * M_PI;
        norm.setVariablePosition(jm->getFirstVariableIndex(), v);
      }
      norm.update();
    }

    moveit_msgs::msg::CollisionObject tbl;
    tbl.header.frame_id = WORLD_FRAME;
    tbl.id = "table";
    shape_msgs::msg::SolidPrimitive tp;
    tp.type = shape_msgs::msg::SolidPrimitive::BOX;
    tp.dimensions = {1.0, 1.0, 0.1};
    geometry_msgs::msg::Pose tbl_pose;
    tbl_pose.position.z = -0.065;
    tbl_pose.orientation.w = 1.0;
    tbl.primitives.push_back(tp);
    tbl.primitive_poses.push_back(tbl_pose);
    tbl.operation = moveit_msgs::msg::CollisionObject::ADD;
    fresh_scene->processCollisionObjectMsg(tbl);

    moveit_msgs::msg::CollisionObject cub;
    cub.header.frame_id = WORLD_FRAME;
    cub.id = "cube";
    shape_msgs::msg::SolidPrimitive cp;
    cp.type = shape_msgs::msg::SolidPrimitive::BOX;
    cp.dimensions = {CUBE_SIDE, CUBE_SIDE, CUBE_SIDE};
    cub.primitives.push_back(cp);
    cub.primitive_poses.push_back(cube_pose);
    cub.operation = moveit_msgs::msg::CollisionObject::ADD;
    fresh_scene->processCollisionObjectMsg(cub);
  }

  task.setProperty("group",    ARM_GROUP);
  task.setProperty("eef",      HAND_GROUP);
  task.setProperty("ik_frame", HAND_FRAME);

  auto sampling_planner      = std::make_shared<mtc::solvers::PipelinePlanner>(node);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner     = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.1);
  cartesian_planner->setMaxAccelerationScalingFactor(0.1);
  cartesian_planner->setStepSize(0.01);

  mtc::Stage* current_state_ptr = nullptr;
  {
    auto s = std::make_unique<mtc::stages::FixedState>("current state", fresh_scene);
    current_state_ptr = s.get();
    task.add(std::move(s));
  }
  {
    auto s = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    s->setGroup(HAND_GROUP);
    s->setGoal("open");
    task.add(std::move(s));
  }
  {
    auto s = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{{ARM_GROUP, sampling_planner}});
    s->setTimeout(30.0);
    s->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(s));
  }
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto s = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      s->properties().set("marker_ns", "approach_object");
      s->properties().set("link", HAND_FRAME);
      s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      s->setMinMaxDistance(0.05, 0.15);
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = HAND_FRAME;
      vec.vector.z = 1.0;
      s->setDirection(vec);
      grasp->insert(std::move(s));
    }
    {
      auto gen = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      gen->properties().configureInitFrom(mtc::Stage::PARENT);
      gen->setPreGraspPose("open");
      gen->setObject("cube");
      gen->setAngleDelta(M_PI / 2);   // 90° steps — 4 grasp angles
      gen->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      grasp_frame_transform.linear() =
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(gen));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, HAND_FRAME);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }
    {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,cube)");
      s->allowCollisions(
        "cube",
        task.getRobotModel()->getJointModelGroup(HAND_GROUP)->getLinkModelNamesWithCollisionGeometry(),
        true);
      grasp->insert(std::move(s));
    }
    {
      auto s = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      s->setGroup(HAND_GROUP);
      s->setGoal("closed");
      grasp->insert(std::move(s));
    }
    {
      auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cube");
      s->attachObject("cube", HAND_FRAME);
      grasp->insert(std::move(s));
    }
    task.add(std::move(grasp));
  }

  try { task.init(); }
  catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, "Pick-grasp task init failed: " << e);
    return false;
  }
  if (!task.plan(1)) {
    RCLCPP_ERROR(LOGGER, "Pick-grasp planning failed");
    return false;
  }
  task.introspection().publishSolution(*task.solutions().front());
  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Pick-grasp execution failed");
    return false;
  }
  RCLCPP_INFO(LOGGER, "Grasp complete — waiting for gripper to fully engage...");
  return true;
}

// ── Step 1b: lift the cube after the gripper has had time to engage ───────────
static bool runLift(const rclcpp::Node::SharedPtr& node)
{
  auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, ARM_GROUP);
  mg->setMaxVelocityScalingFactor(0.1);
  mg->setMaxAccelerationScalingFactor(0.1);
  mg->startStateMonitor(2.0);

  auto current = mg->getCurrentPose(HAND_FRAME).pose;
  geometry_msgs::msg::Pose target = current;
  target.position.z += 0.15;   // aim for 15 cm; accept whatever fraction is reachable

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = mg->computeCartesianPath({target}, 0.01, 0.0, trajectory);
  // Accept as long as we lifted at least 5 cm — enough clearance for rotation.
  if (fraction < 0.33) {
    RCLCPP_ERROR(LOGGER, "Lift path only %.0f%% complete (< 5 cm), aborting", fraction * 100.0);
    return false;
  }
  RCLCPP_INFO(LOGGER, "Lift path %.0f%% complete (%.1f cm)", fraction * 100.0, fraction * 15.0);

  mg->startStateMonitor(2.0);
  if (mg->execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Lift execution failed");
    return false;
  }
  RCLCPP_INFO(LOGGER, "Pick-lift completed");
  return true;
}

// ── Step 2: rotate 90° in air via MoveGroupInterface ─────────────────────────
// MTC's CartesianPath can't handle the wrist singularity during a pure orientation
// change. MoveGroupInterface::computeCartesianPath lets us disable the jump threshold
// so the planner can reconfigure the wrist without being stopped by a joint jump.
static bool rotateInAir(const rclcpp::Node::SharedPtr& node)
{
  auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, ARM_GROUP);
  mg->setMaxVelocityScalingFactor(0.1);
  mg->setMaxAccelerationScalingFactor(0.1);

  // Wait for a fresh joint state so the start-state validation in execute() passes.
  // Without this, on repeated runs the state monitor may hold a stale reading and
  // the trajectory is immediately aborted with "allowed_start_tolerance" exceeded.
  mg->startStateMonitor(2.0);

  auto current = mg->getCurrentPose(HAND_FRAME).pose;

  // Step 1: rotate 90° around world X to change which face is up.
  // The grasp angle varies between runs, so +90° may hit a singularity from some
  // starting configurations. If it does, retry with -90° — both flip to a
  // different face; only which face differs.
  Eigen::Quaterniond q_orig(
    current.orientation.w, current.orientation.x,
    current.orientation.y, current.orientation.z);

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = 0.0;

  for (double angle : {M_PI / 2, -M_PI / 2}) {
    Eigen::Quaterniond q = (Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()) * q_orig).normalized();

    geometry_msgs::msg::Pose target = current;
    target.orientation.x = q.x();
    target.orientation.y = q.y();
    target.orientation.z = q.z();
    target.orientation.w = q.w();

    mg->setStartStateToCurrentState();
    trajectory = moveit_msgs::msg::RobotTrajectory{};
    fraction = mg->computeCartesianPath({target}, 0.01, 0.0, trajectory);

    if (fraction >= 0.8) break;

    RCLCPP_WARN(LOGGER, "Face rotation %.0f° only %.0f%% complete, trying opposite direction.",
      angle * 180.0 / M_PI, fraction * 100.0);
  }

  if (fraction < 0.8) {
    RCLCPP_ERROR(LOGGER, "Face rotation failed in both directions (%.0f%% max)", fraction * 100.0);
    return false;
  }

  // Re-sync the state monitor right before execution so the start-state
  // tolerance check sees the actual current joint positions.
  mg->startStateMonitor(2.0);

  auto result = mg->execute(trajectory);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Face rotation execution failed");
    return false;
  }

  // Step 2: rotate wrist_3 by -90° to swing the fingers back to horizontal.
  // Re-sync the state monitor — the Cartesian execution just finished and the
  // joint states may not have propagated yet if we query immediately.
  mg->startStateMonitor(2.0);
  mg->setStartStateToCurrentState();
  auto joint_values = mg->getCurrentJointValues();
  joint_values.back() -= M_PI / 2;   // negative = rotates fingers to horizontal
  // Normalise the target to (-π, π] so the subsequent return-home move never
  // takes the long way around and leaves wrist_3 accumulated at ±2π.
  while (joint_values.back() >  M_PI) joint_values.back() -= 2.0 * M_PI;
  while (joint_values.back() <= -M_PI) joint_values.back() += 2.0 * M_PI;
  mg->setJointValueTarget(joint_values);
  if (mg->move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Wrist rotation failed");
    return false;
  }

  RCLCPP_INFO(LOGGER, "Rotation complete");
  return true;
}

// ── Task 2: descend, release, retreat, home ───────────────────────────────────
static bool runDescendAndPlace(const rclcpp::Node::SharedPtr& node)
{
  mtc::Task task;
  task.stages()->setName("descend_place");
  task.loadRobotModel(node);

  task.setProperty("group",    ARM_GROUP);
  task.setProperty("eef",      HAND_GROUP);
  task.setProperty("ik_frame", HAND_FRAME);

  auto sampling_planner      = std::make_shared<mtc::solvers::PipelinePlanner>(node);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner     = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.1);
  cartesian_planner->setMaxAccelerationScalingFactor(0.1);
  cartesian_planner->setStepSize(0.01);

  {
    auto s = std::make_unique<mtc::stages::CurrentState>("current state");
    task.add(std::move(s));
  }
  // Gripper opens at post-rotation height — in real hardware the cube falls to the table;
  // in simulation it remains at the released position.
  // Lower until the cube is ~2 cm above the table surface (table top at z=0,
  // cube half-height=0.025, so target gripper z≈0.045).
  {
    auto s = std::make_unique<mtc::stages::MoveRelative>("lower to table", cartesian_planner);
    s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    s->setMinMaxDistance(0.01, 0.05);
    s->setIKFrame(HAND_FRAME);
    s->properties().set("marker_ns", "lower_to_table");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = WORLD_FRAME;
    vec.vector.z = -1.0;
    s->setDirection(vec);
    task.add(std::move(s));
  }
  {
    auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,cube)");
    s->allowCollisions(
      "cube",
      task.getRobotModel()->getJointModelGroup(HAND_GROUP)->getLinkModelNamesWithCollisionGeometry(),
      true);
    task.add(std::move(s));
  }
  {
    auto s = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    s->setGroup(HAND_GROUP);
    s->setGoal("open");
    task.add(std::move(s));
  }
  {
    auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,cube)");
    s->allowCollisions(
      "cube",
      task.getRobotModel()->getJointModelGroup(HAND_GROUP)->getLinkModelNamesWithCollisionGeometry(),
      false);
    task.add(std::move(s));
  }
  {
    auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cube");
    s->detachObject("cube", HAND_FRAME);
    task.add(std::move(s));
  }
  {
    auto s = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    // After the face rotation the HAND_FRAME -z axis is no longer pointing
    // straight back, so the Cartesian retreat can be short.  2 cm minimum
    // (was 5 cm) ensures planning succeeds even when the arm is close to the
    // table after descent.
    s->setMinMaxDistance(0.02, 0.15);
    s->setIKFrame(HAND_FRAME);
    s->properties().set("marker_ns", "retreat");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = HAND_FRAME;
    vec.vector.z = -1.0;
    s->setDirection(vec);
    task.add(std::move(s));
  }
  {
    // Use interpolation planner so return-home always takes the shortest
    // angular path for each joint.  The OMPL sampling planner is stochastic
    // and occasionally wraps the continuous wrist_3 joint by a full 2π on the
    // way back, leaving the robot at wrist_3 = -2π.  The next cycle then
    // fails immediately because the controller sees a 6.28 rad start error.
    // Interpolation is deterministic and always shortest-path, eliminating
    // the accumulation.  If the direct joint-space path is ever in collision
    // the task will fail and the operator should restart — but in practice
    // after the retreat stage the path to home is always clear.
    auto s = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    s->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    s->setGoal("home");
    task.add(std::move(s));
  }

  try { task.init(); }
  catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, "Descend-place task init failed: " << e);
    return false;
  }
  if (!task.plan(5)) {
    RCLCPP_ERROR(LOGGER, "Descend-place planning failed");
    return false;
  }
  task.introspection().publishSolution(*task.solutions().front());
  auto result = task.execute(*task.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Descend-place execution failed");
    return false;
  }
  RCLCPP_INFO(LOGGER, "Pick-rotate-place completed successfully");
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("pick_place_mtc", options);

  auto status_pub = node->create_publisher<std_msgs::msg::Bool>("/pick_place_status", 10);

  std::atomic<bool> pose_received{false};
  geometry_msgs::msg::Pose cube_pose;
  std::mutex pose_mutex;

  std::atomic<bool> estop_active{false};

  auto sub = node->create_subscription<geometry_msgs::msg::PoseArray>(
    "robot_goal_poses", 10,
    [&](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
      if (!pose_received.load() && !msg->poses.empty()) {
        std::lock_guard<std::mutex> lock(pose_mutex);
        cube_pose = msg->poses[0];
        pose_received.store(true);
      }
    });

  auto estop_sub = node->create_subscription<std_msgs::msg::Bool>(
    "/emergency_stop", 10,
    [&](const std_msgs::msg::Bool::SharedPtr msg) {
      estop_active.store(msg->data);
      if (msg->data)
        RCLCPP_WARN(LOGGER, "E-stop active — waiting for unlock.");
      else
        RCLCPP_INFO(LOGGER, "E-stop cleared — resuming.");
    });

  // Blocks until the e-stop is cleared (or the node shuts down).
  auto waitIfEstopped = [&]() {
    while (rclcpp::ok() && estop_active.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  };

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spin_thread = std::make_unique<std::thread>([&executor]() {
    executor.spin();
  });

  // Initial scene will be built by resetScene() before the first pick.

  std_msgs::msg::Bool status_msg;

  while (rclcpp::ok()) {
    RCLCPP_INFO(LOGGER, "Waiting for cube pose on /robot_goal_poses...");
    while (rclcpp::ok() && !pose_received.load()) {
      status_msg.data = false;
      status_pub->publish(status_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (!rclcpp::ok()) break;

    geometry_msgs::msg::Pose current_pose;
    {
      std::lock_guard<std::mutex> lock(pose_mutex);
      current_pose = cube_pose;
    }

    RCLCPP_INFO(LOGGER, "Cube pose received: (%.3f, %.3f, %.3f)",
      current_pose.position.x, current_pose.position.y, current_pose.position.z);

    // Signal running — disables the UI button
    status_msg.data = true;
    status_pub->publish(status_msg);

    {
      auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, ARM_GROUP);
      mg->setNamedTarget("home");
      RCLCPP_INFO(LOGGER, "Moving arm to home position...");
      mg->move();
      RCLCPP_INFO(LOGGER, "Arm at home position");
    }

    resetScene(current_pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    waitIfEstopped();
    if (!rclcpp::ok()) break;

    if (runPickGrasp(node, current_pose)) {
      // Wait for the gripper to fully close and mechanically engage the cube
      // before lifting — without this pause the arm starts moving upward while
      // the fingers are still settling, risking dropping the cube.
      std::this_thread::sleep_for(std::chrono::milliseconds(800));
      waitIfEstopped();
      if (!rclcpp::ok()) break;

      if (!runLift(node)) {
        pose_received.store(false);
        status_msg.data = false;
        status_pub->publish(status_msg);
        RCLCPP_INFO(LOGGER, "Ready for next pick - press Start.");
        continue;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      waitIfEstopped();
      if (!rclcpp::ok()) break;

      if (rotateInAir(node)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        waitIfEstopped();
        if (!rclcpp::ok()) break;

        runDescendAndPlace(node);
      }
    }

    // After each full cycle, physically unwind any continuous joint that has
    // accumulated outside (-π, π].  MoveGroupInterface treats -2π ≡ 0 via
    // angular distance and silently skips the move, so we build a trajectory
    // manually: its first point matches the actual encoder position, satisfying
    // both MoveIt's start-tolerance check and the UR controller's path-tolerance
    // check, then the second point drives the joint to its canonical value.
    {
      auto mg_uw = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, ARM_GROUP);
      mg_uw->startStateMonitor(1.0);
      auto rs_uw = mg_uw->getCurrentState(1.0);
      if (rs_uw) {
        std::vector<double> cur;
        rs_uw->copyJointGroupPositions(ARM_GROUP, cur);
        auto jnames = mg_uw->getJointNames();

        std::vector<double> tgt = cur;
        bool needs_uw = false;
        for (size_t i = 0; i < cur.size(); ++i) {
          const auto* jm = rs_uw->getRobotModel()->getJointModel(jnames[i]);
          if (!jm || jm->getType() != moveit::core::JointModel::REVOLUTE) continue;
          const auto& bnd = jm->getVariableBounds();
          if (!bnd.empty() && bnd[0].position_bounded_) continue;  // skip bounded joints
          double v = cur[i];
          while (v >  M_PI) v -= 2.0 * M_PI;
          while (v <= -M_PI) v += 2.0 * M_PI;
          if (std::abs(cur[i] - v) > 0.1) { tgt[i] = v; needs_uw = true; }
        }

        if (needs_uw) {
          RCLCPP_INFO(LOGGER, "Unwinding continuous joint(s) to canonical range...");
          moveit_msgs::msg::RobotTrajectory uw_traj;
          uw_traj.joint_trajectory.joint_names = jnames;

          // Point 0: actual current values — controller sees zero start error
          trajectory_msgs::msg::JointTrajectoryPoint p0;
          p0.positions = cur;
          p0.velocities.assign(cur.size(), 0.0);
          p0.time_from_start = rclcpp::Duration(0, 0);
          uw_traj.joint_trajectory.points.push_back(p0);

          // Point 1: normalised values — 2π rotation at ~10 % velocity
          double max_delta = 0.0;
          for (size_t i = 0; i < cur.size(); ++i)
            max_delta = std::max(max_delta, std::abs(tgt[i] - cur[i]));
          double dur = std::max(max_delta / (0.1 * 2.0 * M_PI) * 1.3, 2.0);

          trajectory_msgs::msg::JointTrajectoryPoint p1;
          p1.positions = tgt;
          p1.velocities.assign(tgt.size(), 0.0);
          p1.time_from_start = rclcpp::Duration::from_seconds(dur);
          uw_traj.joint_trajectory.points.push_back(p1);

          auto uw_res = mg_uw->execute(uw_traj);
          if (uw_res == moveit::core::MoveItErrorCode::SUCCESS)
            RCLCPP_INFO(LOGGER, "Joint unwind completed.");
          else
            RCLCPP_WARN(LOGGER, "Joint unwind failed (%d) — will retry next cycle.", uw_res.val);
        }
      }
    }

    // Reset so next button press triggers a new run
    pose_received.store(false);

    // Signal idle — re-enables the UI button
    status_msg.data = false;
    status_pub->publish(status_msg);
    RCLCPP_INFO(LOGGER, "Ready for next pick - press Start.");
  }

  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
