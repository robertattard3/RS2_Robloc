#include <cmath>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class MoveRobotNode : public rclcpp::Node
{
public:
  MoveRobotNode() : Node("ur3e_moveit_controller")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "robot_goal_poses",  // topic name — publish a PoseArray here
      10,
      std::bind(&MoveRobotNode::goal_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Waiting for goal poses on /robot_goal_poses...");
  }

  // MoveGroupInterface must be initialized after the node is constructed,
  // so we do it separately and pass a shared_ptr to this node
  void init_move_group(const rclcpp::Node::SharedPtr& node)
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_onrobot_manipulator");
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(2.0);
    move_group_->setMaxVelocityScalingFactor(0.1);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    move_group_->setGoalPositionTolerance(0.005);    // 5 mm — practical for most manipulation tasks
    move_group_->setGoalOrientationTolerance(0.05);  // ~3 deg — reduces IK solver restarts

    // Add table as collision object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = "base_link";
    table.id = "table";

    // Table dimensions (match reference: 10cm thick so collision is robust)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {1.0, 1.0, 0.1};  // 1m x 1m, 10cm thick

    // Position the table — centre at z=-0.06 puts the top surface at z=-0.01,
    // just below the robot base.  Keeping it 1cm below prevents any robot base
    // link from grazing the table while still blocking downward paths.
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.0;
    table_pose.position.z = -0.06;
    table_pose.orientation.w = 1.0;

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface.applyCollisionObject(table);
    RCLCPP_INFO(this->get_logger(), "Table collision object added to planning scene.");

    // Add wall on the -y side (opposite to working area at +x, +y)
    moveit_msgs::msg::CollisionObject wall;
    wall.header.frame_id = "base_link";
    wall.id = "wall";

    shape_msgs::msg::SolidPrimitive wall_primitive;
    wall_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    wall_primitive.dimensions = {1.0, 0.02, 1.0};  // 1m wide, 2cm thick, 1m tall

    geometry_msgs::msg::Pose wall_pose;
    wall_pose.position.x = 0.0;
    wall_pose.position.y = -0.15;  // right behind the robot on the -y side
    wall_pose.position.z = 0.5;    // centred at half height
    wall_pose.orientation.w = 1.0;

    wall.primitives.push_back(wall_primitive);
    wall.primitive_poses.push_back(wall_pose);
    wall.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_interface.applyCollisionObject(wall);
    RCLCPP_INFO(this->get_logger(), "Wall collision object added to planning scene.");

    // Generous workspace — tight bounds clip intermediate waypoints and cause failures
    move_group_->setWorkspace(-0.6, -0.6, -0.1, 0.6, 0.6, 0.9);

    // Gripper publisher — commands finger width in metres
    // RG2 range: 0.0 (closed) to 0.110 (fully open)
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/finger_width_controller/commands", 10);
  }

  void open_gripper()
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {0.11};  // RG2 fully open (metres)
    gripper_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Gripper opening...");
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  }

  void close_gripper()
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {0.0};  // RG2 fully closed
    gripper_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Gripper closing...");
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  }

private:
  void goal_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (!move_group_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroup not initialized yet!");
      return;
    }

    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty PoseArray, ignoring.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Received %zu goal poses.", msg->poses.size());

    // Constrain wrist_3 to [-π, π] so the IK solver never picks a solution
    // that rotates the joint a full extra turn away from zero.
    auto joint_names = move_group_->getJointNames();
    for (const auto & name : joint_names) {
      if (name.find("wrist_3") != std::string::npos) {
        moveit_msgs::msg::JointConstraint jc;
        jc.joint_name = name;
        jc.position = 0.0;
        jc.tolerance_above = 2 * M_PI;
        jc.tolerance_below = 2 * M_PI;
        jc.weight = 1.0;
        moveit_msgs::msg::Constraints constraints;
        constraints.joint_constraints.push_back(jc);
        move_group_->setPathConstraints(constraints);
        break;
      }
    }

    for (size_t i = 0; i < msg->poses.size(); ++i) {
      // Get current state with a generous timeout — the state monitor starts
      // lazily on first call and needs time to receive the first joint state.
      moveit::core::RobotStatePtr start = move_group_->getCurrentState(5.0);
      if (!start) {
        RCLCPP_ERROR(this->get_logger(),
          "Could not fetch current robot state for pose %zu — aborting.", i);
        return;
      }

      // Override finger_width to a safe open value so the gripper is never
      // in self-collision at the start of each plan (finger_width=0 closes
      // the finger tips together which MoveIt treats as a collision)
      const moveit::core::JointModel* jm = start->getJointModel("finger_width");
      if (jm) {
        const double safe_open = 0.05;
        start->setJointPositions(jm, &safe_open);
      }
      move_group_->setStartState(*start);

      move_group_->setPoseTarget(msg->poses[i], move_group_->getEndEffectorLink());

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = false;
      for (int attempt = 1; attempt <= 10 && !success; ++attempt) {
        success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success)
          RCLCPP_WARN(this->get_logger(), "Planning attempt %d/10 failed, retrying...", attempt);
      }

      if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Planning failed for pose %zu after 10 attempts — aborting.", i);
        return;
      }

      // Unwrap wrist_3 (continuous joint) so consecutive waypoints never jump
      // more than π — prevents KDL IK normalization from causing 6.28 rad jumps
      auto & traj = plan.trajectory_.joint_trajectory;
      int wrist3_idx = -1;
      for (size_t j = 0; j < traj.joint_names.size(); ++j) {
        if (traj.joint_names[j].find("wrist_3") != std::string::npos) {
          wrist3_idx = static_cast<int>(j);
          break;
        }
      }
      if (wrist3_idx >= 0) {
        for (size_t pt = 1; pt < traj.points.size(); ++pt) {
          double prev = traj.points[pt - 1].positions[wrist3_idx];
          double & curr = traj.points[pt].positions[wrist3_idx];
          while (curr - prev > M_PI) curr -= 2 * M_PI;
          while (curr - prev < -M_PI) curr += 2 * M_PI;
        }
      }

      RCLCPP_INFO(this->get_logger(), "Executing pose %zu of %zu...", i + 1, msg->poses.size());
      move_group_->execute(plan);
    }

    RCLCPP_INFO(this->get_logger(), "All poses executed.");
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveRobotNode>();
  node->init_move_group(node);

  // MultiThreadedExecutor lets the goal callback run while joint states
  // are still being received on a separate thread (needed for getCurrentJointValues)
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}