#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "pick_And_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto action_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
        node, "hand_controller/gripper_cmd");

  if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
        RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
        return 1;
    }


  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");
  auto goal_msg = control_msgs::action::GripperCommand::Goal();


  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");


  //'''Code to move to pick position'''
  move_group_interface.setNamedTarget("pick");
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
  RCLCPP_INFO(logger, "Moved to Pick Position");


  //'''Code to open and close the gripper'''
  goal_msg.command.position = 0.05; // Example position
  goal_msg.command.max_effort = 0.0; // Example max effort
  auto goal_handle_future = action_client->async_send_goal(goal_msg);
    // Wait for the result
  rclcpp::spin_until_future_complete(node, goal_handle_future);
  RCLCPP_INFO(logger, "Gripper OPEN!");
  rclcpp::sleep_for(std::chrono::seconds(2));
  goal_msg.command.position = 0.0; // Example position
  goal_msg.command.max_effort = 0.0; // Example max effort
  goal_handle_future = action_client->async_send_goal(goal_msg);
    // Wait for the result
  rclcpp::spin_until_future_complete(node, goal_handle_future);
  RCLCPP_INFO(logger, "Gripper CLOSED!");


  //'''Code to move to pick position'''
  move_group_interface.setNamedTarget("place");
  // Create a plan to that target pose
  auto const [success1, plan1] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  // Execute the plan
  if(success1) {
    move_group_interface.execute(plan1);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
  RCLCPP_INFO(logger, "Moved to Place Position");


  //'''Code to open and close the gripper'''
  goal_msg.command.position = 0.05; // Example position
  goal_msg.command.max_effort = 0.0; // Example max effort
  goal_handle_future = action_client->async_send_goal(goal_msg);
    // Wait for the result
  rclcpp::spin_until_future_complete(node, goal_handle_future);
  RCLCPP_INFO(logger, "Gripper OPEN!");


  //'''Code to move to home position'''
  move_group_interface.setNamedTarget("home");
  // Create a plan to that target pose
  auto const [success2, plan2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  // Execute the plan
  if(success2) {
    move_group_interface.execute(plan2);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
    RCLCPP_INFO(logger, "Moved to Home Position");
  goal_msg.command.position = 0.0; // Example position
  goal_msg.command.max_effort = 0.0; // Example max effort
  goal_handle_future = action_client->async_send_goal(goal_msg);
    // Wait for the result
  rclcpp::spin_until_future_complete(node, goal_handle_future);
  RCLCPP_INFO(logger, "Gripper CLOSED!");


  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}