#ifndef YUMI_CONTROL__YUMI_ID_CONTROLLER_HPP_
#define YUMI_CONTROL__YUMI_ID_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Pinocchio includes
#include <eigen3/Eigen/Dense>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace yumi_control {

class YumiInverseDynamicsController : public controller_interface::ControllerInterface {
   public:
    YumiInverseDynamicsController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

   private:
    // Initialize the Pinocchio model from robot_description parameter
    bool initialize_robot_model();

    // Callback for robot description topic
    void robot_description_callback(const std_msgs::msg::String::SharedPtr msg);

    // Wait for robot description with timeout
    bool wait_for_robot_description(const rclcpp::Duration& timeout);

    // Subscription to the position command
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_command_subscriber_;

    // Subscription to the robot description topic
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;

    // Buffer for the command
    realtime_tools::RealtimeBuffer<std::vector<double>> position_command_buffer_;

    // Joint names from parameters
    std::vector<std::string> joint_names_;

    // Command message callback
    void position_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Pinocchio model and data
    pinocchio::Model model_;
    pinocchio::Data model_data_;
    Eigen::VectorXd q_;  // Joint positions
    Eigen::VectorXd v_;  // Joint velocities
    Eigen::VectorXd a_;  // Joint accelerations

    // Debug publisher for model state
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr debug_publisher_;

    // Parameters
    std::string robot_description_;
    std::string robot_description_topic_;
    double kp_ = 1000.0;  // Position gain
    double kd_ = 100.0;   // Velocity gain

    // Flag to track if robot description has been received
    bool robot_description_received_ = false;

    // Mappings from gazebo indices to position in q and v vector
    std::unordered_map<size_t, int> gazebo_indices_to_pin_q_idx_;
    std::unordered_map<size_t, int> gazebo_indices_to_pin_v_idx_;
};

}  // namespace yumi_control

#endif  // YUMI_CONTROL__YUMI_ID_CONTROLLER_HPP_