#include "yumi_control/yumi_id_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace yumi_control {

YumiInverseDynamicsController::YumiInverseDynamicsController()
    : controller_interface::ControllerInterface() {
}

controller_interface::InterfaceConfiguration YumiInverseDynamicsController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // We use the effort command interface for our custom controller
    for (const auto& joint_name : joint_names_) {
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
    }

    return config;
}

controller_interface::InterfaceConfiguration YumiInverseDynamicsController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // We read position and velocity
    for (const auto& joint_name : joint_names_) {
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }

    return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
YumiInverseDynamicsController::on_init() {
    try {
        // Create a parameter subscriber that can be updated from ROS params
        auto_declare<std::vector<std::string>>("joints", {});
        auto_declare<double>("position_gain", 1000.0);
        auto_declare<double>("velocity_gain", 100.0);
        auto_declare<std::string>("robot_description_topic", "robot_description");

        // Initialize message buffer with empty vector
        position_command_buffer_.writeFromNonRT(std::vector<double>{});
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init(): %s", e.what());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
YumiInverseDynamicsController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    // Get joint names from parameter
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "No joints provided");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Get control gains
    kp_ = get_node()->get_parameter("position_gain").as_double();
    kd_ = get_node()->get_parameter("velocity_gain").as_double();

    // Get robot description topic
    robot_description_topic_ = get_node()->get_parameter("robot_description_topic").as_string();

    RCLCPP_INFO(get_node()->get_logger(), "Subscribing to robot description topic: %s", robot_description_topic_.c_str());

    // Subscribe to robot description topic
    robot_description_subscriber_ = get_node()->create_subscription<std_msgs::msg::String>(
        robot_description_topic_, rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) { robot_description_callback(msg); }
    );

    // Initialize the message buffer with zero values for each joint
    position_command_buffer_.writeFromNonRT(std::vector<double>(joint_names_.size(), 0.0));

    // Create subscription for command topic (use QoS configuration for control)
    position_command_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
        "~/commands", rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { position_command_callback(msg); }
    );

    // Create debug publisher
    debug_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/debug", rclcpp::SystemDefaultsQoS()
    );

    // Wait for robot description with a timeout of 10 seconds
    if (!wait_for_robot_description(rclcpp::Duration::from_seconds(10))) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to receive robot description within timeout");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "YumiInverseDynamicsController configured successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void YumiInverseDynamicsController::robot_description_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_node()->get_logger(), "Received robot description (length: %zu)", msg->data.size());

    // Initialize the robot model with the received description
    robot_description_ = msg->data;
    if (initialize_robot_model()) {
        RCLCPP_INFO(get_node()->get_logger(), "Robot model initialized successfully from topic");
        robot_description_received_ = true;
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize robot model from topic");
    }
}

bool YumiInverseDynamicsController::wait_for_robot_description(const rclcpp::Duration& timeout) {
    RCLCPP_INFO(get_node()->get_logger(), "Waiting for robot description with timeout of %.2f seconds", timeout.seconds());

    auto start_time = get_node()->get_clock()->now();
    auto end_time = start_time + timeout;

    // Check if we already have the robot description
    if (robot_description_received_) {
        RCLCPP_INFO(get_node()->get_logger(), "Robot description already received");
        return true;
    }

    // Wait for the robot description with timeout
    while (rclcpp::ok() && get_node()->get_clock()->now() < end_time) {
        if (robot_description_received_) {
            RCLCPP_INFO(get_node()->get_logger(), "Robot description received after waiting");
            return true;
        }

        // Sleep for a short time to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_ERROR(get_node()->get_logger(), "Timeout waiting for robot description");
    return false;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
YumiInverseDynamicsController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // Check if we have a valid robot description
    if (!robot_description_received_) {
        RCLCPP_ERROR(get_node()->get_logger(), "No robot description received yet, cannot activate");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Reset command buffer with zero commands
    position_command_buffer_.writeFromNonRT(std::vector<double>(joint_names_.size(), 0.0));

    RCLCPP_INFO(get_node()->get_logger(), "YumiInverseDynamicsController activated successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
YumiInverseDynamicsController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // Clear commands
    position_command_buffer_.writeFromNonRT(std::vector<double>{});

    RCLCPP_INFO(get_node()->get_logger(), "YumiInverseDynamicsController deactivated successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool YumiInverseDynamicsController::initialize_robot_model() {
    try {
        RCLCPP_INFO(get_node()->get_logger(), "Loading URDF");

        if (robot_description_.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Robot description is empty");
            return false;
        }

        // Build the model from the URDF string
        pinocchio::urdf::buildModelFromXML(robot_description_, model_);
        model_data_ = pinocchio::Data(model_);

        RCLCPP_INFO(get_node()->get_logger(), "Model loaded successfully with %d DoF", model_.nv);
        RCLCPP_INFO(get_node()->get_logger(), "Model name: %s", model_.name.c_str());

        // List all joint names
        RCLCPP_INFO(get_node()->get_logger(), "Joints in the model:");
        for (int i = 1; i < model_.njoints; ++i) {
            RCLCPP_INFO(get_node()->get_logger(), "  Joint %d: %s (ID: %ld)", i, model_.names[i].c_str(), model_.joints[i].id());
        }

        // Initialize with a neutral configuration
        q_ = pinocchio::neutral(model_);
        v_ = Eigen::VectorXd::Zero(model_.nv);
        a_ = Eigen::VectorXd::Zero(model_.nv);

        // Initialize the gazebo_indices_to_pin_q_idx_ and gazebo_indices_to_pin_v_idx_ maps
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            size_t pinocchio_index = model_.getJointId(joint_names_[i]);
            if (pinocchio_index >= model_.joints.size()) {
                RCLCPP_ERROR(get_node()->get_logger(), "Joint %s not found in the model", joint_names_[i].c_str());
                return false;
            }
            gazebo_indices_to_pin_q_idx_[i] = model_.joints[pinocchio_index].idx_q();
            gazebo_indices_to_pin_v_idx_[i] = model_.joints[pinocchio_index].idx_v();
        }

        // Log dimensions of state vectors for debugging
        RCLCPP_INFO(get_node()->get_logger(), "State vector dimensions: q(%ld), v(%ld), a(%ld)", q_.size(), v_.size(), a_.size());

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error loading model: %s", e.what());
        return false;
    }
}

controller_interface::return_type
YumiInverseDynamicsController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // Get command from buffer
    auto position_commands = *position_command_buffer_.readFromRT();

    // Check if we have a valid command
    if (position_commands.empty()) {
        // No command received yet
        return controller_interface::return_type::OK;
    }

    // Check command size
    if (position_commands.size() != joint_names_.size()) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *(get_node()->get_clock()),
            1000,
            "Wrong number of joint commands. Got %zu, expected %zu",
            position_commands.size(), joint_names_.size()
        );
        return controller_interface::return_type::ERROR;
    }

    // Read current joint states from hardware interfaces
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        // Map controller joints to model joints
        // Get joint position and velocity from state interfaces
        q_(gazebo_indices_to_pin_q_idx_[i]) = state_interfaces_[i * 2].get_value();      // Position interface
        v_(gazebo_indices_to_pin_v_idx_[i]) = state_interfaces_[i * 2 + 1].get_value();  // Velocity interface
    }

    // Create target acceleration using PD control law
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        // Map controller joints to model joints again
        // Compute acceleration based on position error and velocity damping
        double position_error = position_commands[i] - q_(gazebo_indices_to_pin_q_idx_[i]);
        a_(gazebo_indices_to_pin_v_idx_[i]) = kp_ * position_error - kd_ * v_(gazebo_indices_to_pin_v_idx_[i]);
    }

    // Compute inverse dynamics using Pinocchio
    Eigen::VectorXd tau = pinocchio::rnea(model_, model_data_, q_, v_, a_);

    // Apply torque commands to the joints
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        // Map controller joints to model joints once more
        // Get the torque for this joint from our computed values
        command_interfaces_[i].set_value(tau(gazebo_indices_to_pin_v_idx_[i]));
    }

    // Publish debug info
    auto debug_msg = std::make_unique<sensor_msgs::msg::JointState>();
    // Add timestamp to the message
    debug_msg->header.stamp = get_node()->get_clock()->now();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        debug_msg->name.push_back(joint_names_[i]);
        debug_msg->position.push_back(q_(gazebo_indices_to_pin_q_idx_[i]));
        debug_msg->velocity.push_back(v_(gazebo_indices_to_pin_v_idx_[i]));
        debug_msg->effort.push_back(tau(gazebo_indices_to_pin_v_idx_[i]));
    }
    debug_publisher_->publish(std::move(debug_msg));

    return controller_interface::return_type::OK;
}

void YumiInverseDynamicsController::position_command_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg
) {
    // Check if we have position data
    if (msg->position.empty()) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *(get_node()->get_clock()),
            1000,
            "Received JointState message with empty position data"
        );
        return;
    }

    // Validate that all required joints are present in the message
    std::vector<double> position_commands(joint_names_.size(), 0.0);
    bool all_joints_found = true;

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
        if (it == msg->name.end()) {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(),
                *(get_node()->get_clock()),
                1000,
                "Joint '%s' not found in the received JointState message",
                joint_names_[i].c_str()
            );
            all_joints_found = false;
            continue;
        }

        size_t index = std::distance(msg->name.begin(), it);
        if (index < msg->position.size()) {
            position_commands[i] = msg->position[index];
        } else {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(),
                *(get_node()->get_clock()),
                1000,
                "Position data for joint '%s' not available in the received JointState message",
                joint_names_[i].c_str()
            );
            all_joints_found = false;
        }
    }

    if (!all_joints_found) {
        return;
    }

    // Write command to buffer to be used in the real-time update loop
    position_command_buffer_.writeFromNonRT(position_commands);
}

}  // namespace yumi_control

// Export controller as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(yumi_control::YumiInverseDynamicsController, controller_interface::ControllerInterface)