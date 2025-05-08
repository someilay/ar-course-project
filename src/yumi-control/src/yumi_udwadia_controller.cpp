#include "yumi_control/yumi_udwadia_controller.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace yumi_control {

YumiUdwadiaController::YumiUdwadiaController()
    : controller_interface::ControllerInterface() {
}

// Utility function to convert a ROS Pose message to a Pinocchio SE3 transform
pin::SE3 YumiUdwadiaController::pose_msg_to_se3(const geometry_msgs::msg::Pose& pose_msg) {
    // Extract position
    Vec3d position;
    position << pose_msg.position.x, pose_msg.position.y, pose_msg.position.z;

    // Extract orientation as quaternion and convert to rotation matrix
    Eigen::Quaterniond quat(
        pose_msg.orientation.w,
        pose_msg.orientation.x,
        pose_msg.orientation.y,
        pose_msg.orientation.z
    );
    Mat3d rotation = quat.toRotationMatrix();

    // Create SE3 transform
    return pin::SE3(rotation, position);
}

geometry_msgs::msg::Pose YumiUdwadiaController::se3_to_pose_msg(const pin::SE3& se3) {
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = se3.translation()[0];
    pose_msg.position.y = se3.translation()[1];
    pose_msg.position.z = se3.translation()[2];

    // Extract orientation from rotation matrix
    Eigen::Matrix3d rotation = se3.rotation();
    Eigen::Quaterniond quat(rotation);
    pose_msg.orientation.x = quat.x();
    pose_msg.orientation.y = quat.y();
    pose_msg.orientation.z = quat.z();
    pose_msg.orientation.w = quat.w();

    return pose_msg;
}

// Utility function to convert a ROS Twist message to a Pinocchio spatial motion
pin::Motion YumiUdwadiaController::twist_msg_to_motion(const geometry_msgs::msg::Twist& twist_msg) {
    // Extract linear velocity
    Vec3d linear;
    linear << twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z;

    // Extract angular velocity
    Vec3d angular;
    angular << twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z;

    // Create spatial motion
    return pin::Motion(linear, angular);
}

geometry_msgs::msg::Twist YumiUdwadiaController::motion_to_twist_msg(const pin::Motion& motion) {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = motion.linear()[0];
    twist_msg.linear.y = motion.linear()[1];
    twist_msg.linear.z = motion.linear()[2];

    twist_msg.angular.x = motion.angular()[0];
    twist_msg.angular.y = motion.angular()[1];
    twist_msg.angular.z = motion.angular()[2];

    return twist_msg;
}

pin::Motion YumiUdwadiaController::accel_msg_to_motion(const geometry_msgs::msg::Accel& accel_msg) {
    // Extract linear acceleration
    Vec3d linear;
    linear << accel_msg.linear.x, accel_msg.linear.y, accel_msg.linear.z;

    // Extract angular acceleration
    Vec3d angular;
    angular << accel_msg.angular.x, accel_msg.angular.y, accel_msg.angular.z;

    // Create spatial motion
    return pin::Motion(linear, angular);
}

controller_interface::InterfaceConfiguration YumiUdwadiaController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // We use the effort command interface for our controller
    for (const auto& joint_name : joint_names_) {
        config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
    }

    return config;
}

controller_interface::InterfaceConfiguration YumiUdwadiaController::state_interface_configuration() const {
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
YumiUdwadiaController::on_init() {
    try {
        // Create a parameter subscriber that can be updated from ROS params
        auto_declare<std::vector<std::string>>("joints", {});
        auto_declare<double>("position_gain", 100.0);
        auto_declare<double>("velocity_gain", 10.0);
        auto_declare<double>("constraint_position_gain", 100.0);
        auto_declare<double>("constraint_velocity_gain", 10.0);
        auto_declare<double>("control_weight", 40.0);
        auto_declare<int>("qp_max_iterations", 100);
        auto_declare<double>("qp_tolerance", 1e-4);
        auto_declare<std::string>("robot_description_topic", "robot_description");
        auto_declare<std::string>("start_link_name", "undefined");
        auto_declare<std::string>("end_link_name", "undefined");

        // Parameters for tDiff_ (relative transform between links)
        auto_declare<double>("tdiff_x", 0.0);
        auto_declare<double>("tdiff_y", 0.4);
        auto_declare<double>("tdiff_z", 0.0);
        auto_declare<double>("tdiff_roll", 0.0);
        auto_declare<double>("tdiff_pitch", 0.0);
        auto_declare<double>("tdiff_yaw", 0.0);

        // Parameters for initial position
        auto_declare<double>("initial_position_x", 0.6);
        auto_declare<double>("initial_position_y", -0.2);
        auto_declare<double>("initial_position_z", 0.5);

        // Parameters for initial orientation (in degrees)
        auto_declare<double>("initial_orientation_roll", 0.0);
        auto_declare<double>("initial_orientation_pitch", 0.0);
        auto_declare<double>("initial_orientation_yaw", 0.0);

        // Initialize task space command buffer with identity pose and zero velocity/acceleration
        TaskSpaceCommand default_command;
        default_command.pose = pin::SE3::Identity();
        default_command.velocity = pin::Motion::Zero();
        default_command.acceleration = pin::Motion::Zero();
        task_command_buffer_.writeFromNonRT(default_command);

        // Initialize task trajectory buffer with empty vector
        task_trajectory_buffer_.writeFromNonRT(std::vector<TaskTrajectoryPoint>{});
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init(): %s", e.what());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
YumiUdwadiaController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    // Get joint names from parameter
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "No joints provided");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Get control gains and parameters
    kp_ = get_node()->get_parameter("position_gain").as_double();
    kd_ = get_node()->get_parameter("velocity_gain").as_double();
    constraint_kp_ = get_node()->get_parameter("constraint_position_gain").as_double();
    constraint_kd_ = get_node()->get_parameter("constraint_velocity_gain").as_double();
    control_weight_ = get_node()->get_parameter("control_weight").as_double();
    qp_max_iterations_ = get_node()->get_parameter("qp_max_iterations").as_int();
    qp_tolerance_ = get_node()->get_parameter("qp_tolerance").as_double();
    startLinkName_ = get_node()->get_parameter("start_link_name").as_string();
    endLinkName_ = get_node()->get_parameter("end_link_name").as_string();

    // Get tDiff_ parameters (relative transform between links)
    double tdiff_x = get_node()->get_parameter("tdiff_x").as_double();
    double tdiff_y = get_node()->get_parameter("tdiff_y").as_double();
    double tdiff_z = get_node()->get_parameter("tdiff_z").as_double();
    double tdiff_roll = get_node()->get_parameter("tdiff_roll").as_double();
    double tdiff_pitch = get_node()->get_parameter("tdiff_pitch").as_double();
    double tdiff_yaw = get_node()->get_parameter("tdiff_yaw").as_double();

    // Create the rotation matrix from roll, pitch, yaw
    Eigen::AngleAxisd rollAngle(M_PI * tdiff_roll / 180, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(M_PI * tdiff_pitch / 180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(M_PI * tdiff_yaw / 180, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
    Mat3d rotation = q.toRotationMatrix();

    // Create the translation vector
    Vec3d translation(tdiff_x, tdiff_y, tdiff_z);

    // Initialize tDiff_ using the rotation and translation
    tDiff_ = pin::SE3(rotation, translation);

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Initialized tDiff_ with x: %.3f, y: %.3f, z: %.3f, roll: %.3f, pitch: %.3f, yaw: %.3f",
        tdiff_x, tdiff_y, tdiff_z, tdiff_roll, tdiff_pitch, tdiff_yaw
    );

    // Get robot description topic
    robot_description_topic_ = get_node()->get_parameter("robot_description_topic").as_string();
    RCLCPP_INFO(get_node()->get_logger(), "Subscribing to robot description topic: %s", robot_description_topic_.c_str());

    // Subscribe to robot description topic
    robot_description_subscriber_ = get_node()->create_subscription<std_msgs::msg::String>(
        robot_description_topic_, rclcpp::QoS(1).transient_local(),
        [this](const std_msgs::msg::String::SharedPtr msg) { robot_description_callback(msg); }
    );

    // Create subscription for task space pose command
    pose_command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/command", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_command_callback(msg); }
    );

    // Create subscription for task space trajectory command
    trajectory_command_subscriber_ = get_node()->create_subscription<moveit_msgs::msg::CartesianTrajectory>(
        "~/trajectory", rclcpp::SystemDefaultsQoS(),
        [this](const moveit_msgs::msg::CartesianTrajectory::SharedPtr msg) { trajectory_command_callback(msg); }
    );

    // Create debug publisher
    debug_joints_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "~/debug/joints", rclcpp::SystemDefaultsQoS()
    );
    debug_cartesian_publisher_ = get_node()->create_publisher<moveit_msgs::msg::CartesianTrajectoryPoint>(
        "~/debug/cartesian", rclcpp::SystemDefaultsQoS()
    );

    // Wait for robot description with a timeout of 10 seconds
    if (!wait_for_robot_description(rclcpp::Duration::from_seconds(10))) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to receive robot description within timeout");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "YumiUdwadiaController configured successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void YumiUdwadiaController::robot_description_callback(const std_msgs::msg::String::SharedPtr msg) {
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

bool YumiUdwadiaController::wait_for_robot_description(const rclcpp::Duration& timeout) {
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
YumiUdwadiaController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // Check if we have a valid robot description
    if (!robot_description_received_) {
        RCLCPP_ERROR(get_node()->get_logger(), "No robot description received yet, cannot activate");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Reset task command buffer with identity pose and zero velocity/acceleration
    TaskSpaceCommand default_command;
    default_command.pose = pin::SE3::Identity();
    default_command.velocity = pin::Motion::Zero();
    default_command.acceleration = pin::Motion::Zero();

    // Get initial position from parameters
    double initial_x = get_node()->get_parameter("initial_position_x").as_double();
    double initial_y = get_node()->get_parameter("initial_position_y").as_double();
    double initial_z = get_node()->get_parameter("initial_position_z").as_double();

    // Get initial orientation from parameters (in degrees)
    double initial_roll = get_node()->get_parameter("initial_orientation_roll").as_double();
    double initial_pitch = get_node()->get_parameter("initial_orientation_pitch").as_double();
    double initial_yaw = get_node()->get_parameter("initial_orientation_yaw").as_double();

    // Create rotation matrix from roll, pitch, yaw
    Eigen::AngleAxisd rollAngle(M_PI * initial_roll / 180, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(M_PI * initial_pitch / 180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(M_PI * initial_yaw / 180, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
    Mat3d rotation = q.toRotationMatrix();

    // Set initial pose with position and orientation
    default_command.pose = pin::SE3(rotation, Vec3d(initial_x, initial_y, initial_z));

    task_command_buffer_.writeFromNonRT(default_command);

    // Reset trajectory buffer
    task_trajectory_buffer_.writeFromNonRT(std::vector<TaskTrajectoryPoint>{});
    has_trajectory_ = false;

    // Initialize state vectors
    q_ = pinocchio::neutral(model_);
    v_ = Eigen::VectorXd::Zero(model_.nv);
    a_ = Eigen::VectorXd::Zero(model_.nv);

    // Create a QP solver
    qp_solver_ = std::make_shared<prox::dense::QP<double>>(model_.nv, 6, 0);

    RCLCPP_INFO(get_node()->get_logger(), "YumiUdwadiaController activated successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
YumiUdwadiaController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // Clear commands
    task_command_buffer_.writeFromNonRT(TaskSpaceCommand{});
    task_trajectory_buffer_.writeFromNonRT(std::vector<TaskTrajectoryPoint>{});
    has_trajectory_ = false;

    RCLCPP_INFO(get_node()->get_logger(), "YumiUdwadiaController deactivated successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool YumiUdwadiaController::initialize_robot_model() {
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

        // Initialize state vectors
        q_ = pinocchio::neutral(model_);
        v_ = Eigen::VectorXd::Zero(model_.nv);
        a_ = Eigen::VectorXd::Zero(model_.nv);

        // Initialize the joint index maps
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            size_t pinocchio_index = model_.getJointId(joint_names_[i]);
            if (pinocchio_index >= model_.joints.size()) {
                RCLCPP_ERROR(get_node()->get_logger(), "Joint %s not found in the model", joint_names_[i].c_str());
                return false;
            }
            gazebo_indices_to_pin_q_idx_[i] = model_.joints[pinocchio_index].idx_q();
            gazebo_indices_to_pin_v_idx_[i] = model_.joints[pinocchio_index].idx_v();
        }

        // Initialize the indices of the start and end links
        indices_.startIdx = model_.getBodyId(startLinkName_);
        indices_.endIdx = model_.getBodyId(endLinkName_);

        if (indices_.startIdx == (int_idx)model_.nframes) {
            RCLCPP_ERROR(get_node()->get_logger(), "Start link %s not found in the model", startLinkName_.c_str());
            return false;
        }

        if (indices_.endIdx == (int_idx)model_.nframes) {
            RCLCPP_ERROR(get_node()->get_logger(), "End link %s not found in the model", endLinkName_.c_str());
            return false;
        }

        // Log dimensions of state vectors for debugging
        RCLCPP_INFO(get_node()->get_logger(), "State vector dimensions: q(%ld), v(%ld), a(%ld)", q_.size(), v_.size(), a_.size());

        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error loading model: %s", e.what());
        return false;
    }
}

VecXd YumiUdwadiaController::get_control_forces(pin::SE3& desPos, pin::Motion& desVel, pin::Motion& desAcc) {
    // perform forward kinematics
    forward_kin(q_, v_);

    // compute mass matrix
    MatXd massMat = get_mass_matrix();

    auto [aObjMat, bObjVec] = get_constraints_affine_desc();

    auto [aConMat, bConVec] = get_control_affine_desc(desPos, desVel, desAcc, indices_.startIdx);

    qp_solver_->settings.eps_abs = qp_tolerance_;
    qp_solver_->settings.max_iter = qp_max_iterations_;
    qp_solver_->settings.initial_guess =
        firstCall_ ? prox::InitialGuessStatus::NO_INITIAL_GUESS : prox::InitialGuessStatus::WARM_START;
    qp_solver_->settings.verbose = false;
    firstCall_ = false;

    qp_solver_->init(
        massMat + control_weight_ * aConMat.transpose() * aConMat,
        -biasForces_ - control_weight_ * aConMat.transpose() * bConVec,
        aObjMat,
        bObjVec,
        proxsuite::nullopt,
        proxsuite::nullopt,
        proxsuite::nullopt
    );
    qp_solver_->solve();

    if (qp_solver_->results.info.status != prox::QPSolverOutput::PROXQP_SOLVED) {
        std::ostringstream str;
        str << __FILE__ << ":" << __LINE__ << " QP cannot be solved!\n";
        std::cerr << str.str();
        std::abort();
    }
    // getting optimal acceleration
    VecXd res = pin::rnea(model_, model_data_, q_, v_, qp_solver_->results.x);

    return res;
}

std::pair<MatXd, VecXd> YumiUdwadiaController::get_constraints_affine_desc(
    bool recomputeForwardKin
) {
    if (recomputeForwardKin) {
        forward_kin(q_, v_);
    }

    // start jacobians
    MatXd jacStart = get_jacobian(indices_.startIdx, false);
    MatXd jacStartLin = jacStart.block(0, 0, 3, model_.nv);
    MatXd jacStartRot = jacStart.block(3, 0, 3, model_.nv);

    // start time differentiated jacobians
    MatXd dJacStart = get_jacobian_time_variation(indices_.startIdx, false);
    MatXd dJacStartLin = dJacStart.block(0, 0, 3, model_.nv);
    MatXd dJacStartRot = dJacStart.block(3, 0, 3, model_.nv);

    // end jacobian / time differentiated jacobian
    MatXd jacEnd = get_jacobian(indices_.endIdx, false);
    MatXd dJacEnd = get_jacobian_time_variation(indices_.endIdx, false);

    // start/end classic velocities
    auto startVel = get_classic_vel(indices_.startIdx, false);
    auto endVel = get_classic_vel(indices_.endIdx, false);

    // start/end attitude
    auto startPos = model_data_.oMf[indices_.startIdx];
    auto endPos = model_data_.oMf[indices_.endIdx];

    // useful quantities
    Vec3d pWorld = startPos.rotation() * tDiff_.translation();
    Mat3d skewPWorld = pin::skew(pWorld);
    Mat3d skewStartAng = pin::skew(startVel.angular());

    // object's end jacobian
    MatXd jacObj = MatXd::Zero(6, model_.nv);
    jacObj.block(0, 0, 3, model_.nv) = jacStartLin - skewPWorld * jacStartRot;
    jacObj.block(3, 0, 3, model_.nv) = jacStartRot;

    // object's end time differentiated jacobian
    MatXd dJacObj = MatXd::Zero(6, model_.nv);
    dJacObj.block(0, 0, 3, model_.nv) = dJacStartLin - skewPWorld * dJacStartRot - skewStartAng * skewPWorld * jacStartRot;
    dJacObj.block(3, 0, 3, model_.nv) = dJacStartRot;

    // differences
    MatXd jacDiff = jacEnd - jacObj;
    MatXd dJacDiff = dJacEnd - dJacObj;
    pin::Motion objVel(startVel.linear() + startVel.angular().cross(pWorld), startVel.angular());
    pin::SE3 objPos = startPos * tDiff_;

    return get_affine_desc(jacDiff, dJacDiff, objVel, endVel, objPos, endPos, constraint_kd_, constraint_kp_);
}

std::pair<MatXd, VecXd> YumiUdwadiaController::get_control_affine_desc(
    pin::SE3& desPos,
    pin::Motion& desVel,
    pin::Motion& desAcc,
    int_idx idx,
    bool recomputeForwardKin
) {
    if (recomputeForwardKin) {
        forward_kin(q_, v_);
    }

    // jacobian
    MatXd jac = -get_jacobian(idx, false);
    // time differentiated jacobian
    MatXd dJac = -get_jacobian_time_variation(idx, false);

    // velocity & position
    auto vel = get_classic_vel(idx, false);
    auto pos = model_data_.oMf[idx];

    auto [aMat, bVec] = get_affine_desc(jac, dJac, vel, desVel, pos, desPos, kd_, kp_);
    // subtract desired acceleration from b
    bVec.segment(0, 3) += -desAcc.linear();
    bVec.segment(3, 3) += desPos.rotation().transpose() * desAcc.angular();

    return {aMat, bVec};
}

std::pair<MatXd, VecXd> YumiUdwadiaController::get_affine_desc(
    MatXd& jacDiff,
    MatXd& dJacDiff,
    pin::Motion& vel,
    pin::Motion& velDes,
    pin::SE3& t,
    pin::SE3& tDes,
    double kD,
    double kP
) {
    // matrix / vector size
    int nc = 6;
    int nv = (int)v_.size();
    int rotShift = 3;

    // difference jacobians / time differentiated jacobians
    MatXd jacDiffLin = jacDiff.block(0, 0, 3, nv);
    MatXd jacDiffRot = jacDiff.block(3, 0, 3, nv);
    MatXd dJacDiffLin = dJacDiff.block(0, 0, 3, nv);
    MatXd dJacDiffRot = dJacDiff.block(3, 0, 3, nv);

    // attitude error
    VecXd err = error_in_se3(t, tDes);

    // useful quantities
    Mat3d skewDesAng = pin::skew(velDes.angular());
    MatXd jacDiffRotDes = tDes.rotation().transpose() * jacDiffRot;
    MatXd dJacDiffRotDes = tDes.rotation().transpose() * (dJacDiffRot - skewDesAng * jacDiffRot);

    // A matrix
    MatXd aMat = MatXd::Zero(nc, nv);
    aMat.block(0, 0, 3, nv) = jacDiffLin;
    aMat.block(rotShift, 0, 3, nv) = jacDiffRotDes;

    // b vector
    VecXd b = Vec6d::Zero(nc);
    b.segment(0, 3) += -dJacDiffLin * v_;
    b.segment(rotShift, 3) += -dJacDiffRotDes * v_;

    // derivative part
    b.segment(0, 3) += -kD * (velDes.linear() - vel.linear());
    b.segment(rotShift, 3) += -kD * tDes.rotation().transpose() * (velDes.angular() - vel.angular());

    // proportional part
    b += -kP * err;

    return {aMat, b};
}

Vec6d YumiUdwadiaController::error_in_se3(const pin::SE3& t, const pin::SE3& t_des) {
    Vec6d error = Vec6d::Zero();
    error.segment(0, 3) = t_des.translation() - t.translation();
    error.segment(3, 3) = pin::log3(t.rotation().transpose() * t_des.rotation());
    return error;
}

void YumiUdwadiaController::forward_kin(VecXd& q, VecXd& v, bool computeSecondDerivatives) {
    pin::forwardKinematics(model_, model_data_, q, v);
    pin::computeJointJacobians(model_, model_data_, q);
    if (computeSecondDerivatives) {
        pin::computeJointJacobiansTimeVariation(model_, model_data_, q, v);
    }
    pin::updateFramePlacements(model_, model_data_);
}

void YumiUdwadiaController::forward_kin(VecXd& q) {
    pin::forwardKinematics(model_, model_data_, q);
    pin::computeJointJacobians(model_, model_data_, q);
    pin::updateFramePlacements(model_, model_data_);
}

MatXd YumiUdwadiaController::get_mass_matrix() {
    MatXd massMatrix = pin::crba(model_, model_data_, q_);
    massMatrix.triangularView<Eigen::StrictlyLower>() = massMatrix.transpose().triangularView<Eigen::StrictlyLower>();
    return massMatrix;
}

MatXd YumiUdwadiaController::get_jacobian(int_idx idx, bool recomputeJacs) {
    if (recomputeJacs) {
        forward_kin(q_);
    }
    MatXd jac = MatXd::Zero(6, model_.nv);
    pin::getFrameJacobian(model_, model_data_, idx, pin::LOCAL_WORLD_ALIGNED, jac);
    return jac;
}

MatXd YumiUdwadiaController::get_jacobian_time_variation(int_idx idx, bool recomputeJacs) {
    if (recomputeJacs) {
        forward_kin(q_, v_);
    }
    MatXd dJac = MatXd::Zero(6, model_.nv);
    pin::getFrameJacobianTimeVariation(model_, model_data_, idx, pin::LOCAL_WORLD_ALIGNED, dJac);
    return dJac;
}

pin::Motion YumiUdwadiaController::get_classic_vel(int_idx idx, bool recomputeForwardKin) {
    if (recomputeForwardKin) {
        forward_kin(q_, v_, false);
    }
    auto frame = model_.frames[idx];
    auto parent_idx = frame.parent;
    auto local_to_world_t = pin::SE3::Identity();

    local_to_world_t.rotation(model_data_.oMf[idx].rotation());
    return local_to_world_t.act(frame.placement.actInv(model_data_.v[parent_idx]));
}

controller_interface::return_type
YumiUdwadiaController::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
    // Read current joint states from hardware interfaces
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        // Get joint position and velocity from state interfaces
        q_(gazebo_indices_to_pin_q_idx_[i]) = state_interfaces_[i * 2].get_value();      // Position interface
        v_(gazebo_indices_to_pin_v_idx_[i]) = state_interfaces_[i * 2 + 1].get_value();  // Velocity interface
    }

    // Initialize desired task space state
    pin::SE3 desired_pose;
    pin::Motion desired_velocity;
    pin::Motion desired_acceleration;

    // Check if we have a trajectory to follow
    if (has_trajectory_ && interpolate_task_trajectory(time, desired_pose, desired_velocity, desired_acceleration)) {
        // Use interpolated trajectory values
        // These are already set in the desired_pose, desired_velocity, desired_acceleration
    } else {
        // If no trajectory or finished trajectory, use pose commands if available
        auto task_command = *task_command_buffer_.readFromRT();
        desired_pose = task_command.pose;
        desired_velocity = task_command.velocity;
        desired_acceleration = task_command.acceleration;

        // No active trajectory anymore
        has_trajectory_ = false;
    }

    // Compute bias forces (gravity, Coriolis, etc.) using RNEA with zero acceleration
    biasForces_ = -pin::rnea(model_, model_data_, q_, v_, VecXd::Zero(model_.nv));

    // Compute control forces using getControlForces
    auto start_time = std::chrono::high_resolution_clock::now();
    Eigen::VectorXd tau = get_control_forces(desired_pose, desired_velocity, desired_acceleration);
    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    // Apply torque commands to the joints
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces_[i].set_value(tau(gazebo_indices_to_pin_v_idx_[i]));
    }

    // Publish debug info
    auto debug_joints_msg = std::make_unique<sensor_msgs::msg::JointState>();
    debug_joints_msg->header.stamp = time;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        debug_joints_msg->name.push_back(joint_names_[i]);
        debug_joints_msg->position.push_back(q_(gazebo_indices_to_pin_q_idx_[i]));
        debug_joints_msg->velocity.push_back(v_(gazebo_indices_to_pin_v_idx_[i]));
        debug_joints_msg->effort.push_back(tau(gazebo_indices_to_pin_v_idx_[i]));
    }
    debug_joints_publisher_->publish(std::move(debug_joints_msg));

    // Publish cartesian debug info
    auto debug_cartesian_msg = std::make_unique<moveit_msgs::msg::CartesianTrajectoryPoint>();
    debug_cartesian_msg->time_from_start = rclcpp::Duration(std::chrono::nanoseconds(time.nanoseconds()));
    debug_cartesian_msg->point.pose = se3_to_pose_msg(model_data_.oMf[indices_.startIdx]);
    debug_cartesian_msg->point.velocity = motion_to_twist_msg(get_classic_vel(indices_.startIdx, false));
    debug_cartesian_publisher_->publish(std::move(debug_cartesian_msg));

    // Log performance info periodically
    RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Control forces computation time: %.3f ms",
        elapsed_time_us / 1000.0
    );

    return controller_interface::return_type::OK;
}

void YumiUdwadiaController::pose_command_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg
) {
    // Create a new task space command
    TaskSpaceCommand command;

    // Convert the pose message to SE3
    command.pose = pose_msg_to_se3(msg->pose);

    // Set zero velocity and acceleration for a static pose command
    command.velocity = pin::Motion::Zero();
    command.acceleration = pin::Motion::Zero();

    // Clear any active trajectory when receiving direct pose commands
    has_trajectory_ = false;

    // Write command to buffer to be used in the real-time update loop
    task_command_buffer_.writeFromNonRT(command);

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Received pose command: position [%.2f, %.2f, %.2f]",
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z
    );
}

void YumiUdwadiaController::trajectory_command_callback(
    const moveit_msgs::msg::CartesianTrajectory::SharedPtr msg
) {
    if (msg->points.empty()) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *(get_node()->get_clock()),
            1000,
            "Received empty trajectory"
        );
        return;
    }

    // Convert joint space trajectory to task space trajectory using forward kinematics
    std::vector<TaskTrajectoryPoint> task_trajectory_points;

    for (const auto& point : msg->points) {
        // Create task space trajectory point
        TaskTrajectoryPoint tp;
        tp.time_from_start = rclcpp::Duration(point.time_from_start.sec, point.time_from_start.nanosec);

        tp.pose = pose_msg_to_se3(point.point.pose);
        tp.velocity = twist_msg_to_motion(point.point.velocity);
        tp.acceleration = accel_msg_to_motion(point.point.acceleration);

        task_trajectory_points.push_back(tp);
    }

    // Set the trajectory start time and write to RT buffer
    trajectory_start_time_ = get_node()->get_clock()->now();
    task_trajectory_buffer_.writeFromNonRT(task_trajectory_points);
    has_trajectory_ = true;

    RCLCPP_INFO(
        get_node()->get_logger(),
        "Received and converted trajectory with %zu points, duration: %.2f seconds",
        task_trajectory_points.size(),
        task_trajectory_points.back().time_from_start.seconds()
    );
}

bool YumiUdwadiaController::interpolate_task_trajectory(
    const rclcpp::Time& current_time,
    pin::SE3& pose,
    pin::Motion& velocity,
    pin::Motion& acceleration
) {
    auto trajectory_points = *task_trajectory_buffer_.readFromRT();

    if (trajectory_points.empty()) {
        return false;
    }

    // Calculate the time since the trajectory started
    rclcpp::Duration time_since_start = current_time - trajectory_start_time_;

    // Find the trajectory segments to interpolate between
    size_t next_idx = 0;
    while (next_idx < trajectory_points.size() &&
           trajectory_points[next_idx].time_from_start.nanoseconds() <= time_since_start.nanoseconds()) {
        next_idx++;
    }

    // Check if we're past the end of the trajectory
    if (next_idx == 0) {
        // We're before the first point, use the first point
        pose = trajectory_points[0].pose;
        velocity = trajectory_points[0].velocity;
        acceleration = trajectory_points[0].acceleration;
        return true;
    } else if (next_idx >= trajectory_points.size()) {
        // We're after the last point, use the last point
        const auto& last_point = trajectory_points.back();
        pose = last_point.pose;
        velocity = pin::Motion::Zero();      // Zero velocity at end
        acceleration = pin::Motion::Zero();  // Zero acceleration at end

        // If we're significantly past the trajectory end, return false to indicate we're done
        if (time_since_start.nanoseconds() > last_point.time_from_start.nanoseconds() + rclcpp::Duration::from_seconds(0.5).nanoseconds()) {
            return false;
        }
        return true;
    }

    // Interpolate between two points
    size_t prev_idx = next_idx - 1;
    const auto& prev_point = trajectory_points[prev_idx];
    const auto& next_point = trajectory_points[next_idx];

    // Calculate the interpolation factor (0.0 to 1.0)
    double t_prev = prev_point.time_from_start.seconds();
    double t_next = next_point.time_from_start.seconds();
    double t_curr = time_since_start.seconds();
    double alpha = (t_curr - t_prev) / (t_next - t_prev);

    // Interpolate position (SE3 interpolation)
    pose = pin::SE3::Interpolate(prev_point.pose, next_point.pose, alpha);

    // Linear interpolation for velocity
    velocity.linear() = prev_point.velocity.linear() + alpha * (next_point.velocity.linear() - prev_point.velocity.linear());
    velocity.angular() = prev_point.velocity.angular() + alpha * (next_point.velocity.angular() - prev_point.velocity.angular());

    // Linear interpolation for acceleration
    acceleration.linear() = prev_point.acceleration.linear() + alpha * (next_point.acceleration.linear() - prev_point.acceleration.linear());
    acceleration.angular() = prev_point.acceleration.angular() + alpha * (next_point.acceleration.angular() - prev_point.acceleration.angular());

    return true;
}

}  // namespace yumi_control

// Export controller as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(yumi_control::YumiUdwadiaController, controller_interface::ControllerInterface)