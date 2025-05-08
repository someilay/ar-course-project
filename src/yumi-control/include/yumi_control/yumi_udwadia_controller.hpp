#ifndef YUMI_CONTROL__YUMI_UDWADIA_CONTROLLER_HPP_
#define YUMI_CONTROL__YUMI_UDWADIA_CONTROLLER_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "moveit_msgs/msg/cartesian_trajectory.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

// Pinocchio includes
#include <eigen3/Eigen/Dense>

#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"

// Proxsuite includes
#include "proxsuite/proxqp/dense/dense.hpp"

namespace yumi_control {

namespace pin = pinocchio;
namespace prox = proxsuite::proxqp;

// Eigen type aliases for common vector/matrix dimensions
using VecXd = Eigen::VectorXd;
using Vec3d = Eigen::Vector3d;
using Vec4d = Eigen::Vector4d;
using Vec6d = Eigen::Matrix<double, 6, 1>;

using MatXd = Eigen::MatrixXd;
using Mat3d = Eigen::Matrix3d;

using int_idx = pin::JointIndex;

// Joint index range struct
struct _indices {
    int_idx startIdx;
    int_idx endIdx;
} typedef Indices;

// Task space command for pose, velocity, and acceleration
struct _taskSpaceCommand {
    pin::SE3 pose;
    pin::Motion velocity;
    pin::Motion acceleration;
} typedef TaskSpaceCommand;

// Single point in a task space trajectory
struct _taskTrajectoryPoint {
    pin::SE3 pose;
    pin::Motion velocity;
    pin::Motion acceleration;
    rclcpp::Duration time_from_start = rclcpp::Duration(0, 0);
} typedef TaskTrajectoryPoint;

/**
 * Controller implementing Udwadia's formulation for task space control of YuMi robot
 */
class YumiUdwadiaController : public controller_interface::ControllerInterface {
   public:
    YumiUdwadiaController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state
    ) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state
    ) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state
    ) override;

   private:
    // Initialize the Pinocchio model from robot_description parameter
    bool initialize_robot_model();

    // Callback for robot description topic
    void robot_description_callback(const std_msgs::msg::String::SharedPtr msg);

    // Wait for robot description with timeout
    bool wait_for_robot_description(const rclcpp::Duration &timeout);

    // Compute control forces using QP optimization
    VecXd get_control_forces(
        pin::SE3 &desPos,
        pin::Motion &desVel,
        pin::Motion &desAcc
    );

    // Get affine description of task constraint (A*qddot = b)
    std::pair<MatXd, VecXd> get_affine_desc(
        MatXd &jacDiff,
        MatXd &dJacDiff,
        pin::Motion &vel,
        pin::Motion &velDes,
        pin::SE3 &t,
        pin::SE3 &tDes,
        double kD,
        double kP
    );

    // Get affine description for joint/link constraints
    std::pair<MatXd, VecXd> get_constraints_affine_desc(
        bool recomputeForwardKin = true
    );

    // Get affine description for control task
    std::pair<MatXd, VecXd> get_control_affine_desc(
        pin::SE3 &desPos,
        pin::Motion &desVel,
        pin::Motion &desAcc,
        int_idx idx,
        bool recomputeForwardKin = true
    );

    // Compute error between current and desired pose in SE3
    Vec6d error_in_se3(const pin::SE3 &t, const pin::SE3 &t_des);

    // Forward kinematics with first and optionally second derivatives
    void forward_kin(VecXd &q, VecXd &v, bool computeSecondDerivatives = true);

    // Forward kinematics with position only
    void forward_kin(VecXd &q);

    // Get mass matrix from model
    MatXd get_mass_matrix();

    // Get Jacobian for specified frame/link
    MatXd get_jacobian(int_idx idx, bool recomputeJacs = true);

    // Get time derivative of Jacobian
    MatXd get_jacobian_time_variation(int_idx idx, bool recomputeJacs = true);

    // Get classical velocity for specified frame/link
    pin::Motion get_classic_vel(
        int_idx idx, bool recomputeForwardKin = true
    );

    // Subscription to task space pose command
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_command_subscriber_;

    // Subscription to task space trajectory command
    rclcpp::Subscription<moveit_msgs::msg::CartesianTrajectory>::SharedPtr trajectory_command_subscriber_;

    // Subscription to the robot description topic
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;

    // Buffer for task space desired pose
    realtime_tools::RealtimeBuffer<TaskSpaceCommand> task_command_buffer_;

    // Buffer for task space trajectory
    realtime_tools::RealtimeBuffer<std::vector<TaskTrajectoryPoint>> task_trajectory_buffer_;
    rclcpp::Time trajectory_start_time_;
    bool has_trajectory_ = false;

    // Joint names from parameters
    std::vector<std::string> joint_names_;

    // Command message callbacks
    void pose_command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void trajectory_command_callback(const moveit_msgs::msg::CartesianTrajectory::SharedPtr msg);

    // Trajectory interpolation
    bool interpolate_task_trajectory(const rclcpp::Time &current_time, pin::SE3 &pose, pin::Motion &velocity, pin::Motion &acceleration);

    // Convert between Pinocchio and ROS message types
    pin::SE3 pose_msg_to_se3(const geometry_msgs::msg::Pose &pose_msg);
    geometry_msgs::msg::Pose se3_to_pose_msg(const pin::SE3 &se3);
    pin::Motion twist_msg_to_motion(const geometry_msgs::msg::Twist &twist_msg);
    geometry_msgs::msg::Twist motion_to_twist_msg(const pin::Motion &motion);
    pin::Motion accel_msg_to_motion(const geometry_msgs::msg::Accel &accel_msg);

    // Pinocchio model and data
    pin::Model model_;
    pin::Data model_data_;
    VecXd q_;           // Joint positions
    VecXd v_;           // Joint velocities
    VecXd a_;           // Joint accelerations
    VecXd biasForces_;  // Bias forces

    // Debug publisher for model state
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr debug_joints_publisher_;
    rclcpp::Publisher<moveit_msgs::msg::CartesianTrajectoryPoint>::SharedPtr debug_cartesian_publisher_;

    // Parameters
    std::string robot_description_;
    std::string robot_description_topic_;
    double kp_ = 100.0;               // Position gain
    double kd_ = 10.0;                // Velocity gain
    double constraint_kp_ = 100.0;    // Constraint position gain (for end-effector constraints)
    double constraint_kd_ = 10.0;     // Constraint velocity gain
    double control_weight_ = 1.0;     // Weight for the control cost in QP

    // Flag to track if robot description has been received
    bool robot_description_received_ = false;

    // Mappings from controller joint indices to Pinocchio model indices
    std::unordered_map<size_t, int> gazebo_indices_to_pin_q_idx_;
    std::unordered_map<size_t, int> gazebo_indices_to_pin_v_idx_;

    // QP solver for computing optimal control forces
    std::shared_ptr<prox::dense::QP<double>> qp_solver_;

    // QP solver parameters
    int qp_max_iterations_ = 100;
    double qp_tolerance_ = 1e-4;
    bool firstCall_ = false;

    // Connected links indices
    Indices indices_;
    std::string startLinkName_;
    std::string endLinkName_;

    // Connected links relative pose
    pin::SE3 tDiff_;
};

}  // namespace yumi_control

#endif  // YUMI_CONTROL__YUMI_UDWADIA_CONTROLLER_HPP_