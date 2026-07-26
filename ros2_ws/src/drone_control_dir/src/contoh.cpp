#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Drone 3 (follower2) is assumed to fly its takeoff leg this far above
// Drone 1's takeoff target altitude.
const float DRONE3_ALT_OFFSET = 5.0f;

using namespace std::chrono_literals;

class Drone1Control : public rclcpp::Node {
public:
    Drone1Control();

private:
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg);
    void offboard_control_mode();
    void arm();
    void set_offboard_command();
    void trajectory_logic();
    void follower1_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr fol1_odom_msg);
    void follower2_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr fol2_odom_msg);

    Eigen::Vector3f collision_logic(const Eigen::Vector3f &current_pos,
        const Eigen::Vector3f &fol1_pos,
        const Eigen::Vector3f &fol2_pos);

    Eigen::Vector3f step_logic(const Eigen::Vector3f &current_pos, const Eigen::Vector3f &target_pos);

    // Rotates a relative BODY-frame displacement (FRD: x-forward, y-right, z-down)
    // into the world/NED frame using the given yaw. Yaw rotation does not affect z.
    Eigen::Vector3f body_to_world(const Eigen::Vector3f &body_disp, float yaw) const;

    bool wait_for_settle(const Eigen::Vector3f &fol2_pos);
    bool waypoint_reached(const Eigen::Vector3f &odom_pos, const Eigen::Vector3f &target);
    void start_run();

    // publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    // subscriber and timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_own_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr follower1_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr follower2_sub;

    enum class OffboardState {
        INIT,
        OFFBOARD,
        ARMED
    };
    OffboardState state_{OffboardState::INIT};

    float current_yaw{0.0f};

    // Waypoints are RELATIVE BODY-FRAME displacements (FRD: x-forward, y-right, z-down),
    // applied one after another. Each is converted to an absolute world/NED target using
    // the yaw and position captured at the start of that segment.
    std::vector<Eigen::Vector3f> input_waypoint;
    size_t current_wp_idx_{0};

    // per-segment absolute target bookkeeping
    Eigen::Vector3f segment_origin_{0.0f, 0.0f, 0.0f};
    float segment_origin_yaw_{0.0f};
    Eigen::Vector3f current_target_world_{0.0f, 0.0f, 0.0f};
    bool target_computed_{false};

    // hold-at-waypoint bookkeeping
    bool wp_hold_started_{false};
    rclcpp::Time wp_hold_start_time_;
    static constexpr double kHoldSeconds = 5.0;

    // pre-offboard setpoint streaming bookkeeping (PX4 requires a setpoint stream
    // before it will accept an OFFBOARD mode switch)
    int setpoint_counter_{0};
    bool initialized_pos{false};
    Eigen::Vector3f init_global_position_3d{0.0f, 0.0f, 0.0f};

    bool odom_received_{false};

    // constants for the collision potential field V_ij
    double d{5.0}, h{1.0};
    double c = pow(d, 2) / 3;
    double a = 4 * h * pow(c, 3);
    static constexpr float kMaxAvoidDisp = 1.0f; // m, safety clamp on avoidance displacement per tick

    // follower / own odometry
    Eigen::Vector3f fol1_pos{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f fol2_pos{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f odom_pos{0.0f, 0.0f, 0.0f};
};

Drone1Control::Drone1Control(): Node("drone1_control_node")
{
    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/px4_1/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/px4_1/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/px4_1/fmu/in/vehicle_command", 10);

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .durability_volatile();

    odom_own_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/px4_1/fmu/out/vehicle_odometry", qos_profile,
        std::bind(&Drone1Control::odom_callback, this, std::placeholders::_1));

    follower1_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/px4_2/fmu/out/vehicle_odometry", qos_profile,
        std::bind(&Drone1Control::follower1_odom_callback, this, std::placeholders::_1));

    follower2_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/px4_3/fmu/out/vehicle_odometry", qos_profile,
        std::bind(&Drone1Control::follower2_odom_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(100ms, std::bind(&Drone1Control::start_run, this));

    // RELATIVE BODY-FRAME displacements: x = forward, y = right, z = down.
    // z = -5.0 on the first entry means "climb 5 m" (up is negative in NED/FRD).
    input_waypoint.reserve(20);
    input_waypoint.emplace_back(0.0f, 0.0f, -5.0f); // takeoff: climb 5 m
    input_waypoint.emplace_back(0.0f, 20.0f, 0.0f); // side-step 20 m
    input_waypoint.emplace_back(5.0f, 0.0f, 0.0f);  // forward 5 m
    input_waypoint.emplace_back(0.0f, -20.0f, 0.0f);
    input_waypoint.emplace_back(5.0f, 0.0f, 0.0f);
    input_waypoint.emplace_back(0.0f, 20.0f, 0.0f);
    input_waypoint.emplace_back(5.0f, 0.0f, 0.0f);
    input_waypoint.emplace_back(0.0f, -20.0f, 0.0f);
    input_waypoint.emplace_back(5.0f, 0.0f, 0.0f);
    input_waypoint.emplace_back(0.0f, 20.0f, 0.0f);
    input_waypoint.emplace_back(0.0f, 0.0f, 5.0f);  // land: descend 5 m
}

void Drone1Control::arm() {
    px4_msgs::msg::VehicleCommand arm_msg{};
    arm_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    arm_msg.param1 = 1.0;
    arm_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    arm_msg.target_system = 2;
    arm_msg.target_component = 1;
    arm_msg.source_system = 1;
    arm_msg.source_component = 1;
    vehicle_command_pub_->publish(arm_msg);
    RCLCPP_INFO(this->get_logger(), "Arm command sent (drone 1)");
}

void Drone1Control::set_offboard_command() {
    px4_msgs::msg::VehicleCommand vehicle_mode_msg{};
    vehicle_mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_mode_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    vehicle_mode_msg.param1 = 1.0;
    vehicle_mode_msg.param2 = 6.0;
    vehicle_mode_msg.target_system = 2;
    vehicle_mode_msg.target_component = 1;
    vehicle_mode_msg.source_system = 1;
    vehicle_mode_msg.source_component = 1;
    vehicle_command_pub_->publish(vehicle_mode_msg);
    RCLCPP_INFO(this->get_logger(), "Offboard mode command sent (drone 1)");
}

void Drone1Control::offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode offboard_msg{};
    offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_msg.position = true;
    offboard_msg.velocity = false;
    offboard_msg.acceleration = false;
    offboard_msg.attitude = false;
    offboard_msg.body_rate = false;
    offboard_control_mode_pub_->publish(offboard_msg);
}

void Drone1Control::follower1_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr fol1_odom_msg) {
    fol1_pos << fol1_odom_msg->position[0], fol1_odom_msg->position[1], fol1_odom_msg->position[2];
}

void Drone1Control::follower2_odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr fol2_odom_msg) {
    fol2_pos << fol2_odom_msg->position[0], fol2_odom_msg->position[1], fol2_odom_msg->position[2];
}

void Drone1Control::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg)
{
    float qw = odom_msg->q[0];
    float qx = odom_msg->q[1];
    float qy = odom_msg->q[2];
    float qz = odom_msg->q[3];

    current_yaw = std::atan2(
        2.0f * (qw * qz + qx * qy),
        1.0f - 2.0f * (qy * qy + qz * qz)
    );

    odom_pos << odom_msg->position[0], odom_msg->position[1], odom_msg->position[2];

    odom_received_ = true;
}

Eigen::Vector3f Drone1Control::body_to_world(const Eigen::Vector3f &body_disp, float yaw) const
{
    Eigen::Vector3f world_disp;
    world_disp.x() = std::cos(yaw) * body_disp.x() - std::sin(yaw) * body_disp.y();
    world_disp.y() = std::sin(yaw) * body_disp.x() + std::cos(yaw) * body_disp.y();
    world_disp.z() = body_disp.z(); // pure yaw rotation does not touch the vertical axis
    return world_disp;
}

Eigen::Vector3f Drone1Control::collision_logic(const Eigen::Vector3f &current_pos,
    const Eigen::Vector3f &fol1_pos,
    const Eigen::Vector3f &fol2_pos)
{
    std::vector<Eigen::Vector3f> fol_list = {fol1_pos, fol2_pos};
    Eigen::Vector3f total_avoid = Eigen::Vector3f::Zero();

    for (size_t i = 0; i < fol_list.size(); i++) {
        Eigen::Vector3f q_sub = current_pos - fol_list[i];
        float beta = q_sub.squaredNorm();

        if (beta < 1e-6f) {
            continue; // coincident with follower, skip to avoid a div-by-zero blow-up
        }

        float rho = 0.0f;
        if (beta < static_cast<float>(c)) {
            rho = -static_cast<float>(2.0 * a) / (beta * beta);
        } else if (beta < static_cast<float>(pow(d, 2))) {
            rho = -4.0f * static_cast<float>(h) * (beta - static_cast<float>(pow(d, 2)));
        }
        // beta >= d^2 -> outside the influence radius, rho stays 0 (no contribution)

        total_avoid += rho * (-q_sub);
    }

    return total_avoid;
}

// Only meaningful right after takeoff (waypoint 0): checks whether Drone 3
// (follower2) has settled at its takeoff altitude before the formation moves on.
bool Drone1Control::wait_for_settle(const Eigen::Vector3f &fol2_pos)
{
    float target_alt = current_target_world_.z() + DRONE3_ALT_OFFSET;
    return std::abs(fol2_pos.z() - target_alt) < 0.2f;
}

bool Drone1Control::waypoint_reached(const Eigen::Vector3f &odom_pos, const Eigen::Vector3f &target)
{
    float distance_selisih = (odom_pos - target).norm();
    return (distance_selisih < 0.2f);
}

Eigen::Vector3f Drone1Control::step_logic(const Eigen::Vector3f &current_pos, const Eigen::Vector3f &target_pos)
{
    Eigen::Vector3f diff = target_pos - current_pos;
    float dist = diff.norm();

    if (dist <= 0.2f || dist < 1e-3f) {
        return target_pos;
    }
    return current_pos + diff.normalized() * 0.3f;
}

void Drone1Control::trajectory_logic()
{
    // Mission complete: hold last position indefinitely.
    if (current_wp_idx_ >= input_waypoint.size()) {
        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        traj.position = {odom_pos.x(), odom_pos.y(), odom_pos.z()};
        traj.yaw = current_yaw;
        trajectory_setpoint_pub_->publish(traj);
        return;
    }

    // Compute the absolute (world/NED) target for this segment exactly once, by
    // rotating the relative BODY-frame waypoint displacement with the yaw captured
    // at the moment the segment starts.
    if (!target_computed_) {
        segment_origin_ = odom_pos;
        segment_origin_yaw_ = current_yaw;
        Eigen::Vector3f world_disp = body_to_world(input_waypoint[current_wp_idx_], segment_origin_yaw_);
        current_target_world_ = segment_origin_ + world_disp;
        target_computed_ = true;
    }

    if (!waypoint_reached(odom_pos, current_target_world_)) {
        // Move toward target, blending in a collision-avoidance displacement.
        Eigen::Vector3f avoid = collision_logic(odom_pos, fol1_pos, fol2_pos);
        float avoid_norm = avoid.norm();
        if (avoid_norm > kMaxAvoidDisp && avoid_norm > 1e-6f) {
            avoid *= (kMaxAvoidDisp / avoid_norm); // safety clamp
        }

        Eigen::Vector3f wp = step_logic(odom_pos, current_target_world_) + avoid;

        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        traj.position = {wp.x(), wp.y(), wp.z()};
        traj.yaw = current_yaw;
        trajectory_setpoint_pub_->publish(traj);
        return;
    }

    // Target reached: hold position for kHoldSeconds.
    if (!wp_hold_started_) {
        wp_hold_start_time_ = this->get_clock()->now();
        wp_hold_started_ = true;
    }

    px4_msgs::msg::TrajectorySetpoint traj{};
    traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    traj.position = {current_target_world_.x(), current_target_world_.y(), current_target_world_.z()};
    traj.yaw = current_yaw;
    trajectory_setpoint_pub_->publish(traj);

    double elapsed = (this->get_clock()->now() - wp_hold_start_time_).seconds();
    bool hold_done = elapsed >= kHoldSeconds;

    // Drone 1 only waits for Drone 3 (follower2) right after takeoff (waypoint 0),
    // to let it settle at its takeoff altitude before the formation starts moving.
    bool drone3_ready = (current_wp_idx_ != 0) || wait_for_settle(fol2_pos);

    if (hold_done && drone3_ready) {
        current_wp_idx_++;
        target_computed_ = false;
        wp_hold_started_ = false;
    }
}

void Drone1Control::start_run(){
    offboard_control_mode();

    if (setpoint_counter_ < 10) {
        if (!odom_received_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting for vehicle_odometry (drone 1)...");
            return;
        }

        // Stream current position as the setpoint; PX4 requires a setpoint
        // stream before it will accept an OFFBOARD mode switch.
        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        traj.position = {odom_pos.x(), odom_pos.y(), odom_pos.z()};
        traj.yaw = current_yaw;
        trajectory_setpoint_pub_->publish(traj);
        setpoint_counter_++;

        if (!initialized_pos) {
            init_global_position_3d = odom_pos;
            initialized_pos = true;

            RCLCPP_INFO(this->get_logger(),
                "Initial position latched | current = [%.3f, %.3f, %.3f]",
                odom_pos.x(), odom_pos.y(), odom_pos.z());
        }
        return;
    }

    if (state_ == OffboardState::INIT) {
        RCLCPP_INFO(this->get_logger(), "Setting offboard mode (drone 1)...");
        set_offboard_command();
        state_ = OffboardState::OFFBOARD;
        return;
    }

    if (state_ == OffboardState::OFFBOARD) {
        RCLCPP_INFO(this->get_logger(), "Arming (drone 1)...");
        arm();
        state_ = OffboardState::ARMED;
        return;
    }

    offboard_control_mode();
    trajectory_logic();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Drone1Control>());
    rclcpp::shutdown();
    return 0;
}