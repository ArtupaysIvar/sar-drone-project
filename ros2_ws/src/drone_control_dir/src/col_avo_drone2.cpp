#include <rclcpp/rclcpp.hpp>
// #include <lib/mathlib/mathlib.h>
// #include <lib/geo/geo.h>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

class Drone2Control : public rclcpp::Node
{
public:
    Drone2Control();

private:
    // functions:
    // logic
    void start_run();
    void trajectory_logic();
    Eigen::Vector3f collision_logic();

    // callbacks
    void own_gps_to_ned(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr odom_own_msg);
    void lead_gps_to_ned(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr odom_lead_msg);
    void fol_gps_to_ned(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr odom_fol_msg);
    void own_odom_func(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_own_msg);

    // inherent
    void offboard_control_mode();
    void arm();
    void set_offboard_command();


    // Publishers and subscribers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr fol_gps_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr lead_gps_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr own_gps_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr own_odom_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    // state machine for arming
    enum class OffboardState
    {
        INIT,
        OFFBOARD,
        ARMED
    };
    OffboardState state{OffboardState::INIT};
    
    // variables:

    // references
    double ref_lat{0}, ref_lon{0}, ref_alt{0};
    Eigen::Vector3f desired_own_pos;
    Eigen::Vector3f desired_lead_pos;

    // odom callbacks
    Eigen::Vector3f lead_odom_pos;
    Eigen::Vector3f fol_odom_pos;
    Eigen::Vector3f own_gps_to_odom;
    Eigen::Vector3f own_odom_pos;

    bool gps_own_received {false};
    bool gps_fol_received {false};
    bool gps_lead_received {false};
    bool odom_own_received {false};

    // from collision_logic
    Eigen::Vector3f avoid_input;
    double d{5.0}, h{1.0};
    double c = pow(d,2)/3;
    double a = 4*h*pow(c,3);  

    int setpoint_counter_{0};
    bool odom_received_ = false;

    Eigen::Vector3f formation_input;
    Eigen::Vector3f collision_input;
    Eigen::Vector3f control_input;

    float current_yaw{0.0};

    bool z_sync_phase{true};
    float z_tolerance{0.3f};

    float step_gain{0.5};

    // Eigen::Vector3f px4_target{Eigen::Vector3f::Zero()};
};

Drone2Control::Drone2Control() : Node("displacement_control_drone2")
{
    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/px4_2/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/px4_2/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/px4_2/fmu/in/vehicle_command", 10);

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                           .best_effort()
                           .durability_volatile();

    lead_gps_sub = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/px4_1/fmu/out/vehicle_global_position",
        qos_profile,
        std::bind(&Drone2Control::lead_gps_to_ned, this, std::placeholders::_1));

    fol_gps_sub = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/px4_3/fmu/out/vehicle_global_position",
        qos_profile,
        std::bind(&Drone2Control::fol_gps_to_ned, this, std::placeholders::_1));

    own_gps_sub = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/px4_2/fmu/out/vehicle_global_position",
        qos_profile,
        std::bind(&Drone2Control::own_gps_to_ned, this, std::placeholders::_1));


    own_odom_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/px4_2/fmu/out/vehicle_odometry",
        qos_profile,
        std::bind(&Drone2Control::own_odom_sub, this, std::placeholders::_1));

    // Timer to send setpoints periodically
    timer_ = create_wall_timer(100ms, std::bind(&Drone2Control::start_run, this));

    desired_own_pos << -2.0f, -3.464f, 0.0f;
    desired_lead_pos << 0.0f, 0.0f, 0.0f;
}

Eigen::Vector3f Drone2Control::collision_logic()
{

    std::vector<Eigen::Vector3f> fol_list = {fol_odom_pos, lead_odom_pos};

    for (auto i = 0; i < fol_list.size(); i++)
    {
        float beta = (own_odom_pos - fol_list[i]).squaredNorm();
        // float beta_dist = (current_pos - fol_list[i]).norm();
        Eigen::Vector3f q_sub = own_odom_pos - fol_list[i];

        if (beta < c)
        {
            float rho = -(2 * a) / pow(beta, 2);
            return avoid_input += rho * (-q_sub);
        }
        else if (c <= beta && beta < pow(d, 2))
        {
            float rho = -4 * h * (beta - pow(d, 2));
            return avoid_input += rho * (-q_sub);
        }
        else
        {
            avoid_input << 0.0, 0.0, 0.0;
            return avoid_input;
        }
    }
    return Eigen::Vector3f::Zero();
}

Eigen::Vector3f gps_to_ned(const Eigen::Vector3d gps_msg){

    const double R = 6371000.0; // earth radius meters
    float north = static_cast<float>(gps_msg.x() * (M_PI/180.0) * R);
    float east  = static_cast<float>(gps_msg.y() * (M_PI/180.0) * R);
    float down  = static_cast<float>(-gps_msg.z());
    
    Eigen::Vector3f ned_coor = {north, east, down};
    
    return ned_coor;
}

void  Drone2Control::lead_gps_to_ned(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr lead_gps_msg)
{
    // Eigen::Quaternionf lead_orientation = {odom_lead_msg -> q[0], odom_lead_msg -> q[1], 
    //     odom_lead_msg -> q[2], odom_lead_msg -> q[3]};
    
    Eigen::Vector3d lead_gps_pos = {lead_gps_msg -> lat, lead_gps_msg -> lon, static_cast<double>(lead_gps_msg -> alt)};

    lead_odom_pos << gps_to_ned(lead_gps_pos);
    gps_lead_received = true;
}

void Drone2Control::fol_gps_to_ned(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr fol_gps_msg)
{
    Eigen::Vector3d fol_gps_pos = {fol_gps_msg -> lat, fol_gps_msg -> lon, static_cast<double>(fol_gps_msg -> alt)};

    fol_odom_pos << gps_to_ned(fol_gps_pos);
    gps_fol_received = true;
}

void Drone2Control::own_gps_to_ned(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr own_gps_msg)
{
    Eigen::Vector3d own_gps_pos = {own_gps_msg -> lat, own_gps_msg -> lon, static_cast<double>(own_gps_msg -> alt)};

    own_gps_to_odom << gps_to_ned(own_gps_pos);
    gps_own_received = true;
}

void Drone2Control::own_odom_func(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_own_msg)
{
    own_odom_pos << odom_own_msg -> position[0], odom_own_msg -> position[1], odom_own_msg -> position[2];
    odom_own_received = true;
}


// func count 1
void Drone2Control::trajectory_logic()
{   
    formation_input = -(own_gps_to_odom - lead_odom_pos) + (desired_own_pos - desired_lead_pos);
    collision_input = collision_logic();
    control_input = (own_odom_pos + formation_input) * step_gain + collision_input;

    // bedain logic pas takeoff
    if (z_sync_phase)
    {
        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        traj.position = {
        own_gps_to_odom.x(),
        own_gps_to_odom.y(),
        control_input.z()
    };

        // traj.yaw = current_yaw;
        trajectory_setpoint_pub_->publish(traj);
        Eigen::Vector3f actual_offset = own_gps_to_odom - lead_odom_pos;
        Eigen::Vector3f desired_offset = desired_own_pos - desired_lead_pos;
        Eigen::Vector3f offset_error = actual_offset - desired_offset;

        float z_error = own_gps_to_odom.z() - lead_odom_pos.z();
        if (std::abs(z_error) < z_tolerance)
        {
            z_sync_phase = false;
        }
        return;
    }

    if (!z_sync_phase)
    {
        Eigen::Vector3f actual_offset = own_gps_to_odom - lead_odom_pos;
        Eigen::Vector3f desired_offset = desired_own_pos - desired_lead_pos;
        Eigen::Vector3f offset_error = actual_offset - desired_offset;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "\n--- CONVERGENCE CHECK (GPS/NED) ---"
                             "\n  own  pos (NED) : [%.3f, %.3f, %.3f]"
                             "\n  lead pos (NED) : [%.3f, %.3f, %.3f]"
                             "\n  fol pos (NED) : [%.3f, %.3f, %.3f]"
                             "\n  actual  offset : [%.3f, %.3f, %.3f]"
                             "\n  desired offset : [%.3f, %.3f, %.3f]"
                             "\n  error          : [%.3f, %.3f, %.3f]"
                             "\n  error magnitude: %.3f m",
                             own_gps_to_odom.x(), own_gps_to_odom.y(), own_gps_to_odom.z(),
                             lead_odom_pos.x(), lead_odom_pos.y(), lead_odom_pos.z(),
                             fol_odom_pos.x(), fol_odom_pos.y(), fol_odom_pos.z(),
                             actual_offset.x(), actual_offset.y(), actual_offset.z(),
                             desired_offset.x(), desired_offset.y(), desired_offset.z(),
                             offset_error.x(), offset_error.y(), offset_error.z(),
                             offset_error.norm());

        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        traj.position = {
            control_input.x(),
            control_input.y(),
            control_input.z()};
        // traj.yaw = current_yaw;
        trajectory_setpoint_pub_->publish(traj);
    }
}

void Drone2Control::start_run()
{
    offboard_control_mode();

    if (setpoint_counter_ < 10)
    {
        if (!gps_own_received && !gps_lead_received)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Waiting vehicle_odometry for (drone 2 = own) and lead");
            return;
        }

        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        traj.position = {
            own_odom_pos.x(),
            own_odom_pos.y(),
            own_odom_pos.z()};
        // traj.yaw = current_yaw;
        trajectory_setpoint_pub_->publish(traj);
        setpoint_counter_++;
        return;
    }

    if (state == OffboardState::INIT)
    {
        RCLCPP_INFO(this->get_logger(), "Setting offboard mode (drone 2)...");
        set_offboard_command();
        state = OffboardState::OFFBOARD;
        return;
    }

    if (state == OffboardState::OFFBOARD)
    {
        RCLCPP_INFO(this->get_logger(), "Arming (drone 2)...");
        arm();
        state = OffboardState::ARMED;
        return;
    }

    // publish offboard_control_mode sama setpoint terus menerus
    offboard_control_mode();
    trajectory_logic();
}

void Drone2Control::offboard_control_mode()
{
    // PUBLISHER_COUNT (offboard control)
    px4_msgs::msg::OffboardControlMode offboard_msg{};
    offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_msg.position = true;
    offboard_msg.velocity = false;
    offboard_msg.acceleration = false;
    offboard_msg.attitude = false;
    offboard_msg.body_rate = false;
    offboard_control_mode_pub_->publish(offboard_msg);
}

void Drone2Control::arm()
{
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.param1 = 1.0; // arm
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.target_system = 3; // Drone 2
    cmd.target_component = 1;
    cmd.source_system = 2;
    cmd.source_component = 1;
    cmd.from_external = true;
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "Arm command sent (drone 2)");
}

void Drone2Control::set_offboard_command()
{
    // PUBLISHER_COUNT (vehicle command :vehicle mode)
    px4_msgs::msg::VehicleCommand vehicle_mode_msg{};
    vehicle_mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_mode_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    vehicle_mode_msg.param1 = 1.0;      // custom
    vehicle_mode_msg.param2 = 6.0;      // offboard
    vehicle_mode_msg.target_system = 3; // Drone 2
    vehicle_mode_msg.target_component = 1;
    vehicle_mode_msg.source_system = 1;
    vehicle_mode_msg.source_component = 1;
    vehicle_command_pub_->publish(vehicle_mode_msg);
    RCLCPP_INFO(this->get_logger(), "Offboard mode command sent (drone 2)");
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Drone2Control>());
    rclcpp::shutdown();
    return 0;
}