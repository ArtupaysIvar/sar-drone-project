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

using namespace std::chrono_literals;

class Drone1Control : public rclcpp::Node {
public:
    Drone1Control();

private:
    void offboard_control_mode();
    void arm();
    void set_offboard_command();
    void trajectory_logic();
    void follower1_gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr fol1_odom_msg);
    void follower2_gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr fol2_odom_msg);
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg);
    void own_gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr own_gps_msg);
    Eigen::Vector3f collision_logic();
    Eigen::Vector3f step_logic(const Eigen::Vector3f &input_avoid);
    bool wait_for_settle(const Eigen::Vector3f &fol2_pos, const std::vector<Eigen::Vector3f> &input_waypoint);
    bool waypoint_reached();
    void start_run();
    bool holding_func();
    
    // publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    // rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr origin_pub;
    // subscriber and timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_own_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr follower1_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr follower2_sub;
    
    // copy dari state machine
    enum class OffboardState {
    INIT,
    OFFBOARD,
    ARMED
    };
    OffboardState state_{OffboardState::INIT};

    float current_yaw{0.0};

    // Waypoint variables
    std::vector<Eigen::Vector3f> input_waypoint;
    size_t current_wp_idx_{0};

    // hold timer variables
    bool odom_received {false}; // supaya pasti ada isi data nya

    // constants for V_ij
    double d{5.0}, h{1.0};
    double c = pow(d,2)/3;
    double a = 4*h*pow(c,3);  

    //follower odom
    Eigen::Vector3f fol1_pos;
    Eigen::Vector3f fol2_pos;
    Eigen::Vector3f own_odom_pos;  
    Eigen::Vector3f own_gpstoodom_pos;  
    Eigen::Vector3f avoid_input;

    bool gps_fol1_received;
    bool gps_fol2_received;
    bool gps_own_received;
    bool odom_own_received;

    Eigen::Vector3f control_input;

    int setpoint_counter{0};
    bool z_sync_phase{true};

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
        std::bind(&Drone1Control::follower1_gps_callback, this, std::placeholders::_1));
    
    follower2_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/px4_3/fmu/out/vehicle_odometry", qos_profile,
        std::bind(&Drone1Control::follower2_gps_callback, this, std::placeholders::_1));
    
    timer_ = create_wall_timer(100ms, std::bind(&Drone1Control::start_run, this));
    
    // JANGAN LUPA SET WAYPOINT NYA
    input_waypoint.reserve(20);
    // kalo z = 0 berarti dia simply tidak climb (bukan berarti landing) 
    input_waypoint.emplace_back(0.0f, 0.0f, -5.0f);
    input_waypoint.emplace_back(0.0f, 20.0f, 0.0f);
    input_waypoint.emplace_back(5.0f, 0.0f, 0.0f);
    input_waypoint.emplace_back(0.0f, -20.0f, 0.0f);
    input_waypoint.emplace_back(5.0f, 0.0f, 0.0f);
    input_waypoint.emplace_back(0.0f, 20.0f, 0.0f);
    input_waypoint.emplace_back(5.0f, 0.0f, 0.0f);
    input_waypoint.emplace_back(0.0f, -20.0f, 0.0f);
    input_waypoint.emplace_back(5.0f, 0.0f, 0.0f);
    input_waypoint.emplace_back(0.0f, 20.0f, 0.0f);
    input_waypoint.emplace_back(0.0f, 0.0f, 5.0f);}

void Drone1Control::arm() {
    // PUBLISHER_COUNT (vehicle command: arm)
    px4_msgs::msg::VehicleCommand arm_msg{};
    arm_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    arm_msg.param1 = 1.0;  // arm
    arm_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    arm_msg.target_system = 2;     
    arm_msg.target_component = 1;
    arm_msg.source_system = 1;
    arm_msg.source_component = 1;
    vehicle_command_pub_->publish(arm_msg);
    RCLCPP_INFO(this->get_logger(), "Arm command sent (drone 1)");
}

void Drone1Control::set_offboard_command() {
    // PUBLISHER_COUNT (vehicle command :vehicle mode)
    px4_msgs::msg::VehicleCommand vehicle_mode_msg{};
    vehicle_mode_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_mode_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    vehicle_mode_msg.param1 = 1.0;  // custom
    vehicle_mode_msg.param2 = 6.0;  // offboard
    vehicle_mode_msg.target_system = 2;
    vehicle_mode_msg.target_component = 1;
    vehicle_mode_msg.source_system = 1;
    vehicle_mode_msg.source_component = 1;
    vehicle_command_pub_->publish(vehicle_mode_msg);
    RCLCPP_INFO(this->get_logger(), "Offboard mode command sent (drone 1)");
}

void Drone1Control::offboard_control_mode() {
    // PUBLISHER_COUNT (offboard control)
    px4_msgs::msg::OffboardControlMode offboard_msg{};
    offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_msg.position = true;
    // set true apa ya?
    offboard_msg.velocity = false;
    offboard_msg.acceleration = false;
    offboard_msg.attitude = false;
    offboard_msg.body_rate = false;
    offboard_control_mode_pub_->publish(offboard_msg);
}

Eigen::Vector3f gps_to_ned(const Eigen::Vector3d gps_msg){

    const double R = 6371000.0; // earth radius meters
    float north = static_cast<float>(gps_msg.x() * (M_PI/180.0) * R);
    float east  = static_cast<float>(gps_msg.y() * (M_PI/180.0) * R);
    float down  = static_cast<float>(-gps_msg.z());

    Eigen::Vector3f ned_coor = {north, east, down};
    return ned_coor;
}

void Drone1Control::follower1_gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr fol1_gps_msg) {
    Eigen::Vector3d fol1_gps_pos = {fol1_gps_msg -> lat, fol1_gps_msg -> lon, static_cast<double>(fol1_gps_msg -> alt)};
    fol1_pos = gps_to_ned(fol1_gps_pos);

    gps_fol1_received = true;
}

void Drone1Control::follower2_gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr fol2_gps_msg) {
    Eigen::Vector3d fol2_gps_pos = {fol2_gps_msg -> lat, fol2_gps_msg -> lon, static_cast<double>(fol2_gps_msg -> alt)};   
    fol2_pos = gps_to_ned(fol2_gps_pos);

    gps_fol2_received = true;
}

void Drone1Control::own_gps_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr own_gps_msg) {
    Eigen::Vector3d own_gps_pos = {own_gps_msg -> lat, own_gps_msg -> lon, static_cast<double>(own_gps_msg -> alt)};
    own_gpstoodom_pos = gps_to_ned(own_gps_pos);

    gps_own_received = true;
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

    // yaw_rotational_matrix <<
    // std::cos(current_yaw), -std::sin(current_yaw),
    // std::sin(current_yaw),  std::cos(current_yaw);

    own_odom_pos << odom_msg->position[0], odom_msg->position[1], odom_msg->position[2];
    odom_received = true;
}

Eigen::Vector3f Drone1Control::collision_logic(){

    std::vector<Eigen::Vector3f> fol_list= {fol1_pos, fol2_pos};
    Eigen::Vector3f avoid_input(Eigen::Vector3f::Zero());

    for(auto i = 0; i<fol_list.size(); i++){
        float beta = (own_odom_pos - fol_list[i]).squaredNorm();

        if (beta < 1e-6f) {
            continue; // coincident with follower, skip to avoid a div-by-zero blow-up
        }
        
        float rho{0.0f};
        
        // float beta_dist = (current_pos - fol_list[i]).norm();
        Eigen::Vector3f q_sub = own_odom_pos - fol_list[i];
    
        if(beta < c){
            rho = -(2*a)/pow(beta,2);
            return avoid_input += rho * (-q_sub);
        }
        else if(c <= beta && beta < pow(d,2)){
            rho = -4*h*(beta-pow(d,2));
        }
        
        avoid_input = rho * (-q_sub);
    }
    return avoid_input;
}

// only run for the first wp
// bool Drone1Control::wait_for_settle(const Eigen::Vector3f &fol2_pos, 
//     const std::vector<Eigen::Vector3f> &input_waypoint){
//     // assuming that drrone will not go up again
//     bool fol_alt_ready = std::abs(fol2_pos[2] - input_waypoint[0].z()) < 0.2;
//     return fol_alt_ready;
// }


// bool Drone1Control::holding_func(){
//     double elapsed = (this->get_clock()->now()).seconds();
//     bool cont = elapsed >= 5.0;
//     return cont;
// }

bool Drone1Control::waypoint_reached()
{
    float distance_selisih = (own_odom_pos - input_waypoint[current_wp_idx_]).norm();
    return (distance_selisih < 0.2);
}

// this one is to control velocity
Eigen::Vector3f Drone1Control::step_logic(const Eigen::Vector3f &input_avoid)
{
    // Eigen::Vector3f target_pos_3d = target_pos[idx];
    // Eigen::Vector3f odom_pos_3d = current_pos;
    Eigen::Vector3f selisih_disp = (input_waypoint[current_wp_idx_] - own_odom_pos).normalized();
    float selisih_dist = selisih_disp.norm();
   
    Eigen::Vector3f current_pos = own_odom_pos;

    if (selisih_dist <= 0.2 || selisih_dist < 1e-3f)
    {return input_waypoint[current_wp_idx_];}
    else return current_pos + (selisih_disp + input_avoid) * 0.3;
}

void Drone1Control::trajectory_logic(){

    control_input = step_logic(collision_logic());

    if (current_wp_idx_ >= input_waypoint.size()) {
        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        traj.position = {own_odom_pos.x(), own_odom_pos.y(), own_odom_pos.z()};
        traj.yaw = current_yaw;
        trajectory_setpoint_pub_->publish(traj);
        return;
    }

    if (z_sync_phase){
        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        traj.position = {input_waypoint[current_wp_idx_].x(), input_waypoint[current_wp_idx_].y(), input_waypoint[current_wp_idx_].z()};
        traj.yaw = current_yaw;
        trajectory_setpoint_pub_->publish(traj); 

        if((own_gpstoodom_pos - fol2_pos).norm() < 0.3){
            current_wp_idx_++;
            z_sync_phase = false;
            return;
        }
    }

    px4_msgs::msg::TrajectorySetpoint traj{};
    traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    traj.position = {control_input.x(), control_input.y(), control_input.z()};
    traj.yaw = current_yaw;
    trajectory_setpoint_pub_->publish(traj);


    if (waypoint_reached){
        current_wp_idx_++;
        double hold_time = this->get_clock()->now().seconds();
            if (hold_time >= 5.0){
                px4_msgs::msg::TrajectorySetpoint traj{};
                traj.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                traj.position = {own_odom_pos.x(), own_odom_pos.y(), own_odom_pos.z()};
                traj.yaw = current_yaw;
                trajectory_setpoint_pub_->publish(traj);
            }
            else return;
    }

}


// to reset origin after waypoint is reached
// Eigen::Vector3f Drone1Control::reset_pos(const Eigen::Vector2f &target, )
// {   
//     return Eigen::Vector3f curr_pos = ;
// }



// function buat starting
void Drone1Control::start_run(){
    offboard_control_mode();
    if (setpoint_counter < 10) {
        if (!odom_received) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Waiting for vehicle_odometry (drone 1)...");
        return;
        }

        offboard_control_mode();

        // publisher setting buat waypoint
        px4_msgs::msg::TrajectorySetpoint traj{};
        traj.timestamp = this->get_clock()->now().nanoseconds() / 1000; 
        traj.position = {
            own_odom_pos[0],
            own_odom_pos[1],
            own_odom_pos[2]};
        traj.yaw = current_yaw;
        trajectory_setpoint_pub_->publish(traj);
        setpoint_counter++;
        
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

    // publish offboard_control_mode sama setpoint terus menerus
    offboard_control_mode();
    trajectory_logic();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Drone1Control>());
    rclcpp::shutdown();
    return 0;
}