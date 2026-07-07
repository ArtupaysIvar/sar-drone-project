#include <rclcpp/rclcpp.hpp>
#include "pixel_msgs/msg/pixel_coordinates.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <chrono>
#include <fstream>
#include <limits>
#include <iomanip>
// #include "sensor_msgs/msg/camera_info.hpp"
// #include <sensor_msgs/msg/nav_sat_fix.hpp>
// #include <Eigen/Cholesky>
// #include <Eigen/Eigenvalues>

class DataCollectionNode : public rclcpp::Node
{   
public:
    // constructor
    DataCollectionNode();
    ~DataCollectionNode();
private:
    // callback funcs
    // void pixelCallback1(const pixel_msgs::msg::PixelCoordinates::SharedPtr YOLOmsg1);
    // void pixelCallback2(const pixel_msgs::msg::PixelCoordinates::SharedPtr YOLOmsg2);
    // void pixelCallback3(const pixel_msgs::msg::PixelCoordinates::SharedPtr YOLOmsg3);
    void odomCallback1(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg1);
    void odomCallback2(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg2);
    void odomCallback3(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg3);
    void triangulationCallback(const geometry_msgs::msg::Point::SharedPtr point_msg);
    void csvLogging();  

    double x1, y1, z1;
    double x2, y2, z2;
    double x3, y3, z3;
    Eigen::Vector3f origin_drone1, origin_drone2, origin_drone3;
    bool odom1_received{false}, odom2_received{false}, odom3_received{false};
    
    // Eigen::Vector3f pixel_vec1, pixel_vec2, pixel_vec3;
    // bool pixel_detected{0};
    // bool pixel1_received{false}, pixel2_received{false}, pixel3_received{false};
    // float conf1{0}, conf2{0}, conf3{0};
    // bool id_yolo1{false}, id_yolo2{false}, id_yolo3{false};

    Eigen::Vector3f target_location;
    Eigen::Vector3f act_target_location;
    bool target_location_received{false};
    double error_tri{0};
    
    double mission_time_s;
    rclcpp::Time mission_start_time;
    
    std::ofstream csv_file_;
    

    // subscribers
    // rclcpp::Subscription<pixel_msgs::msg::PixelCoordinates>::SharedPtr drone1_pix_subs;
    // rclcpp::Subscription<pixel_msgs::msg::PixelCoordinates>::SharedPtr drone2_pix_subs;
    // rclcpp::Subscription<pixel_msgs::msg::PixelCoordinates>::SharedPtr drone3_pix_subs;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr drone1_conf_subs;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr drone2_conf_subs;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr drone3_conf_subs;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_location_sub;
};

DataCollectionNode::DataCollectionNode() : Node("data_collection_node"),
    mission_start_time(rclcpp::Time(0, 0, RCL_ROS_TIME))

    {
        // drone1_pix_subs = this->create_subscription<pixel_msgs::msg::PixelCoordinates>
        // ("/pixel_topic1", 10, std::bind(&DataCollectionNode::pixelCallback1, this, 
        // std::placeholders::_1));
        // drone2_pix_subs = this->create_subscription<pixel_msgs::msg::PixelCoordinates>
        // ("/pixel_topic2", 10, std::bind(&DataCollectionNode::pixelCallback2, this, 
        // std::placeholders::_1));
        // drone3_pix_subs = this->create_subscription<pixel_msgs::msg::PixelCoordinates>
        // ("/pixel_topic3", 10, std::bind(&DataCollectionNode::pixelCallback3, this, 
        // std::placeholders::_1));
        
        // auto qos1 = rclcpp::QoS(rclcpp::KeepLast(10))
        //     .best_effort()
        //     .transient_local();
        auto qos1 = rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .durability_volatile();
        target_location_sub = this->create_subscription<geometry_msgs::msg::Point>
        ("/point_location", qos1, std::bind(&DataCollectionNode::triangulationCallback, this, 
        std::placeholders::_1));
        
        auto qos2 = rclcpp::QoS(rclcpp::KeepLast(10))
                           .best_effort()
                           .durability_volatile();

        drone1_conf_subs = this->create_subscription<px4_msgs::msg::VehicleOdometry>
        ("/px4_1/fmu/out/vehicle_odometry", qos2, std::bind(&DataCollectionNode::odomCallback1, this, 
        std::placeholders::_1));
        drone2_conf_subs = this->create_subscription<px4_msgs::msg::VehicleOdometry>
        ("/px4_2/fmu/out/vehicle_odometry", qos2, std::bind(&DataCollectionNode::odomCallback2, this, 
        std::placeholders::_1));
        drone3_conf_subs = this->create_subscription<px4_msgs::msg::VehicleOdometry>
        ("/px4_3/fmu/out/vehicle_odometry", qos2, std::bind(&DataCollectionNode::odomCallback3, this, 
        std::placeholders::_1));
        
        csv_file_.open("triangulation_data.csv", std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open triangulation_data.csv for writing.");
    } else {
        csv_file_ << "mission_time_s,tx,ty,tz,error,x1,x2,x3,y1,y2,y3,z1,z2,z3\n";
        csv_file_ << std::fixed << std::setprecision(6);
        RCLCPP_INFO(this->get_logger(), "Writing data to triangulation_data.csv");
    }
    origin_drone1 << 0.0f, 0.0f, 0.0f;
    origin_drone2 << -2.0f, -2.0f, 0.0f;
    origin_drone3 << 2.0f, -2.0f, 0.0f;
    act_target_location << 0.0f, 15.0f, 0.0f; 

    mission_start_time = this->now();
    // last_triangulation_time = this->now();
    };
DataCollectionNode::~DataCollectionNode()
{
    if (csv_file_.is_open()) {
        csv_file_.close();
    }
}

void DataCollectionNode::triangulationCallback(const geometry_msgs::msg::Point::SharedPtr point_msg){
    target_location << point_msg->x, point_msg->y, point_msg->z;
    error_tri = (target_location - act_target_location).norm();
    target_location_received = true;
}
// void DataCollectionNode::pixelCallback1(const pixel_msgs::msg::PixelCoordinates::SharedPtr YOLOmsg1){
//     conf1 = YOLOmsg1->confidence;
//     if(conf1 > 0.3){
//         pixel_vec1 << YOLOmsg1->u, YOLOmsg1->v, 1.0;
//         pixel1_received = true;;
//     }
//     else{
//         pixel1_received = false;
//     }
// }
// void DataCollectionNode::pixelCallback2(const pixel_msgs::msg::PixelCoordinates::SharedPtr YOLOmsg2){
//     conf2 = YOLOmsg2->confidence;
//     if(conf2 > 0.3){
//         pixel_vec2 << YOLOmsg2->u, YOLOmsg2->v, 1.0;
//         pixel2_received = true;
//     }
//     else{
//         pixel2_received = false;
//     }
// }
// void DataCollectionNode::pixelCallback3(const pixel_msgs::msg::PixelCoordinates::SharedPtr YOLOmsg3){
//     conf3 = YOLOmsg3->confidence;
//     if(conf3 > 0.3){
//         pixel_vec3 << YOLOmsg3->u, YOLOmsg3->v, 1.0;
//         pixel3_received = true;
//     }
//     else{
//         pixel3_received = false;
//     }
// }

void DataCollectionNode::odomCallback1(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg1){
    x1 = odom_msg1->position[0];
    y1 = odom_msg1->position[1];
    z1 = odom_msg1->position[2];
    odom1_received = true;
    // if (odom1_received && odom2_received && odom3_received) {
    //     csvLogging();
    // }
}

void DataCollectionNode::odomCallback2(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg2){
    x2 = odom_msg2->position[0];
    y2 = odom_msg2->position[1];
    z2 = odom_msg2->position[2];
    odom2_received = true;
    //     csvLogging();
    // if (odom1_received && odom2_received && odom3_received) {
    // }
}

void DataCollectionNode::odomCallback3(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg3){
    x3 = odom_msg3->position[0];
    y3 = odom_msg3->position[1];
    z3 = odom_msg3->position[2];
    odom3_received = true;
    if (odom1_received && odom2_received && odom3_received) {
        csvLogging();
    }
}

void DataCollectionNode::csvLogging(){
    
    // int detection_count = pixel1_received + pixel2_received + pixel3_received;
    bool valid_triangulation = target_location_received;

    mission_time_s = (this->now() - mission_start_time).seconds();

    // id_yolo1 = pixel1_received ? 1 : 0;
    // id_yolo2 = pixel2_received ? 1 : 0;
    // id_yolo3 = pixel3_received ? 1 : 0;

    // bool all_detected = pixel1_received && pixel2_received && pixel3_received;

    double tx = std::numeric_limits<double>::quiet_NaN();
    double ty = std::numeric_limits<double>::quiet_NaN();
    double tz = std::numeric_limits<double>::quiet_NaN();
    double err = std::numeric_limits<double>::quiet_NaN();

    if (valid_triangulation) {
        tx = target_location(0);
        ty = target_location(1);
        tz = target_location(2);
        err = error_tri;
    }
    
    if (csv_file_.is_open()) {
      if (valid_triangulation) {
        csv_file_ << mission_time_s << ","
                  << tx << ","
                  << ty << ","
                  << tz << ","
                  << err << ",";
    } else {
        csv_file_ << mission_time_s << ","
                  << "NaN,NaN,NaN,NaN,";
    }

    csv_file_
        << x1+origin_drone1(0)<< "," << x2+origin_drone2(0) << "," << x3+origin_drone3(0) << ","
        << y1+origin_drone1(1) << "," << y2+origin_drone2(1) << "," << y3+origin_drone3(1) << ","
        << z1+origin_drone1(2) << "," << z2+origin_drone2(2) << "," << z3+origin_drone3(2) << "\n";

} else {
    RCLCPP_ERROR(this->get_logger(), "Unable to write to CSV file.");
}

    odom1_received = false;
    odom2_received = false;
    odom3_received = false;
    // pixel1_received = false;
    // pixel2_received = false;
    // pixel3_received = false;
    // target_location_received = false;
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataCollectionNode>());
    rclcpp::shutdown();
    return 0;
}
