// testing konsep git pull

// #include <iostream>
// #include <ctime>
// #include <Eigen/Dense>
// #include <random>
// #include <iostream>
// #include <fstream>
// #include <ceres/ceres.h>
#include <rclcpp/rclcpp.hpp>
#include "pixel_msgs/msg/pixel_coordinates.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Jacobi>
#include <cmath>
#include <vector>
#include <chrono>

class CalcGPSNode : public rclcpp::Node
{   
public:
    // constructor
    CalcGPSNode();

private:

    // callback funcs
    void pixelCallback1(const pixel_msgs::msg::PixelCoordinates::SharedPtr YOLOmsg1);
    void pixelCallback2(const pixel_msgs::msg::PixelCoordinates::SharedPtr YOLOmsg2);
    void pixelCallback3(const pixel_msgs::msg::PixelCoordinates::SharedPtr YOLOmsg3);
    void confCallback1(const px4_msgs::msg::VehicleOdometry::SharedPtr confmsg1);
    void confCallback2(const px4_msgs::msg::VehicleOdometry::SharedPtr confmsg2);
    void confCallback3(const px4_msgs::msg::VehicleOdometry::SharedPtr confmsg3);
    void cameraInfoCallback1(const sensor_msgs::msg::CameraInfo::SharedPtr caminfo1);
    void cameraInfoCallback2(const sensor_msgs::msg::CameraInfo::SharedPtr caminfo2);
    void cameraInfoCallback3(const sensor_msgs::msg::CameraInfo::SharedPtr caminfo3);
    Eigen::Vector3f gps_to_ned(double lat, double lon, double alt,
                            double ref_lat, double ref_lon, double ref_alt);
    void origin_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void gps1_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void gps2_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void gps3_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    bool dataCheck();
    void timerCallback();
    bool ready();
    void projectionFormula();
    // bool MVMP_triangulation();
    bool IRMP_triangulation();


    // buat pixelCallback
    float conf1{0}, conf2{0}, conf3{0};
    
    // buat confCallback
    double drone1_xpos, drone1_ypos, drone1_zpos;
    double drone2_xpos, drone2_ypos, drone2_zpos;
    double drone3_xpos, drone3_ypos, drone3_zpos;
    Eigen::Vector3d drone1_pos_vec;
    Eigen::Vector3d drone2_pos_vec;
    Eigen::Vector3d drone3_pos_vec;
    Eigen::Quaterniond q1;
    Eigen::Quaterniond q2;
    Eigen::Quaterniond q3;
    
    // buat checking
    bool pixel1_received = false;
    bool pixel2_received = false;
    bool pixel3_received = false;
    bool odom1_received = false;
    bool odom2_received = false;
    bool odom3_received = false;
    bool cam_mat1_received = false;
    bool cam_mat2_received = false;
    bool cam_mat3_received = false;

    // buat projection
    // std::vector<Eigen::Vector3d> proj_vec;
    // std::vector<Eigen::Vector3d> proj_uvec;
    // std::vector<Eigen::Matrix3d> rot_mat;
    Eigen::Vector3d pixel_vec1;
    Eigen::Vector3d pixel_vec2;
    Eigen::Vector3d pixel_vec3;
    // cam intrinsic 
    Eigen::Matrix3d cam_mat1;
    Eigen::Matrix3d cam_mat2;
    Eigen::Matrix3d cam_mat3;
    // for the cam body rotation
    // Eigen::Matrix3d enu_to_ned_mat;
    Eigen::Matrix3d rot_cam_frame;

    // for the drone body rotation
    Eigen::Matrix3d rot_mat1;
    Eigen::Matrix3d rot_mat2;
    Eigen::Matrix3d rot_mat3;

    Eigen::Vector3d proj_vec1;
    Eigen::Vector3d proj_vec2;
    Eigen::Vector3d proj_vec3;

    Eigen::Vector3d proj_uvec1;
    Eigen::Vector3d proj_uvec2;
    Eigen::Vector3d proj_uvec3;
    
    // buat MVMP_triangulation
    std::vector<Eigen::Vector3d> b_vec;
    std::vector<Eigen::Vector3d> o_vec;
    std::vector<Eigen::Matrix3d> B_proj_mat;

    Eigen::Matrix3d A_sum;
    Eigen::Vector3d B_sum;
    Eigen::Vector3d target_point;
    bool target_valid = false;

    // buat IRMP_triangulation
    int max_iters_ = 10;
    Eigen::Vector3d prev_point;
    // Eigen::Vector3d error_vec;
    // Eigen::Vector3d cam_to_P_vec;
    // double cam_to_P_vec_dist_squared;
    // double weight_squared;
    // double irmp_error;

    // Eigen::Matrix3d A_sum = Eigen::Matrix3d::Zero();
    // Eigen::Vector3d b_sum = Eigen::Vector3d::Zero();

    // subscribers
    rclcpp::Subscription<pixel_msgs::msg::PixelCoordinates>::SharedPtr drone1_pix_subs;
    rclcpp::Subscription<pixel_msgs::msg::PixelCoordinates>::SharedPtr drone2_pix_subs;
    rclcpp::Subscription<pixel_msgs::msg::PixelCoordinates>::SharedPtr drone3_pix_subs;
    
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr drone1_conf_subs;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr drone2_conf_subs;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr drone3_conf_subs;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr drone1_caminfo_subs;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr drone2_caminfo_subs;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr drone3_caminfo_subs;

    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr drone1_gps_sub1;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr drone2_gps_sub2;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr drone3_gps_sub3;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr origin_sub;

    // publisher & timer
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool origin_received = false;
    double ref_lat{0}, ref_lon{0}, ref_alt{0};
    bool gps1_received{false}, gps2_received{false}, gps3_received{false};
};

CalcGPSNode::CalcGPSNode() : Node("calc_gps_node")
    {
        RCLCPP_INFO(this->get_logger(), "GPS approx. node started");

        drone1_pix_subs = this->create_subscription<pixel_msgs::msg::PixelCoordinates>
        ("/pixel_topic1", 10, std::bind(&CalcGPSNode::pixelCallback1, this, 
        std::placeholders::_1));
        drone2_pix_subs = this->create_subscription<pixel_msgs::msg::PixelCoordinates>
        ("/pixel_topic2", 10, std::bind(&CalcGPSNode::pixelCallback2, this, 
        std::placeholders::_1));
        drone3_pix_subs = this->create_subscription<pixel_msgs::msg::PixelCoordinates>
        ("/pixel_topic3", 10, std::bind(&CalcGPSNode::pixelCallback3, this, 
        std::placeholders::_1));
        
        auto qos1 = rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .durability_volatile();

        drone1_gps_sub1 = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>
        ("/px4_1/fmu/out/vehicle_global_position", qos1, std::bind(&CalcGPSNode::gps1_callback, this, 
        std::placeholders::_1));
        drone2_gps_sub2 = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>
        ("/px4_2/fmu/out/vehicle_global_position", qos1, std::bind(&CalcGPSNode::gps2_callback, this, 
        std::placeholders::_1));
        drone3_gps_sub3 = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>
        ("/px4_3/fmu/out/vehicle_global_position", qos1, std::bind(&CalcGPSNode::gps3_callback, this, 
        std::placeholders::_1));
        
        // TODO: bikin callback function sama parse buat masukin ke camera matrix
        drone1_caminfo_subs = this->create_subscription<sensor_msgs::msg::CameraInfo>
        ("/world/custom/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/camera_info", 
            10, std::bind(&CalcGPSNode::cameraInfoCallback1, this, 
        std::placeholders::_1));
        drone2_caminfo_subs = this->create_subscription<sensor_msgs::msg::CameraInfo>
        ("/world/custom/model/x500_mono_cam_down_2/link/camera_link/sensor/imager/camera_info", 
            10, std::bind(&CalcGPSNode::cameraInfoCallback2, this, 
        std::placeholders::_1));
        drone3_caminfo_subs = this->create_subscription<sensor_msgs::msg::CameraInfo>
        ("/world/custom/model/x500_mono_cam_down_3/link/camera_link/sensor/imager/camera_info", 
            10, std::bind(&CalcGPSNode::cameraInfoCallback3, this, 
        std::placeholders::_1));
                        
        auto qos2 = rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .transient_local();

        drone1_conf_subs = this->create_subscription<px4_msgs::msg::VehicleOdometry>
        ("/px4_1/fmu/out/vehicle_odometry", qos2, std::bind(&CalcGPSNode::confCallback1, this, 
        std::placeholders::_1));
        drone2_conf_subs = this->create_subscription<px4_msgs::msg::VehicleOdometry>
        ("/px4_2/fmu/out/vehicle_odometry", qos2, std::bind(&CalcGPSNode::confCallback2, this, 
        std::placeholders::_1));
        drone3_conf_subs = this->create_subscription<px4_msgs::msg::VehicleOdometry>
        ("/px4_3/fmu/out/vehicle_odometry", qos2, std::bind(&CalcGPSNode::confCallback3, this, 
        std::placeholders::_1));
        
        auto origin_qos = rclcpp::QoS(1)
                    .reliable()
                    .transient_local();
        
        origin_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/swarm/global_origin", origin_qos,
            std::bind(&CalcGPSNode::origin_callback, this, std::placeholders::_1));


        // drone1_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>
        // ("/px4_1/fmu/out/vehicle_global_position", 10, std::bind(&CalcGPSNode::gps_callback_drone1, this, 
        // std::placeholders::_1));
        // drone2_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>
        // ("/px4_2/fmu/out/vehicle_global_position", 10, std::bind(&CalcGPSNode::gps_callback_drone2, this, 
        // std::placeholders::_1));
        // drone3_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>
        // ("/px4_3/fmu/out/vehicle_global_position", 10, std::bind(&CalcGPSNode::gps_callback_drone3, this, 
        // std::placeholders::_1));
        

        timer_ = this->create_wall_timer(std::chrono::milliseconds(400), std::bind(&CalcGPSNode::timerCallback, this));

        publisher_ = this->create_publisher<geometry_msgs::msg::Point>
        ("/point_location", 10);
        
        B_proj_mat.resize(3);
        rot_cam_frame << 
            0, -1, 0,
            1,  0, 0,
            0,  0, 1;
            RCLCPP_INFO(this->get_logger(),
            "1,2 matrix element: %f", rot_cam_frame(0,1));

            // 0, -1, 0, 
            // -1, 0, 0, 
            // 0, 0, 1;

            // 0, 1, 0, 
            // 1, 0, 0, 
            // 0, 0, 1; -> closest

            // 0, 1, 0, 
            // 1, 0, 0, 
            // 0, 0, -1;

            // 0,0,1,
            // 1,0,0,
            // 0,1,0;

    };
Eigen::Vector3f CalcGPSNode::gps_to_ned(
    double lat, double lon, double alt,
    double ref_lat, double ref_lon, double ref_alt)
{
    const double R = 6371000.0; // earth radius meters
    float north = static_cast<float>((lat - ref_lat) * (M_PI/180.0) * R);
    float east  = static_cast<float>((lon - ref_lon) * (M_PI/180.0) * R * std::cos(ref_lat * M_PI/180.0));
    float down  = static_cast<float>(-(alt - ref_alt));
    return {north, east, down};
}

void CalcGPSNode::origin_callback(const sensor_msgs::msg::NavSatFix::SharedPtr origin_msg)
{
    if(!origin_received){
    ref_lat = origin_msg -> latitude;
    ref_lon = origin_msg -> longitude;
    ref_alt = origin_msg -> altitude;

    RCLCPP_INFO(this->get_logger(),
    "Received swarm origin: lat=%.8f lon=%.8f alt=%.2f",
    ref_lat, ref_lon, ref_alt);

    origin_received = true;}
    else return;
}

void CalcGPSNode::gps1_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr gps_msg)
{
    if (!gps_msg->lat_lon_valid || !gps_msg->alt_valid) return;
    if (!origin_received) return;
    auto ned_pos = gps_to_ned(gps_msg->lat, gps_msg->lon, static_cast<double>(gps_msg->alt),
                            ref_lat, ref_lon, ref_alt);
    drone1_pos_vec << ned_pos.x(), ned_pos.y(), ned_pos.z();
    gps1_received = true;
}

void CalcGPSNode::gps2_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr gps_msg)
{
    if (!gps_msg->lat_lon_valid || !gps_msg->alt_valid)return;
    if (!origin_received) return;
    auto ned_pos = gps_to_ned(gps_msg->lat, gps_msg->lon, static_cast<double>(gps_msg->alt),
                            ref_lat, ref_lon, ref_alt);
    drone2_pos_vec << ned_pos.x(), ned_pos.y(), ned_pos.z();
    gps2_received = true;
}

void CalcGPSNode::gps3_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr gps_msg)
{
    if (!gps_msg->lat_lon_valid || !gps_msg->alt_valid)return;
    if (!origin_received) return;
    auto ned_pos = gps_to_ned(gps_msg->lat, gps_msg->lon, static_cast<double>(gps_msg->alt),
                            ref_lat, ref_lon, ref_alt);
    drone3_pos_vec << ned_pos.x(), ned_pos.y(), ned_pos.z();
    gps3_received = true;
}
        // CALLBACK BUAT PIXEL
void CalcGPSNode::pixelCallback1(const pixel_msgs::msg::PixelCoordinates::SharedPtr YOLOmsg1){
        conf1 = YOLOmsg1->confidence;
        pixel_vec1 << YOLOmsg1->u, YOLOmsg1->v, 1.0;
        pixel1_received = true;
    }

void CalcGPSNode::pixelCallback2(const pixel_msgs::msg::PixelCoordinates::SharedPtr YOLOmsg2){
        conf2 = YOLOmsg2->confidence;
        pixel_vec2 << YOLOmsg2->u, YOLOmsg2->v, 1;
        pixel2_received = true;
    }

void CalcGPSNode::pixelCallback3(const pixel_msgs::msg::PixelCoordinates::SharedPtr YOLOmsg3){
        conf3 = YOLOmsg3->confidence;
        pixel_vec3 << YOLOmsg3->u, YOLOmsg3->v, 1;
        pixel3_received = true;
    }
        // CALLBACK BUAT conf
        // rotational matrix will be the functions of drone's rotation

void CalcGPSNode::confCallback1(const px4_msgs::msg::VehicleOdometry::SharedPtr confmsg1){
        // drone1_xpos = confmsg1 -> position[0];
        // drone1_ypos = confmsg1 -> position[1];
        // drone1_zpos = confmsg1 -> position[2];

        // drone1_pos_vec << confmsg1 -> position[0], confmsg1 -> position[1], confmsg1 -> position[2];
        // Check if PX4 quaternion order is [w,x,y,z] or [x,y,z,w]
        q1 = Eigen::Quaterniond(static_cast<double>(confmsg1->q[0]), static_cast<double>(confmsg1->q[1]), 
        static_cast<double>(confmsg1->q[2]), static_cast<double>(confmsg1->q[3]));
        rot_mat1 = q1.toRotationMatrix();
        odom1_received = true;
    }

void CalcGPSNode::confCallback2(const px4_msgs::msg::VehicleOdometry::SharedPtr confmsg2){
        // drone2_xpos = confmsg2 -> position[0];
        // drone2_ypos = confmsg2 -> position[1];
        // drone2_zpos = confmsg2 -> position[2];

        // drone2_pos_vec << confmsg2 -> position[0], confmsg2 -> position[1], confmsg2 -> position[2];
        q2 = Eigen::Quaterniond(static_cast<double>(confmsg2->q[0]), static_cast<double>(confmsg2->q[1]), 
        static_cast<double>(confmsg2->q[2]), static_cast<double>(confmsg2->q[3]));
        rot_mat2 = q2.toRotationMatrix();
        odom2_received = true;
    }

void CalcGPSNode::confCallback3(const px4_msgs::msg::VehicleOdometry::SharedPtr confmsg3){
        // drone3_xpos = confmsg3 -> position[0];
        // drone3_ypos = confmsg3 -> position[1];
        // drone3_zpos = confmsg3 -> position[2];

        // drone3_pos_vec << confmsg3 -> position[0], confmsg3 -> position[1], confmsg3 -> position[2];
        q3 = Eigen::Quaterniond(static_cast<double>(confmsg3->q[0]), static_cast<double>(confmsg3->q[1]), 
        static_cast<double>(confmsg3->q[2]), static_cast<double>(confmsg3->q[3]));
        rot_mat3 = q3.toRotationMatrix();
        odom3_received = true;
    }

void CalcGPSNode::cameraInfoCallback1(const sensor_msgs::msg::CameraInfo::SharedPtr caminfo1){
        cam_mat1 << caminfo1->k[0], caminfo1->k[1], caminfo1->k[2],
                   caminfo1->k[3], caminfo1->k[4], caminfo1->k[5],
                   caminfo1->k[6], caminfo1->k[7], caminfo1->k[8];
        cam_mat1_received = true;
    }

void CalcGPSNode::cameraInfoCallback2(const sensor_msgs::msg::CameraInfo::SharedPtr caminfo2){
        cam_mat2 << caminfo2->k[0], caminfo2->k[1], caminfo2->k[2],
                   caminfo2->k[3], caminfo2->k[4], caminfo2->k[5],
                   caminfo2->k[6], caminfo2->k[7], caminfo2->k[8];
        cam_mat2_received = true;
    }

void CalcGPSNode::cameraInfoCallback3(const sensor_msgs::msg::CameraInfo::SharedPtr caminfo3){
        cam_mat3 << caminfo3->k[0], caminfo3->k[1], caminfo3->k[2],
                   caminfo3->k[3], caminfo3->k[4], caminfo3->k[5],
                   caminfo3->k[6], caminfo3->k[7], caminfo3->k[8];
        cam_mat3_received = true;
    }

bool CalcGPSNode::dataCheck()
{
    return (pixel1_received && pixel2_received && pixel3_received &&
            odom1_received && odom2_received && odom3_received && 
            gps1_received && gps2_received && gps3_received &&
            cam_mat1_received && cam_mat2_received && cam_mat3_received &&
            origin_received);
}

bool CalcGPSNode::ready(){
        // if(conf1>0.5 || conf2>0.5 || conf3>0.5);
        return (conf1 > 0.2f && conf2 > 0.2f && conf3 > 0.2f);
}

    // start triangulation
void CalcGPSNode::projectionFormula()
    {
        // x = K^-1 * x'
        // Eigen::Vector3d cam_mat_curr =  cam_mat_curr[0];    
        // INGETIN URUTANNYA 
        // MUNGKIN ADA ISU SAMA CAM_MAT nya

        proj_vec1 =  rot_mat1 * rot_cam_frame * (cam_mat1.inverse() * pixel_vec1);
        proj_vec2 =  rot_mat2 * rot_cam_frame * (cam_mat2.inverse() * pixel_vec2);
        proj_vec3 =  rot_mat3 * rot_cam_frame * (cam_mat3.inverse() * pixel_vec3);
        
        proj_uvec1 = proj_vec1/proj_vec1.norm();
        proj_uvec2 = proj_vec2/proj_vec2.norm();
        proj_uvec3 = proj_vec3/proj_vec3.norm();


        RCLCPP_INFO(this->get_logger(),
        "rot_mat1:\n %.2f %.2f %.2f\n %.2f %.2f %.2f\n %.2f %.2f %.2f",
        rot_mat1(0,0), rot_mat1(0,1), rot_mat1(0,2),
        rot_mat1(1,0), rot_mat1(1,1), rot_mat1(1,2),
        rot_mat1(2,0), rot_mat1(2,1), rot_mat1(2,2));
        
        RCLCPP_INFO(this->get_logger(),
        "rot_mat2:\n %.2f %.2f %.2f\n %.2f %.2f %.2f\n %.2f %.2f %.2f",
        rot_mat2(0,0), rot_mat2(0,1), rot_mat2(0,2),
        rot_mat2(1,0), rot_mat2(1,1), rot_mat2(1,2),
        rot_mat2(2,0), rot_mat2(2,1), rot_mat2(2,2));

        RCLCPP_INFO(this->get_logger(),
        "rot_mat3:\n %.2f %.2f %.2f\n %.2f %.2f %.2f\n %.2f %.2f %.2f",
        rot_mat3(0,0), rot_mat3(0,1), rot_mat3(0,2),
        rot_mat3(1,0), rot_mat3(1,1), rot_mat3(1,2),
        rot_mat3(2,0), rot_mat3(2,1), rot_mat3(2,2));


        // in projectionFormula(), temporarily add:
        Eigen::Vector3d raw1 = cam_mat1.inverse() * pixel_vec1;
        Eigen::Vector3d after_camframe1 = rot_cam_frame * raw1;
        Eigen::Vector3d after_rotmat1 = rot_mat1 * after_camframe1;

        Eigen::Vector3d raw2 = cam_mat2.inverse() * pixel_vec2;
        Eigen::Vector3d after_camframe2 = rot_cam_frame * raw2;
        Eigen::Vector3d after_rotmat2 = rot_mat2 * after_camframe2;

        Eigen::Vector3d raw3 = cam_mat3.inverse() * pixel_vec3;
        Eigen::Vector3d after_camframe3 = rot_cam_frame * raw3;
        Eigen::Vector3d after_rotmat3 = rot_mat3 * after_camframe3;

        RCLCPP_INFO(this->get_logger(),
        "raw1: [%.2f %.2f %.2f] | after_camframe1: [%.2f %.2f %.2f] | final1: [%.2f %.2f %.2f]",
        raw1.x(), raw1.y(), raw1.z(),
        after_camframe1.x(), after_camframe1.y(), after_camframe1.z(),
        after_rotmat1.x(), after_rotmat1.y(), after_rotmat1.z());

        RCLCPP_INFO(this->get_logger(),
        "raw2: [%.2f %.2f %.2f] | after_camframe2: [%.2f %.2f %.2f] | final2: [%.2f %.2f %.2f]",
        raw2.x(), raw2.y(), raw2.z(),
        after_camframe2.x(), after_camframe2.y(), after_camframe2.z(),
        after_rotmat2.x(), after_rotmat2.y(), after_rotmat2.z());

        RCLCPP_INFO(this->get_logger(),
        "raw3: [%.2f %.2f %.2f] | after_camframe3: [%.2f %.2f %.2f] | final3: [%.2f %.2f %.2f]",
        raw3.x(), raw3.y(), raw3.z(),
        after_camframe3.x(), after_camframe3.y(), after_camframe3.z(),
        after_rotmat3.x(), after_rotmat3.y(), after_rotmat3.z());
         
    }
/*
bool CalcGPSNode::MVMP_triangulation(){
    
    b_vec = {proj_uvec1, proj_uvec2, proj_uvec3};
    o_vec = {drone1_pos_vec, drone2_pos_vec, drone3_pos_vec};
    A_sum.setZero();
    B_sum.setZero();
    
    for (int i = 0; i < 3; ++i) {
        B_proj_mat[i] = Eigen::Matrix3d::Identity() - b_vec[i] * b_vec[i].transpose();
        
        A_sum += B_proj_mat[i];
        B_sum += B_proj_mat[i] * o_vec[i];
    }
     
    // target_point = A_sum.ldlt().solve(B_sum);
    // res = A_sum.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b_sum);

    // 1st check: cek determinant
    double det = A_sum.determinant();
    if (std::abs(det) < 1e-10) {
        // RCLCPP_WARN(this->get_logger(), "Triangulation failed: A_sum is near-singular (det=%.2e)", det);
        target_valid = false;
        return false;
    }

    // target_point = A_sum.ldlt().solve(B_sum); // solve
    
    // 2nd check: cek solution
    if (!target_point.allFinite()) {
        // RCLCPP_WARN(this->get_logger(), "Triangulation failed: result contains NaN or Inf");
        target_valid = false;
        return false;
    }
    
    // 3rd check: check if point is in front of all cameras 
    // for (int i = 0; i < 3; ++i) {
    //     Eigen::Vector3d to_point = target_point - o_vec[i];
    //     double depth = to_point.dot(b_vec[i]);
    //     if (depth < 0.0) {
    //         // RCLCPP_WARN(this->get_logger(), "Triangulation warning: point behind camera %d (depth=%.2f)", i+1, depth);
    //         target_valid = false;
    //         return false;
    //     }
    // }

    target_valid = true;

    return true;
}
*/
bool CalcGPSNode::IRMP_triangulation()
{   
    b_vec = {proj_uvec1, proj_uvec2, proj_uvec3};
    o_vec = {drone1_pos_vec, drone2_pos_vec, drone3_pos_vec};
    
    // B_proj_mat(3);
    for (int i = 0; i < 3; ++i) {
        B_proj_mat[i] = Eigen::Matrix3d::Identity() - b_vec[i] * b_vec[i].transpose();
    }

    A_sum.setZero();
    B_sum.setZero();

    for(int i = 0; i < 3; ++i){
        A_sum += B_proj_mat[i];
        B_sum += B_proj_mat[i] * o_vec[i];
    }
     
    // target_point = A_sum.ldlt().solve(B_sum);
    // res = A_sum.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b_sum);

    // 1st check: cek determinant
    double det = A_sum.determinant();

    if (std::abs(det) < 1e-10) {
        // RCLCPP_WARN(this->get_logger(), "Triangulation failed: A_sum is near-singular (det=%.2e)", det);
        target_valid = false;
        return false;
    }

    // target_point = A_sum.ldlt().solve(B_sum); // solve for the first time using mvmp
    
    target_point = A_sum.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(B_sum);

    for(int iter = 1; iter < max_iters_; iter++)
    {
        prev_point = target_point;

        A_sum.setZero();
        B_sum.setZero();
        
        for(int i = 0; i < 3; i ++)
        {

            // Eigen::Vector3d op = prev_point - o_vec[i];
            // cam_to_P_vec = prev_point - o_vec[i];
            // cam_to_P_vec_dist_squared = cam_to_P_vec.squaredNorm();
            // error_vec = B_proj_mat[i] * cam_to_P_vec;

            Eigen::Vector3d cam_to_P_vec = prev_point - o_vec[i];
            double cam_to_P_vec_dist_squared = cam_to_P_vec.squaredNorm();

            Eigen::Vector3d error_vec = B_proj_mat[i] * cam_to_P_vec; // sine_op

            double weight_squared = 1.0 / cam_to_P_vec_dist_squared;
            
            double irmp_error = weight_squared * error_vec.squaredNorm();
            
            // e_i(p) = ||B_i * (p - o_i)||^2 / ||p - o_i||^2
            A_sum += B_proj_mat[i] * weight_squared;

            B_sum += weight_squared * ((B_proj_mat[i] * o_vec[i]) + irmp_error * cam_to_P_vec);
            // B_sum += weight_squared * ((B_proj_mat[i] * o_vec[i]) + irmp_error * cam_to_P_vec);
        }

        // pake yg mana ya? coba pake svd dulu, kalo ga bisa baru pake ldlt
        target_point = A_sum.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(B_sum);
        // target_point = A_sum.ldlt().solve(B_sum);
        
        // all finite
        if (!target_point.allFinite()) {
            target_valid = false;
            return false;
        }
        
        double update_norm = (target_point - prev_point).norm();
        if (update_norm < 1e-3) {  
            // RCLCPP_DEBUG(this->get_logger(), 
            //     "IRMP converged in %d iterations (delta=%.6f)", 
            //     iter, update_norm);
            break;
        }
    }
    
    // for (int i = 0; i < 3; ++i) {
    //     Eigen::Vector3d to_point = target_point - o_vec[i];
    //     double depth = to_point.dot(B_proj_mat[i]);
    //     if (depth < 0.0) {
    //         target_valid = false;
    //         return false;
    //     }
    // }
    
    target_valid = true;
    return true;
}

void CalcGPSNode::timerCallback(){

if (!dataCheck()) {
    return;
    }

if (!ready()) {
    return;
    }

projectionFormula();
// if (MVMP_triangulation()) {

if (IRMP_triangulation()) {
    RCLCPP_INFO(this->get_logger(),
    "D1: %.2f %.2f %.2f | D2: %.2f %.2f %.2f | D3: %.2f %.2f %.2f",
    drone1_pos_vec.x(), drone1_pos_vec.y(), drone1_pos_vec.z(),
    drone2_pos_vec.x(), drone2_pos_vec.y(), drone2_pos_vec.z(),
    drone3_pos_vec.x(), drone3_pos_vec.y(), drone3_pos_vec.z());

    RCLCPP_INFO(this->get_logger(),
    "proj_uvec1: %.3f %.3f %.3f | proj_uvec2: %.3f %.3f %.3f | proj_uvec3: %.3f %.3f %.3f",
    proj_uvec1.x(), proj_uvec1.y(), proj_uvec1.z(),
    proj_uvec2.x(), proj_uvec2.y(), proj_uvec2.z(),
    proj_uvec3.x(), proj_uvec3.y(), proj_uvec3.z());

    RCLCPP_INFO(this->get_logger(),
    "pixel1: u=%.1f v=%.1f | cam_mat diag: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
    pixel_vec1.x(), pixel_vec1.y(),
    cam_mat1(0,0), cam_mat1(1,1), cam_mat1(0,2), cam_mat1(1,2));

    // publish 
    geometry_msgs::msg::Point GPSmsg;
    GPSmsg.x = target_point.x();
    GPSmsg.y = target_point.y();
    GPSmsg.z = target_point.z();
    publisher_->publish(GPSmsg);
    
    // RCLCPP_INFO(this->get_logger(), 
    //     "Published target: x=%.2f, y=%.2f, z=%.2f", 
    //     GPSmsg.x, GPSmsg.y, GPSmsg.z);
} else {
    RCLCPP_WARN(this->get_logger(), "Triangulation failed");
    return;
}
}
  /*
    if (ready() && odom1_received && odom2_received && odom3_received) {
        projectionFormula();
        if (MVMP_triangulation()) {
            geometry_msgs::msg::Point GPSmsg;
            GPSmsg.x = target_point.x();
            GPSmsg.y = target_point.y();
            GPSmsg.z = target_point.z();
            publisher_->publish(GPSmsg);
            
            RCLCPP_INFO(this->get_logger(), "Published target: x=%.2f, y=%.2f, z=%.2f", 
                       GPSmsg.x, GPSmsg.y, GPSmsg.z);
        }
    }

    */

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalcGPSNode>());
    rclcpp::shutdown();
    return 0;
}


/*
    void timer_callback(){
        geometry_msgs::msg::Point GPSmsg;
        
        placeholder_1 = 0;
        placeholder_2 = 0;
        placeholder_3 = 0;

        GPSmsg.x = placeholder_1;
        GPSmsg.y = placeholder_2;
        GPSmsg.z = placeholder_3;

        publisher_->publish(GPSmsg);

        RCLCPP_INFO(
            this->get_logger(),
            "Published Point: x=%.2f y=%.2f z=%.2f",
            GPSmsg.x, GPSmsg.y, GPSmsg.z
        );

    }
    */

  