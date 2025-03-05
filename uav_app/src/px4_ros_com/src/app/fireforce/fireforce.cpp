
/**
* @brief UAV Command node
* @file uav_app.cpp
* @author Lohit Muralidharan
*/


#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <curl/curl.h>
#include <iostream>
#include <fstream>
#include <curl/curl.h>
#include <memory>  
#include <stdint.h>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class HTTPPOSTServer : public rclcpp::Node {
    public:
        // Node constructor
        explicit HTTPPOSTServer();
        // HTTP Rest API
        void post_frame();

    private: 

        rclcpp::TimerBase::SharedPtr timer_;	
        std::atomic<uint64_t> timestamp_;   	//!< common synced timestamped
        uint64_t offboard_setpoint_counter_;    //!< counter for the number of setpoints sent
        CURL* curl;
        CURLcode res;

	std::string supabase_url = ""; // TODO: Fill out information
	std::string key = "";	       // TODO: Fill out information
};

HTTPPOSTServer::HTTPPOSTServer() : Node("HTTP_SERVER") {

    timer_ = this->create_wall_timer(300ms, std::bind(&HTTPPOSTServer::post_frame, this));
    
}

size_t read_callback(void *ptr, size_t size, size_t nmemb, void *data) {
    std::ifstream *file = static_cast<std::ifstream *>(data);
    file->read(reinterpret_cast<char *>(ptr), size * nmemb); 
    return file->gcount(); // Return the number of bytes read
}

void HTTPPOSTServer::post_frame() {
    
    // RCLCPP_INFO(this->get_logger(), "=================FRAME POSTED================");
    // curl = curl_easy_init();
    // if (curl) {
    //     std::ifstream file("src/px4_ros_com/src/app/fireforce/images/cv_frame.jpg", std::ios::binary);
    //     if (!file.is_open()) {
    //         std::cerr << "Error opening file!" << std::endl;
    //         return;
    //     }

    //     struct curl_slist *headers = nullptr;
    //     headers = curl_slist_append(headers, "Authorization: Bearer " + key);
    //     headers = curl_slist_append(headers, "Content-Type: image/jpg");

    //     curl_easy_setopt(curl, CURLOPT_URL, supabase_url + "/storage/v1/object/UAVFrame/cv_frame.jpg");
    //     curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    //     curl_easy_setopt(curl, CURLOPT_READFUNCTION, read_callback);
    //     curl_easy_setopt(curl, CURLOPT_READDATA, &file);
    //     curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

    //     res = curl_easy_perform(curl);

    //     if (res != CURLE_OK) 
    //         std::cerr << "POST REQUEST FAILED" << std::endl;

    //     curl_easy_cleanup(curl);
    //     curl_slist_free_all(headers);
    //     file.close();
    // }

    // RCLCPP_INFO(this->get_logger(), "=================SENSOR POSTED================");
    // curl = curl_easy_init();
    // if (curl) {

    //     std::ifstream file("src/px4_ros_com/src/app/fireforce/data/data.txt");
    //     if (!file.is_open()) {
    //         std::cerr << "Error opening file!" << std::endl;
    //         return;
    //     }

    //     struct curl_slist *headers = nullptr;
    //     headers = curl_slist_append(headers, "Authorization: Bearer " + key);
    //     headers = curl_slist_append(headers, "Content-Type: text/plain");

    //     curl_easy_setopt(curl, CURLOPT_URL, supabase_url + "storage/v1/object/UAVFrame/sensor_msg.txt");
    //     curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    //     curl_easy_setopt(curl, CURLOPT_READFUNCTION, read_callback);
    //     curl_easy_setopt(curl, CURLOPT_READDATA, &file);
    //     curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

    //     res = curl_easy_perform(curl);

    //     if (res != CURLE_OK) 
    //         std::cerr << "POST REQUEST FAILED" << std::endl;

    //     curl_easy_cleanup(curl);
    //     curl_slist_free_all(headers);
    //     file.close();
    // }
}



class UAVCommand : public rclcpp::Node {
    public:
        // Node constructor
        explicit UAVCommand();

        // Subscriber Callbacks
        void read_local_position(const VehicleLocalPosition::UniquePtr msg);
        void read_camera_image_raw(const sensor_msgs::msg::Image::SharedPtr msg);
        void read_gps(const VehicleGlobalPosition::UniquePtr msg);
        void read_imu(const SensorCombined::UniquePtr msg);

        // Publisher Callbacks
        void publish_offboard_control_mode(bool pos, bool vel);
        void publish_trajectory_setpoint(float x, float y, float z, float yaw);
        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

        // Offboard Control Mode
        void arm();
        void disarm();

        // Run Mission
        void run_mission();

        // Post Frame
        void post_frame();

        // Helpers
        void print_data();
        void write_file();

    private:

        enum direction { UP, DOWN };

        typedef struct VehicleState {
            float x;
            float y;
            float z;
            float roll;
            float pitch;
            float yaw;
            float vx;
            float vy;
            float vz;
            float waypoint_x;
            float waypoint_y;
            float waypoint_z;
            float waypoint_yaw;
            direction dir;
            int64_t ms;
        } VehicleState;

        typedef struct SensorMsg {
            float roll;
            float pitch;
            float yaw;
            float lon;
            float lat;
            float alt;
        } SensorMsg;

        rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr global_position_subscriber;
        rclcpp::Subscription<SensorCombined>::SharedPtr sensor_combined_subscriber;
        rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscriber;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_raw_subscriber;

        rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher;
        rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher;
        
        rclcpp::TimerBase::SharedPtr timer_;	
        std::atomic<uint64_t> timestamp_;   	//!< common synced timestamped
        uint64_t offboard_setpoint_counter_;    //!< counter for the number of setpoints sent

        VehicleState vehicle_state;
        float tolerarance = 0.75;
        float peak_altitude = -40.0;
        float pi = 3.14;

        cv::dnn::Net net;
        float thresh = 0.5;
        float nms_thresh = 0.45; 
        float marker_x;
        float marker_y;

        CURL* curl;
        CURLcode res;
        std::thread frame_thread;
        std::mutex frame_mutex;

        SensorMsg sensor_msg;

};


/**
* @brief UAV Command node constructor
* @link 
*/
UAVCommand::UAVCommand() : Node("uav_command") {

    std::cout << "=============================Instantiating UAV Command Node=============================" << std::endl;

    net = cv::dnn::readNetFromONNX("src/px4_ros_com/src/app/fireforce/model/best.onnx");

    rmw_qos_profile_t qos_profile_pos = rmw_qos_profile_sensor_data;
    rmw_qos_profile_t qos_profile_camera = rmw_qos_profile_sensor_data;

    rmw_qos_profile_t qos_profile_pos2 = rmw_qos_profile_sensor_data;
    rmw_qos_profile_t qos_profile_imu = rmw_qos_profile_sensor_data;

    rclcpp::QoS qos_pos(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5), qos_profile_pos);
    rclcpp::QoS qos_camera(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5), qos_profile_camera);

    rclcpp::QoS qos_pos2(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5), qos_profile_pos2);
    rclcpp::QoS qos_imu(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5), qos_profile_imu);

    sensor_msg.roll = 0.0;
    sensor_msg.pitch = 0.0;
    sensor_msg.yaw = 0.0;
    sensor_msg.lon = 0.0;
    sensor_msg.lat = 0.0;
    sensor_msg.alt = 0.0;

    vehicle_state.waypoint_x = 0;
    vehicle_state.waypoint_y = 0;
    vehicle_state.waypoint_z = 0;
    vehicle_state.ms = 0;

    global_position_subscriber = this->create_subscription<VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position", 
        qos_pos2,
        std::bind(&UAVCommand::read_gps, this, std::placeholders::_1)
    );

    sensor_combined_subscriber = this->create_subscription<SensorCombined>(
        "/fmu/out/sensor_combined", 
        qos_imu,
        std::bind(&UAVCommand::read_imu, this, std::placeholders::_1)
    );

    local_position_subscriber = this->create_subscription<VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", 
        qos_pos,
        std::bind(&UAVCommand::read_local_position, this, std::placeholders::_1)
    );

    camera_image_raw_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 
        qos_camera,
        std::bind(&UAVCommand::read_camera_image_raw, this, std::placeholders::_1)
    );

    offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    offboard_setpoint_counter_ = 0;
    
    timer_ = this->create_wall_timer(300ms, std::bind(&UAVCommand::run_mission, this));

}

/**
* @brief Debug
*/
void UAVCommand::print_data() {
    std::cout << "Longitude: " << sensor_msg.lon << std::endl;
    std::cout << "Latitude: " <<  sensor_msg.lat << std::endl;
    std::cout << "Altitude: " <<  sensor_msg.alt << std::endl;
    std::cout << "Roll: " <<  sensor_msg.roll << std::endl;
    std::cout << "Pitch: " <<  sensor_msg.pitch << std::endl;
    std::cout << "Yaw: " <<  sensor_msg.yaw << std::endl;
}

/**
* @brief Write to data file
*/
void UAVCommand::write_file() {
    std::ofstream file("/home/lohitoburrito/uav_app/src/px4_ros_com/src/app/fireforce/data/data.txt");
    if (!file) {
        std::cerr << "Error opening the file!" << std::endl;
        return;
    }

    file << sensor_msg.roll << "\n";
    file << sensor_msg.pitch << "\n";
    file << sensor_msg.yaw << "\n";
    file << sensor_msg.lon << "\n";
    file << sensor_msg.lat << "\n";
    file << sensor_msg.alt << "\n";

    file.close();
    // print_data(); 
}


/**
* @brief UAV Command Global Position Subscriber Callback
* @param msg Vehicle Global Position message pointer
* @link 
*/
void UAVCommand::read_gps(const VehicleGlobalPosition::UniquePtr msg) {
    sensor_msg.lon = msg->lon;
    sensor_msg.lat = msg->lat;
    sensor_msg.alt = msg->alt; 
    write_file();
}

/**
* @brief UAV Command Sensor Combined Subscriber Callback
* @param msg Vehicle Sensor Combined message pointer
* @link 
*/
void UAVCommand::read_imu(const SensorCombined::UniquePtr msg) {
    float accel_x = msg->accelerometer_m_s2[0];
    float accel_y = msg->accelerometer_m_s2[1];
    float accel_z = msg->accelerometer_m_s2[2];

    sensor_msg.roll = std::atan2(accel_y, std::sqrt(std::pow(accel_x, 2) + std::pow(accel_z, 2)));
    sensor_msg.pitch = std::atan2(-accel_x, std::sqrt(std::pow(accel_y, 2) + std::pow(accel_z, 2)));
    // sensor_msg.yaw += msg->gyro_rad[2];    
    write_file();   
}

/**
* @brief UAV Command Local Position Subscriber Callback
* @param msg Vehicle Local Position message pointer
* @link 
*/
void UAVCommand::read_local_position(const VehicleLocalPosition::UniquePtr msg) {
    vehicle_state.x = msg->x;
    vehicle_state.y = msg->y;
    vehicle_state.z = msg->z;
    vehicle_state.vx = msg->vx;
	vehicle_state.vy = msg->vy;
	vehicle_state.vz = msg->vz;
    vehicle_state.yaw = std::atan2(msg->vy, msg->vx);
    // std::cout << vehicle_state.x << " " << vehicle_state.y << " " << vehicle_state.z << std::endl;
}

/**
* @brief UAV Command Camera Subscriber Callback
* @param msg Camera Image Raw message pointer
* @link 
*/
void UAVCommand::read_camera_image_raw(const sensor_msgs::msg::Image::SharedPtr msg) {

    try {

        RCLCPP_INFO(this->get_logger(), "===============IMAGE RETRIEVED===============");
        cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

        cv::Mat resized_frame;
        cv::resize(frame, resized_frame, cv::Size(640, 640));

        cv::Mat blob = cv::dnn::blobFromImage(resized_frame, 1.0/255.0, cv::Size(640, 640), cv::Scalar(0, 0, 0), true, false);

        net.setInput(blob);

        cv::Mat output = net.forward();

        // Run Thresholding
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;

        for (int i = 0; i < output.size[1]; ++i) {
            float score = output.at<float>(0, i, 4);
            if (score > thresh) {
                int classId = output.at<float>(0, i, 5) > output.at<float>(0, i, 6) ? 5 : 6;
    
                float cx = output.at<float>(0, i, 0);
                float cy = output.at<float>(0, i, 1);
                float w = output.at<float>(0, i, 2);
                float h = output.at<float>(0, i, 3);

                int x = static_cast<int>(cx - w / 2);
                int y = static_cast<int>(cy - h / 2);
    
                boxes.push_back(cv::Rect(x, y, static_cast<int>(w), static_cast<int>(h)));
                scores.push_back(output.at<float>(0, i, classId)); // Use class score
            }
        }
    
        // Run NMS
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, thresh, nms_thresh, indices);

        if (indices.size() == 0)
            marker_x = marker_y = -1;

        for (int idx : indices) {

            int id = static_cast<int>(output.at<float>(0, idx, 5));
            std::string label = (id == 0) ? "fire" : "empty";
            int label_x = boxes[idx].x;
            int label_y = boxes[idx].y - 10; 
            cv::putText(resized_frame, label, cv::Point(label_x, label_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            cv::rectangle(resized_frame, boxes[idx], cv::Scalar(0, 255, 0), 2);

            if (id == 0) {	

                marker_x = boxes[idx].x + boxes[idx].width  / 2;
                marker_y = boxes[idx].y + boxes[idx].height / 2;

                vehicle_state.ms = -1;

                break;
            }

        }

        // Debug output
        cv::imwrite("src/px4_ros_com/src/app/fireforce/images/cv_frame.jpg", resized_frame);

    } catch (cv_bridge::Exception& e) {

        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());

    }
}

/**
* @brief UAV Command Offboard Control Publisher
* @link 
*/
void UAVCommand::publish_offboard_control_mode(bool pos, bool vel) {
    OffboardControlMode msg{};
    msg.position = pos;
    msg.velocity = vel;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher->publish(msg);
}

/**
* @brief UAV Command Offboard Trajectory Publisher
* @param x x (m)
* @param y y (m)
* @param z altitude (m)
* @param yaw yaw angle (rad)
* @link 
*/
void UAVCommand::publish_trajectory_setpoint(float x, float y, float z, float yaw) {
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.yaw = yaw; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher->publish(msg);
}

/**
* @brief UAV Vehicle Command Publisher
* @param command   Command code
* @param param1    Command parameter 1
* @param param2    Command parameter 2
* @link 
*/
void UAVCommand::publish_vehicle_command(uint16_t command, float param1, float param2) {
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher->publish(msg);
}

/**
* @brief Send a command to Arm the vehicle
*/
void UAVCommand::arm() { 
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
* @brief Send a command to Disarm the vehicle
*/
void UAVCommand::disarm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
* @brief UAV Mission Checkpoints
*   1. [ms = -1] -> abort mission and land at coordinates (0m, 2m, 0m)
*	2. [ms = 0] -> Arm the vehicle and set offboard control mode
*	3. [ms = 1] -> Get to coordinate (0m, 0m, -15m)
* 	4. [ms = 2] -> 
*/
void UAVCommand::run_mission() {

    auto dist = [] (float x1, float y1, float z1, float x2, float y2, float z2) {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
    };

    switch (vehicle_state.ms) {

        case -2:

            RCLCPP_INFO(this->get_logger(), "================MISSION ABORT================");
            
            vehicle_state.waypoint_x = 0.0;
            vehicle_state.waypoint_y = 2.0;
            vehicle_state.waypoint_z = 0.0;

            publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);

            if (vehicle_state.z >= 0) {
                disarm();
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION, 1);
            }

            break;

        case -1:

            RCLCPP_INFO(this->get_logger(), "================FIRE DETECTED================");

            if (vehicle_state.dir == UP) 
                vehicle_state.waypoint_x = vehicle_state.x + ((std::sqrt(2.0) / 2.0) * ((marker_x - 320.0) / 320.0));
            else
                vehicle_state.waypoint_x = vehicle_state.x - ((std::sqrt(2.0) / 2.0) * ((marker_x - 320.0) / 320.0));

            vehicle_state.waypoint_y = vehicle_state.y + ((std::sqrt(2.0) / 2.0) * ((marker_x - 320.0) / 320.0));
			vehicle_state.waypoint_z = vehicle_state.z - ((320.0 - marker_y) / 320.0);

            break;

        case 0:

            RCLCPP_INFO(this->get_logger(), "==================MISSION 1==================");

            if (offboard_setpoint_counter_ == 10) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                arm();
            }

            vehicle_state.waypoint_x = 0.0;
            vehicle_state.waypoint_y = 0.0;
            vehicle_state.waypoint_z = -5.0;
        
            if (offboard_setpoint_counter_ < 11)
                offboard_setpoint_counter_++;
            
            break;

        case 1:
            
            RCLCPP_INFO(this->get_logger(), "==================MISSION 2==================");

            vehicle_state.waypoint_x = 5.0;
            vehicle_state.waypoint_y = 12.5;
            vehicle_state.waypoint_z = -5.0;
            vehicle_state.waypoint_yaw = -pi / 4;

            break;

        case 2:

            RCLCPP_INFO(this->get_logger(), "==================MISSION 3==================");

            vehicle_state.waypoint_x = 5.0;
            vehicle_state.waypoint_y = 12.5;
            vehicle_state.waypoint_z = peak_altitude;
            vehicle_state.waypoint_yaw = -pi / 4;
            vehicle_state.dir = UP;

            break;

        case 3:

            RCLCPP_INFO(this->get_logger(), "==================MISSION 4==================");

            vehicle_state.waypoint_x = 0.0;
            vehicle_state.waypoint_y = 0.0;
            vehicle_state.waypoint_z = peak_altitude;
            vehicle_state.waypoint_yaw = 0.0;

            break;

        case 4:

            RCLCPP_INFO(this->get_logger(), "==================MISSION 5==================");

            vehicle_state.waypoint_x = 5.0;
            vehicle_state.waypoint_y = -12.5;
            vehicle_state.waypoint_z = peak_altitude;
            vehicle_state.waypoint_yaw = pi / 4;

            break;
        
        case 5:

            RCLCPP_INFO(this->get_logger(), "==================MISSION 6==================");

            vehicle_state.waypoint_x = 5.0;
            vehicle_state.waypoint_y = -12.5;
            vehicle_state.waypoint_z = -5.0;
            vehicle_state.waypoint_yaw = pi / 4;
            vehicle_state.dir = DOWN;

            break;

        default:
        
            break;
            
    }

    if (vehicle_state.ms >= 0 && dist(vehicle_state.x, vehicle_state.y, vehicle_state.z, vehicle_state.waypoint_x, vehicle_state.waypoint_y, vehicle_state.waypoint_z) <= tolerarance)
        vehicle_state.ms++;
    
    if (vehicle_state.ms == 6)
        vehicle_state.ms = -2;


    // std::cout << "Waypoint X: " << vehicle_state.waypoint_x << std::endl;
    // std::cout << "Waypoint Y: " << vehicle_state.waypoint_y << std::endl;
    // std::cout << "Waypoint Z: " << vehicle_state.waypoint_z << std::endl;
    // std::cout << "Current X: " << vehicle_state.x << std::endl;
    // std::cout << "Current Y: " << vehicle_state.y << std::endl;
    // std::cout << "Current Z: " << vehicle_state.z << std::endl;
    sensor_msg.yaw = vehicle_state.waypoint_yaw * 180.0 / pi;
    publish_offboard_control_mode(true, false);
    publish_trajectory_setpoint(vehicle_state.waypoint_x, vehicle_state.waypoint_y, vehicle_state.waypoint_z, vehicle_state.waypoint_yaw);
    
}


int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<UAVCommand>());


    auto uav_command = std::make_shared<UAVCommand>();
    auto http_server = std::make_shared<HTTPPOSTServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(uav_command);
    executor.add_node(http_server);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}

