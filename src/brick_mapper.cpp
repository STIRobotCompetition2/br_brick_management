#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

#include <chrono>
#include <mutex>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>

#include <opencv2/opencv.hpp>



#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>



#include <eigen3/Eigen/Geometry>




#define DECAY_THRESH 1e-2
#define DECAY_FACTOR_ACTIVE 0.7
#define DECAY_FACTOR_PASSIVE 1-1e-9

#define DETECTION_THRESHOLD 0.2
#define DELETE_THRESHOLD 0.02
#define OBSTACLE_THRESHOLD 0.1
#define KERNEL_SIZE_X 0.5
#define KERNEL_SIZE_Y 0.2

#define MAP_FRAME "arena"

#define VALID_CODE_FREE 0
#define VALID_CODE_OCCUPIED_COLLECTED 1 
#define VALID_CODE_OCCUPIED_VALIDMAPPUB 2


// #include <tf2_ros/transform_listener.h>


// #include <tf2_eigen/tf2_eigen.hpp>

using namespace std::placeholders;

enum BrickMapperState{
    ACTIVE,
    PASSIVE
};

class BrickMapper : public rclcpp::Node {
    public:
    BrickMapper() : Node("brick_mapper") {
        brick_detector_array_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>("/marker_array", 5, std::bind(&BrickMapper::bufferMarkerArray, this, _1));
        brick_detector_single_sub = this->create_subscription<visualization_msgs::msg::Marker>("/marker_single", 5, std::bind(&BrickMapper::bufferMarkerSingle, this, _1));
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);


        process_map_timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&BrickMapper::updateMap, this));
        main_grid_map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("/brick_grid_map", 1);
        valid_grid_map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("/valid_grid_map", 1);
        binarized_result_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("/binarized_result/compressed", 1);
        floodfill_result_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("/floodfill_result/compressed", 1);
        state_pub = this->create_publisher<std_msgs::msg::String>("state", 1);
        grid_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", 5, std::bind(&BrickMapper::updateValidMap, this, _1));


        set_passive_client = this->create_client<std_srvs::srv::Trigger>("set_passive");
        set_active_server = this->create_service<std_srvs::srv::Trigger>("set_active", std::bind(&BrickMapper::setActiveCallback, this, _1, _2, _3));
        // set_active_server

        rclcpp::QoS map_qos(10);  // initialize to default
        if (true) {
            map_qos.transient_local();
            map_qos.reliable();
            map_qos.keep_last(1);
        }
        cost_map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/brick_cost_map", map_qos);

        map = std::make_shared<grid_map::GridMap>(grid_map::GridMap());
        map->setFrameId(MAP_FRAME);
        map->setGeometry(grid_map::Length(9.0,9.0), 0.02, Eigen::Vector2d(4.0, 4.0));
        RCLCPP_INFO(this->get_logger(), "Created map with size %f x %f m (%i x %i cells).",
            map->getLength().x(), map->getLength().y(),
            map->getSize()(0), map->getSize()(1)
        );
        map->add("bricks", grid_map::Matrix::Zero(map->getSize()(0), map->getSize()(1)));
        map->add("valid", VALID_CODE_OCCUPIED_VALIDMAPPUB * grid_map::Matrix::Ones(map->getSize()(0), map->getSize()(1)));
        map->add("obstacle", grid_map::Matrix::Zero(map->getSize()(0), map->getSize()(1)));
        
    }



    private:

    void setActiveCallback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if(this->state == BrickMapperState::ACTIVE){
            RCLCPP_ERROR(this->get_logger(), "Tried to set active but was already active");
            response->success = false;
            response->message = "Tried to set active but was already active";
            nav_msgs::msg::OccupancyGrid costmap;
            grid_map::GridMapRosConverter::toOccupancyGrid(*map, "bricks", 0, std::numeric_limits<double>::max(), costmap);
            costmap.header.frame_id = MAP_FRAME;
            costmap.header.stamp = this->get_clock()->now();
            for(size_t i = 0; i < 5; i++) cost_map_pub->publish(costmap);
        }
        else{
            this->state = BrickMapperState::ACTIVE;
            RCLCPP_DEBUG(this->get_logger(), "Set to active");
            response->success = true;
            response->message = "Set to active";
        }
    }

    bool updateObstacleMap(){
        for(grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it){
            grid_map::Position position;
            map->getPosition(*it, position);
            if(map->at("valid", *it) == VALID_CODE_OCCUPIED_COLLECTED) {
                map->at("obstacle", *it) = 0.0;
                continue;
            }
            if(state == BrickMapperState::PASSIVE){
                if(map->at("bricks", *it) > OBSTACLE_THRESHOLD) map->at("obstacle", *it) = OBSTACLE_THRESHOLD;
                else map->at("obstacle", *it) = 0.0;
            }
            else map->at("obstacle", *it) = 0.0;


            

        }
    }

    bool fuseValidMaps(const grid_map::GridMap to_process){
        for(grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it){
            grid_map::Position position;
            map->getPosition(*it, position);
            if(map->at("valid", *it) == VALID_CODE_OCCUPIED_COLLECTED) continue;
            if(to_process.atPosition("clear", position) < 2.5 && to_process.atPosition("clear", position) > 1.5){
                RCLCPP_DEBUG(this->get_logger(), "Fuse Valid Maps called with clear-region request");
                map->at("valid", *it) = VALID_CODE_OCCUPIED_COLLECTED;
            }
            else if(to_process.atPosition("clear", position) > 2.5){
                RCLCPP_DEBUG(this->get_logger(), "Fuse Valid Maps called with validmap-pub request");
                map->at("valid", *it) = VALID_CODE_OCCUPIED_VALIDMAPPUB;
            }
            else if(to_process.atPosition("clear", position) < 0.5){
                map->at("valid", *it) = VALID_CODE_FREE;
            }

        }
        RCLCPP_INFO(this->get_logger(), "Fuse Valid Maps");
        return true;

        
    }

    bool clearRegion(const Eigen::Vector2d seed = Eigen::Vector2d(3,1), const bool make_invalid = true){
        cv::Mat img_cv, img_processed1, img_processed2, img_processed3, img_processed4, img_processed5;

        grid_map::GridMapCvConverter::toImage<float, 1>(*this->map, static_cast<std::string>("bricks"), CV_32FC1, 0.0F, 1.0F, img_cv);
        // cv::Mat img_cv(img_cv_bridge.image);
        cv::threshold(img_cv, img_processed1, DELETE_THRESHOLD, 1.0, cv::THRESH_BINARY_INV);
        // cv::bitwise_not(img_processed1, img_processed1);
        // cv::imshow("Display window 1", img_processed1);

        cv_bridge::CvImage img_cv_bridge;
        img_cv_bridge.encoding = static_cast<std::string>("mono8");

        img_processed1.convertTo(img_cv, CV_8UC1);
        img_cv_bridge.image = 255 * img_cv;
        img_cv_bridge.header.stamp = this->get_clock()->now();

        // cv::imshow("Display window 2" , img_processed4);
        binarized_result_pub->publish(*img_cv_bridge.toCompressedImageMsg());
        // grid_map::getIndexFromPosition
        size_t seed_x = static_cast<size_t>(img_processed1.size().width * (1.0 - (seed.y() + 0.5) / this->map->getLength().y()));
        size_t seed_y = static_cast<size_t>(img_processed1.size().height * (1.0 - (seed.x() + 0.5) / this->map->getLength().x()));

        // RCLCPP_INFO(this->get_logger(), "X: %u, Y: %u", seed_x, seed_y);

        cv::floodFill(img_processed1, cv::Point(seed_x, seed_y), 0.5);

        cv::threshold(img_processed1, img_processed2, 0.2, true, cv::THRESH_BINARY);
        cv::threshold(img_processed1, img_processed3, 0.7, true, cv::THRESH_BINARY_INV);
    

        cv::bitwise_and(img_processed2, img_processed3, img_processed4);
        // cv::morphologyEx(img_processed3, img_processed4, cv::MORPH_DILATE, cv::Disk)


        img_processed5 = img_processed4;

        // cv::drawMarker(img_processed5, cv::Point(seed_x, seed_y), 0.5, cv::MARKER_CROSS);


        
        
        img_processed5.convertTo(img_cv, CV_8UC1);
        img_cv_bridge.image = 255 * img_cv;
        img_cv_bridge.header.stamp = this->get_clock()->now();

        // cv::imshow("Display window 2" , img_processed4);
        floodfill_result_pub->publish(*img_cv_bridge.toCompressedImageMsg());
        if(cv::sum(img_processed4)[0] > 0.5 * img_processed4.size().height * img_processed4.size().width){
            RCLCPP_ERROR(this->get_logger(), "Seed for flood-fill was ill-posed. Filled area > 50 percent of whole area ...");
            return false;
        }
        else{

            grid_map::GridMap gm_temp;
            gm_temp.setFrameId(MAP_FRAME);
            gm_temp.setGeometry(grid_map::Length(9.0,9.0), 0.02, Eigen::Vector2d(4.0, 4.0));
            grid_map::GridMapCvConverter::addLayerFromImage<float,1>(img_processed4 + 1.0, "clear", gm_temp);
            for(grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it){
                grid_map::Position position;
                map->getPosition(*it, position);
                if(gm_temp.atPosition("clear", position) > 1.5){
                    map->at("bricks", *it) = 0.0;
                }
            }
            fuseValidMaps(gm_temp);
            return true;
        }


    }

    bool addKernel(const Eigen::Vector2d position, const double dx, const double dy, const double yaw = 0.0, const double height = 1.0){

        if(! map->isInside(grid_map::Position(position.x(),position.y()))) return false;
        if(! map->isInside(grid_map::Position(position.x(),position.y()))) return false;


        Eigen::Matrix2d projection;
        projection << std::cos(yaw), std::sin(yaw), -std::sin(yaw), std::cos(yaw);
        
        Eigen::Matrix<double, 2, 4> bounds;
        bounds.col(0) = projection.row(0).transpose() * dx + projection.row(1).transpose() * dy;
        bounds.col(1) = projection.row(0).transpose() * dx - projection.row(1).transpose() * dy;
        bounds.col(2) = -projection.row(0).transpose() * dx + projection.row(1).transpose() * dy;
        bounds.col(3) = -projection.row(0).transpose() * dx - projection.row(1).transpose() * dy;

        

        for(double x = bounds.row(0).minCoeff(); x <= bounds.row(0).maxCoeff(); x+=map->getResolution() * 0.999)
        for(double y = bounds.row(1).minCoeff(); y <= bounds.row(1).maxCoeff(); y+=map->getResolution() * 0.999){

            grid_map::Position map_position(x + position.x(), y + position.y());

            if(! map->isInside(map_position)){
                continue;
            }
            Eigen::Vector2d diff(x, y);
            diff = projection * diff;
            if(std::fabs(diff.x()) > dx || std::fabs(diff.y()) > dy) continue;

            map->atPosition("bricks", map_position) += height * std::cos(std::fabs(diff.x()) / dx * M_PI_2) * std::cos(std::fabs(diff.y()) / dy * M_PI_2);
        }
        return true;

         


    }

    void updateValidMap(std::shared_ptr<nav_msgs::msg::OccupancyGrid> valid_region_occ_grid){
        grid_map::GridMap gm_temp;
        gm_temp.setFrameId(MAP_FRAME);
        gm_temp.setGeometry(grid_map::Length(9.0,9.0), 0.02, Eigen::Vector2d(4.0, 4.0));
        gm_temp.add("clear", VALID_CODE_OCCUPIED_VALIDMAPPUB * grid_map::Matrix::Ones(map->getSize()(0), map->getSize()(1)));
        grid_map::GridMapRosConverter::fromOccupancyGrid(*valid_region_occ_grid, "clear", gm_temp);
        fuseValidMaps(gm_temp);

        
    }

    void updateMap(){
        geometry_msgs::msg::TransformStamped t;
        Eigen::Isometry3d t_eigen;
        Eigen::Vector4d position;

        marker_buffer_lock.lock();

        for(visualization_msgs::msg::Marker m : this->marker_buffer){
            try {
                t = tf_buffer->lookupTransform(MAP_FRAME, m.header.frame_id, this->get_clock()->now());
            } 
            catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    m.header.frame_id.c_str(), MAP_FRAME, ex.what());
                continue;
            }

            t_eigen = tf2::transformToEigen(t);
            position = t_eigen.matrix() * Eigen::Vector4d(m.pose.position.x, m.pose.position.y, 0.0, 1.0);

            if(!(m.type == m.SPHERE || m.type == m.CUBE || m.type == m.CYLINDER)) {
                RCLCPP_WARN(this->get_logger(), "Receiver marker of type %u but need %u, %u, or %u. Continuing, but gridmap may be compromised ...", m.type, m.SPHERE, m.CUBE, m.CYLINDER);
            }
            
            if(map->atPosition("valid", grid_map::Position(position.x(), position.y())) > 0.5) continue;

            if(addKernel(Eigen::Vector2d(position.x(), position.y()) ,m.scale.x, m.scale.y, 2 * std::atan2(t.transform.rotation.z, t.transform.rotation.w), m.scale.z))
                RCLCPP_DEBUG(this->get_logger(), "Added kernel at (%f,%f) with kernel size (%f,%f)", m.pose.position.x, m.pose.position.y, m.scale.x, m.scale.y);
            else
                RCLCPP_ERROR(this->get_logger(), "Could not add kernel at (%f,%f) with kernel size (%f,%f)", m.pose.position.x, m.pose.position.y, m.scale.x, m.scale.y);
        }
        this->marker_buffer.clear();
        marker_buffer_lock.unlock();
        double max_value = std::numeric_limits<double>::min();
        grid_map::Index max_index;
        for(grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it){
            map->at("bricks", *it) *= state == BrickMapperState::ACTIVE ? DECAY_FACTOR_ACTIVE : DECAY_FACTOR_PASSIVE;
            if(map->at("bricks", *it) < DECAY_THRESH) map->at("bricks", *it) = 0.0;
            if(map->at("bricks", *it) > max_value){
                std::vector<std::string> keys = map->getLayers();
                if(keys.size() != 3u){
                    RCLCPP_FATAL(this->get_logger(), "Valid-Map consists of %lu layers (must be 2) !", keys.size());
                    throw std::runtime_error("Invalid configuration of internal state");
                }
                else{
                    grid_map::Position max_candidate_position;
                    map->getPosition(*it, max_candidate_position);
                    float value = map->atPosition("valid", max_candidate_position);
                    if(value > 0.5F) continue;
                }
                
                max_value = map->at("bricks", *it);
                max_index = *it;    
            }
        }
        if(this->state == BrickMapperState::ACTIVE){
            if(max_value > DETECTION_THRESHOLD){
                grid_map::Position max_position;
                this->map->getPosition(max_index, max_position);
                float value = map->atPosition("valid", max_position);
                if(value > 0.5F) {
                    RCLCPP_WARN(this->get_logger(), "Found maximum brick confidence at %f,%f which is an illegal position. Skipping ...");
                }
                else{
                    RCLCPP_INFO(this->get_logger(), "Found valid maximum at %f %f! Checking if path is free ...", max_position.x(), max_position.y());
                    RCLCPP_ERROR(this->get_logger(), "TODO");
                    bool is_path_free = true;
                    if(is_path_free){
                        RCLCPP_INFO(this->get_logger(), "Path is free ! Forwarding to motion control node ...");
                        RCLCPP_ERROR(this->get_logger(), "TODO");
                        // INTERRUPT 
                        this->state = BrickMapperState::PASSIVE;

                    }
                    else{
                        RCLCPP_WARN(this->get_logger(), "Path is NOT free ! Doing nothing ...", max_position.x(), max_position.y());
                    }

                    clearRegion(Eigen::Vector2d(max_position.x(), max_position.y()));
                }
            }
        }
        updateObstacleMap();

        nav_msgs::msg::OccupancyGrid costmap;
        grid_map::GridMapRosConverter::toOccupancyGrid(*map, "obstacle", 0, OBSTACLE_THRESHOLD, costmap);
        costmap.header.frame_id = MAP_FRAME;
        costmap.header.stamp = this->get_clock()->now();
        cost_map_pub->publish(costmap);
    
        this->main_grid_map_pub->publish(grid_map::GridMapRosConverter::toMessage(*this->map));
        this->state_pub->publish(state2stringmsg());
    }
    void bufferMarkerArray(std::shared_ptr<visualization_msgs::msg::MarkerArray> brick_detections){
        marker_buffer_lock.lock();
        marker_buffer.insert(marker_buffer.end(), brick_detections->markers.begin(), brick_detections->markers.end());
        marker_buffer_lock.unlock();
    }
    void bufferMarkerSingle(std::shared_ptr<visualization_msgs::msg::Marker> brick_detection){
        marker_buffer_lock.lock();
        marker_buffer.push_back(*brick_detection);
        marker_buffer_lock.unlock();
    }

    std_msgs::msg::String state2stringmsg(){
        std::string state_str;
        switch(this->state){
            case BrickMapperState::ACTIVE:
            state_str = std::string("active");
            break;
            case BrickMapperState::PASSIVE:
            state_str = std::string("passive");
            break;
            default:
            throw std::runtime_error("Brick mapper found unknown state");
        }
        std_msgs::msg::String out_msg;
        out_msg.data = state_str;
        return out_msg;



    }

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr brick_detector_array_sub;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr brick_detector_single_sub;

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr main_grid_map_pub, valid_grid_map_pub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_pub;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr binarized_result_pub, floodfill_result_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
    BrickMapperState state = BrickMapperState::ACTIVE;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_active_server;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr set_passive_client;


    rclcpp::TimerBase::SharedPtr process_map_timer;
    std::shared_ptr<grid_map::GridMap> map;
    std::vector<visualization_msgs::msg::Marker> marker_buffer;
    std::mutex marker_buffer_lock;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    BrickMapper::SharedPtr bm(new BrickMapper());
    rclcpp::spin(bm);
}
