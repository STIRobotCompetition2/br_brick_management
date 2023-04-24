#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <chrono>
#include <mutex>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"




#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>



#include <eigen3/Eigen/Geometry>




#define DECAY 0.1
#define DETECTION_THRESHOLD 0.2
#define KERNEL_SIZE_X 0.5
#define KERNEL_SIZE_Y 0.2

#define MAP_FRAME "arena"

// #include <tf2_ros/transform_listener.h>


// #include <tf2_eigen/tf2_eigen.hpp>

using std::placeholders::_1;


class BrickMapper : public rclcpp::Node {
    public:
    BrickMapper() : Node("brick_mapper") {
        brick_detector_array_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>("/marker_array", 5, std::bind(&BrickMapper::bufferMarkerArray, this, _1));
        brick_detector_single_sub = this->create_subscription<visualization_msgs::msg::Marker>("/marker_single", 5, std::bind(&BrickMapper::bufferMarkerSingle, this, _1));
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        process_map_timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&BrickMapper::updateMap, this));
        grid_map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("/brick_grid_map", 1);
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
         map->getSize()(0), map->getSize()(1));
        map->add("bricks", grid_map::Matrix::Zero(map->getSize()(0), map->getSize()(1)));
        // map->atPosition("bricks", grid_map::Position(1,1)) = 5;
        addKernel(Eigen::Vector2d(3,3), 0.5, 1);
        


        
    }





    private:
    bool addKernel(const Eigen::Vector2d position, const double dx, const double dy, const double yaw = 0.0, const double height = 1.0){
        // grid_map::Index start_idx, end_idx;
        // map->getIndex(grid_map::Position(position + Eigen::Vector2d(-2*vx, -2*vy)), start_idx);
        // map->getIndex(grid_map::Position(position + Eigen::Vector2d(2*vx, 2*vy)), end_idx);
        
        // size_t i = 0;
        // for (grid_map::SubmapIterator it(*map, start_idx, start_idx - end_idx); !it.isPastEnd(); ++it) {
        //     std::cerr << "A" << i << std::endl;
        //     std::cerr << "B" << start_idx << std::endl;
        //     std::cerr << "C" << end_idx << std::endl;

        //     i++;
        //     map->at("bricks", *it) = 1.0;
        // }


        if(! map->isInside(grid_map::Position(position.x(),position.y()))) return false;

        Eigen::Matrix2d projection;
        projection << std::cos(yaw), std::sin(yaw), -std::sin(yaw), std::cos(yaw);
        
        Eigen::Matrix<double, 2, 4> bounds;
        bounds.col(0) = projection.row(0).transpose() * dx + projection.row(1).transpose() * dy;
        bounds.col(1) = projection.row(0).transpose() * dx - projection.row(1).transpose() * dy;
        bounds.col(2) = -projection.row(0).transpose() * dx + projection.row(1).transpose() * dy;
        bounds.col(3) = -projection.row(0).transpose() * dx - projection.row(1).transpose() * dy;
        std::cerr << bounds << std::endl << std::endl;

        

        for(double x = bounds.row(0).minCoeff(); x <= bounds.row(0).maxCoeff(); x+=map->getResolution() * 0.999)
        for(double y = bounds.row(1).minCoeff(); y <= bounds.row(1).maxCoeff(); y+=map->getResolution() * 0.999){
            // double exponent = -1 * (Eigen::Vector2d(x, y) - position).cwiseProduct(Eigen::Vector2d(x, y) - position).dot(Eigen::Vector2d(1/vx, 1/vy));
            // map->atPosition("bricks", grid_map::Position(x,y)) = std::exp(exponent);
            RCLCPP_INFO(this->get_logger(), "Hi %f, %f", x, y);

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
    void updateMap(){
        geometry_msgs::msg::TransformStamped t;
        Eigen::Isometry3d t_eigen;
        Eigen::Vector4d position;

        


        marker_buffer_lock.lock();

        for(visualization_msgs::msg::Marker m : this->marker_buffer){

            // Look up for the transformation between target_frame and turtle2 frames
            // and send velocity commands for turtle2 to reach target_frame
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
            std::cerr << "Q" << t_eigen.rotation() << std::endl << std::endl;
            position = t_eigen.matrix() * Eigen::Vector4d(m.pose.position.x, m.pose.position.y, 0.0, 1.0);

            

            if(addKernel(Eigen::Vector2d(position.x(), position.y()) ,KERNEL_SIZE_X, KERNEL_SIZE_Y, 2 * std::atan2(t.transform.rotation.z, t.transform.rotation.w)))
                RCLCPP_DEBUG(this->get_logger(), "Added kernel at (%f,%f) with kernel size (%f,%f)", m.pose.position.x, m.pose.position.y, KERNEL_SIZE_X, KERNEL_SIZE_Y);
            else
                RCLCPP_WARN(this->get_logger(), "Could not add kernel at (%f,%f) with kernel size (%f,%f)", m.pose.position.x, m.pose.position.y, KERNEL_SIZE_X, KERNEL_SIZE_Y);

        }
        this->marker_buffer.clear();
        marker_buffer_lock.unlock();
        double max_value = std::numeric_limits<double>::min();
        grid_map::Index max_index;
        for(grid_map::GridMapIterator it(*map); !it.isPastEnd(); ++it){
            map->at("bricks", *it) = std::max(0.0, map->at("bricks", *it) - DECAY);
            if(map->at("bricks", *it) > max_value){
                max_value = map->at("bricks", *it);
                max_index = *it;    
            }
        }
        if(max_value > DETECTION_THRESHOLD){
            grid_map::Position max_position;
            this->map->getPosition(max_index, max_position);
            RCLCPP_INFO(this->get_logger(), "Found valid maximum at %f %f", max_position.x(), max_position.y());
            // for(grid_map::CircleIterator it(*map, max_position, 0.3); !it.isPastEnd(); ++it){
            //     map->at("bricks", *it) = 0.0;

            // }
        }
        nav_msgs::msg::OccupancyGrid costmap;
        grid_map::GridMapRosConverter::toOccupancyGrid(*map, "bricks", 0, DETECTION_THRESHOLD, costmap);
        costmap.header.frame_id = MAP_FRAME;
        costmap.header.stamp = this->get_clock()->now();

        cost_map_pub->publish(costmap);

        


        this->grid_map_pub->publish(grid_map::GridMapRosConverter::toMessage(*this->map));
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
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr brick_detector_array_sub;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr brick_detector_single_sub;

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_pub;


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
