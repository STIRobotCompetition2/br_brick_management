#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>




#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>



#include <eigen3/Eigen/Geometry>



#define DECAY 0.1
#define DETECTION_THRESHOLD 0.2

// #include <tf2_ros/transform_listener.h>


// #include <tf2_eigen/tf2_eigen.hpp>

using std::placeholders::_1;

class BrickMapper : public rclcpp::Node {
    public:
    BrickMapper() : Node("brick_mapper") {
        brick_detector_sub = this->create_subscription<visualization_msgs::msg::MarkerArray>("/viz", 5, std::bind(&BrickMapper::updateMap, this, _1));
        process_map_timer = this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&BrickMapper::processMap, this));
        grid_map_pub = this->create_publisher<grid_map_msgs::msg::GridMap>("/brick_grid_map", 1);
        rclcpp::QoS map_qos(10);  // initialize to default
        if (true) {
            map_qos.transient_local();
            map_qos.reliable();
            map_qos.keep_last(1);
        }
        cost_map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/brick_cost_map", map_qos);

        map = std::make_shared<grid_map::GridMap>(grid_map::GridMap());
        map->setFrameId("arena");
        map->setGeometry(grid_map::Length(9.0,9.0), 0.02, Eigen::Vector2d(4.0, 4.0));
        RCLCPP_INFO(this->get_logger(), "Created map with size %f x %f m (%i x %i cells).",
         map->getLength().x(), map->getLength().y(),
         map->getSize()(0), map->getSize()(1));
        map->add("bricks", grid_map::Matrix::Zero(map->getSize()(0), map->getSize()(1)));
        // map->atPosition("bricks", grid_map::Position(1,1)) = 5;
        addKernel(Eigen::Vector2d(3,3), 0.5, 1);
        
        


        
    }





    private:
    void addKernel(const Eigen::Vector2d position, const double vx, const double vy){
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
        
        

        for(double x = position.x() - 2*vx; x < position.x() + 2*vx; x+=map->getResolution() * 0.95)
        for(double y = position.y() - 2*vy; y < position.y() + 2*vy; y+=map->getResolution() * 0.95){
            // double exponent = -1 * (Eigen::Vector2d(x, y) - position).cwiseProduct(Eigen::Vector2d(x, y) - position).dot(Eigen::Vector2d(1/vx, 1/vy));
            // map->atPosition("bricks", grid_map::Position(x,y)) = std::exp(exponent);

            map->atPosition("bricks", grid_map::Position(x,y)) = std::cos(std::fabs(x - position.x()) / (2 * vx) * M_PI_2) * std::cos(std::fabs(y - position.y()) / (2 * vy) * M_PI_2);
        }
         


    }
    void processMap(){
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
            for(grid_map::CircleIterator it(*map, max_position, 0.3); !it.isPastEnd(); ++it){
                map->at("bricks", *it) = 0.0;

            }
        }
        nav_msgs::msg::OccupancyGrid costmap;
        grid_map::GridMapRosConverter::toOccupancyGrid(*map, "bricks", 0, DETECTION_THRESHOLD, costmap);
        costmap.header.frame_id = "arena";
        costmap.header.stamp = this->get_clock()->now();

        cost_map_pub->publish(costmap);

        


        this->grid_map_pub->publish(grid_map::GridMapRosConverter::toMessage(*this->map));
    }
    void updateMap(std::shared_ptr<visualization_msgs::msg::MarkerArray> brick_detections){
        map->setTimestamp(brick_detections->markers.at(0).header.stamp.nanosec);
    }
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr brick_detector_sub;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr cost_map_pub;


    rclcpp::TimerBase::SharedPtr process_map_timer;
    std::shared_ptr<grid_map::GridMap> map;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    BrickMapper::SharedPtr bm(new BrickMapper());
    rclcpp::spin(bm);
}
