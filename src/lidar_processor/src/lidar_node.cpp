#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>   
#include <chrono>
#include <limits>
#include <functional>
#include <cmath>


using namespace std::chrono_literals;

class LidarNode : public rclcpp::Node
{
public:
    LidarNode() : Node("lidar_node")
    {
        // create subscriber with callback function
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&LidarNode::scan_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            500ms, std::bind(&LidarNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Lidar receiver node started.");

    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // store the latest scan data
        last_scan_ = msg;

        // log basic scan information
        RCLCPP_INFO(this->get_logger(),
                    "Received scan with %zu ranges, angle_min: %.2f, angle_max: %.2f",
                    msg->ranges.size(), msg->angle_min, msg->angle_max);

        
        // find min distance
        float min_range = std::numeric_limits<float>::max();
        size_t min_index = 0;

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > msg->range_min) {
                if (msg->ranges[i] < min_range) {
                    min_range = msg->ranges[i];
                    min_index = i;
                }
            }
        }

        float angle = msg->angle_min + min_index * msg->angle_increment;
        RCLCPP_INFO(this->get_logger(),
                    "Closest object at index %zu: distance = %.2f m, angle = %.2f radians",
                    min_index, min_range, angle);
    }

    void timer_callback() {
        if (last_scan_) {
            RCLCPP_INFO(this->get_logger(),
                        "Timer: Last scan had %zu points",
                        last_scan_->ranges.size());
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Timer: No scan data received yet.");
        }
    }
        
    
    

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto lidar_node = std::make_shared<LidarNode>();
    rclcpp::spin(lidar_node);
    rclcpp::shutdown();
    return 0;
}