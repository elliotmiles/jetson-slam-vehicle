#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <chrono>


using namespace std::chrono_literals;

class FakeRPLidar : public rclcpp::Node {
public:
    FakeRPLidar() : Node("fake_rplidar") {

        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&FakeRPLidar::publish_scan, this));

        RCLCPP_INFO(this->get_logger(), "Fake RPLIDAR started");
    }



private:
    void publish_scan() {
        auto scan = sensor_msgs::msg::LaserScan();

        scan.header.stamp = this->now();
        scan.header.frame_id = "laser_frame";

        scan.angle_min = -M_PI;
        scan.angle_max = M_PI;
        scan.angle_increment = M_PI / 180.0;  // 360 points
        scan.scan_time = 0.1;
        scan.time_increment = scan.scan_time / 360.0;
        scan.range_min = 0.15;
        scan.range_max = 12.0;

        int points = 360;
        scan.ranges.resize(points);
        scan.intensities.resize(points);

        for (int i = 0; i < points; i++) {
            float angle = scan.angle_min + i * scan.angle_increment;

            // fake wall at 2m with noise
            float distance = 2.0 + 0.05 * std::sin(4 * angle);

            // random dropouts
            if ((i % 40) == 0) {
                scan.ranges[i] = std::numeric_limits<float>::infinity();
            } else {
                scan.ranges[i] = distance;
            }

            scan.intensities[i] = 100.0;
        }

        publisher_->publish(scan);
    }
    

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeRPLidar>());
    rclcpp::shutdown();
    return 0;
}