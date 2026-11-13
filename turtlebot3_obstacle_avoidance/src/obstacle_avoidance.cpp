
#include <memory>
#include <cmath>
#include <functional>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
    ObstacleAvoidanceNode()
        : Node("obstacle_avoidance_node")
    {
        scan_subscriber_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &ObstacleAvoidanceNode::scanCallback,
                    this,
                    std::placeholders::_1
                )
            );

        RCLCPP_INFO(
            this->get_logger(),
            "LiDAR Subscriber Started"
        );
    }

private:
    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan
    >::SharedPtr scan_subscriber_;

    double getDistance(
        const sensor_msgs::msg::LaserScan::SharedPtr scan,
        double angle)
    {
        if (scan->ranges.empty() ||
            scan->angle_increment <= 0.0)
        {
            return NAN;
        }

        // Normalize the requested angle into scan coverage.
        double target = angle;
        constexpr double two_pi = 6.283185307179586;

        while (target < scan->angle_min) {
            target += two_pi;
        }

        while (target > scan->angle_max) {
            target -= two_pi;
        }

        if (target < scan->angle_min ||
            target > scan->angle_max)
        {
            return NAN;
        }

        long index = std::lround(
            (target - scan->angle_min) /
            scan->angle_increment
        );

        if (index < 0 ||
            static_cast<size_t>(index) >= scan->ranges.size())
        {
            return NAN;
        }

        double distance = scan->ranges[index];

        if (!std::isfinite(distance) ||
            distance < scan->range_min ||
            distance > scan->range_max)
        {
            return NAN;
        }

        return distance;
    }

    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        double front = getDistance(scan, 0.0);
        double left  = getDistance(scan, M_PI / 2.0);
        double right = getDistance(scan, -M_PI / 2.0);

        RCLCPP_INFO(
            this->get_logger(),
            "Front: %.2f m | Left: %.2f m | Right: %.2f m",
            front,
            left,
            right
        );
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ObstacleAvoidanceNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}
