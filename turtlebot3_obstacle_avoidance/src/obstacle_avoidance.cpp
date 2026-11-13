
#include <memory>
#include <cmath>
#include <functional>
#include <chrono>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
    ObstacleAvoidanceNode()
        : Node("obstacle_avoidance_node")
    {
        // Publisher: Send velocity commands to TurtleBot3
        velocity_publisher_ =
            this->create_publisher<
                geometry_msgs::msg::TwistStamped>(
                "/cmd_vel", 10);

        // Subscriber: Receive LiDAR data
        scan_subscriber_ =
            this->create_subscription<
                sensor_msgs::msg::LaserScan>(
                "/scan",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &ObstacleAvoidanceNode::scanCallback,
                    this,
                    std::placeholders::_1));

        // Timer: Check whether LiDAR data is still arriving
        safety_timer_ = this->create_wall_timer(
            100ms,
            std::bind(
                &ObstacleAvoidanceNode::safetyCallback,
                this));

        last_scan_time_ = std::chrono::steady_clock::now();

        // Publish an initial STOP command
        stopRobot();

        RCLCPP_INFO(
            this->get_logger(),
            "Obstacle Avoidance Node Started");

        RCLCPP_INFO(
            this->get_logger(),
            "LiDAR Subscriber: /scan");

        RCLCPP_INFO(
            this->get_logger(),
            "Velocity Publisher: /cmd_vel (TwistStamped)");

        RCLCPP_INFO(
            this->get_logger(),
            "Safety Mode: STOP ONLY");
    }

private:
    // ROS 2 Subscriber
    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan
    >::SharedPtr scan_subscriber_;

    // ROS 2 Publisher
    rclcpp::Publisher<
        geometry_msgs::msg::TwistStamped
    >::SharedPtr velocity_publisher_;

    // Safety Timer
    rclcpp::TimerBase::SharedPtr safety_timer_;

    // Last LiDAR reception time
    std::chrono::steady_clock::time_point last_scan_time_;

    bool scan_received_ = false;

    // Maximum allowed time without a LiDAR message
    const double scan_timeout_ = 0.5;

    // --------------------------------------------------
    // Publish a zero-velocity command
    // --------------------------------------------------
    void stopRobot()
    {
        geometry_msgs::msg::TwistStamped cmd;

        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";

        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.linear.z = 0.0;

        cmd.twist.angular.x = 0.0;
        cmd.twist.angular.y = 0.0;
        cmd.twist.angular.z = 0.0;

        velocity_publisher_->publish(cmd);
    }

    // --------------------------------------------------
    // Get LiDAR distance at a requested angle
    // --------------------------------------------------
    double getDistance(
        const sensor_msgs::msg::LaserScan::SharedPtr scan,
        double angle)
    {
        const double invalid =
            std::numeric_limits<double>::quiet_NaN();

        if (scan->ranges.empty() ||
            !std::isfinite(scan->angle_min) ||
            !std::isfinite(scan->angle_max) ||
            !std::isfinite(scan->angle_increment) ||
            scan->angle_increment <= 0.0)
        {
            return invalid;
        }

        // Normalize requested angle to scan coverage
        constexpr double TWO_PI = 6.283185307179586;

        double target = angle;

        while (target < scan->angle_min)
        {
            target += TWO_PI;
        }

        while (target > scan->angle_max)
        {
            target -= TWO_PI;
        }

        // Requested direction is outside scan coverage
        if (target < scan->angle_min ||
            target > scan->angle_max)
        {
            return invalid;
        }

        long index = std::lround(
            (target - scan->angle_min) /
            scan->angle_increment);

        if (index < 0 ||
            static_cast<size_t>(index) >=
                scan->ranges.size())
        {
            return invalid;
        }

        double distance = scan->ranges[index];

        // Reject invalid or out-of-range measurements
        if (!std::isfinite(distance) ||
            distance < scan->range_min ||
            distance > scan->range_max)
        {
            return invalid;
        }

        return distance;
    }

    // --------------------------------------------------
    // LiDAR Subscriber Callback
    // --------------------------------------------------
    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        last_scan_time_ = std::chrono::steady_clock::now();
        scan_received_ = true;

        constexpr double PI = 3.141592653589793;

        // Read three directions
        double front = getDistance(scan, 0.0);
        double left  = getDistance(scan, PI / 2.0);
        double right = getDistance(scan, -PI / 2.0);

        // Display measured distances
        RCLCPP_INFO(
            this->get_logger(),
            "Front: %.2f m | Left: %.2f m | Right: %.2f m",
            front,
            left,
            right);

        // Phase 3: Always stop
        stopRobot();
    }

    // --------------------------------------------------
    // Safety Timer Callback
    // --------------------------------------------------
    void safetyCallback()
    {
        auto current_time =
            std::chrono::steady_clock::now();

        double elapsed =
            std::chrono::duration<double>(
                current_time - last_scan_time_).count();

        if (!scan_received_ || elapsed > scan_timeout_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "LiDAR unavailable or timed out. STOP.");

            stopRobot();
        }
    }
};

// --------------------------------------------------
// Main Function
// --------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ObstacleAvoidanceNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
