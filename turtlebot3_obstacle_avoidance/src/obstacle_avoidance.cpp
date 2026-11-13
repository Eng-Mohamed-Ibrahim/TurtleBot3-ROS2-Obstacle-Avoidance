
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

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
        // Publisher for robot velocity
        cmd_pub_ =
            create_publisher<geometry_msgs::msg::TwistStamped>(
                "/cmd_vel", 10);

        // Subscriber for LiDAR measurements
        scan_sub_ =
            create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                rclcpp::SensorDataQoS(),
                std::bind(
                    &ObstacleAvoidanceNode::scanCallback,
                    this,
                    std::placeholders::_1));

        // Controller timer: 10 Hz
        control_timer_ = create_wall_timer(
            100ms,
            std::bind(
                &ObstacleAvoidanceNode::controlLoop,
                this));

        // Stop immediately at startup
        publishVelocity(linear, angular);
        RCLCPP_INFO(
            get_logger(),
            "Obstacle Avoidance Controller Started");

        RCLCPP_INFO(
            get_logger(),
            "Mode: DIAGNOSTIC / STOP ONLY");
    }

    ~ObstacleAvoidanceNode() override
    {
        publishVelocity(0.0, 0.0);
    }

private:
    enum class State
    {
        STOP,
        FORWARD,
        TURN_LEFT,
        TURN_RIGHT
    };

    rclcpp::Publisher<
        geometry_msgs::msg::TwistStamped
    >::SharedPtr cmd_pub_;

    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan
    >::SharedPtr scan_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    State state_ = State::STOP;

    double front_ = NAN;
    double left_ = NAN;
    double right_ = NAN;

    bool scan_received_ = false;
    bool scan_valid_ = false;

    std::chrono::steady_clock::time_point last_scan_time_;

    // Controller parameters
    const double forward_speed_ = 0.10;
    const double turn_speed_ = 0.40;

    const double obstacle_threshold_ = 0.60;
    const double clear_threshold_ = 0.80;
    const double scan_timeout_ = 0.50;

    static constexpr double PI = 3.14159265358979323846;

    // -------------------------------------------------
    // Extract minimum valid distance from a sector
    // -------------------------------------------------
    double sectorDistance(
        const sensor_msgs::msg::LaserScan &scan,
        double center,
        double half_width)
    {
        if (scan.ranges.empty() ||
            !std::isfinite(scan.angle_min) ||
            !std::isfinite(scan.angle_increment) ||
            scan.angle_increment <= 0.0 ||
            !std::isfinite(scan.range_min) ||
            !std::isfinite(scan.range_max) ||
            scan.range_min >= scan.range_max)
        {
            return NAN;
        }

        double minimum =
            std::numeric_limits<double>::infinity();

        size_t valid_count = 0;

        for (size_t i = 0; i < scan.ranges.size(); ++i)
        {
            double angle =
                scan.angle_min +
                static_cast<double>(i) *
                scan.angle_increment;

            // Normalize angular difference to [-pi, pi]
            double difference = std::atan2(
                std::sin(angle - center),
                std::cos(angle - center));

            if (std::abs(difference) > half_width)
            {
                continue;
            }

            double distance = scan.ranges[i];

            // Skip invalid readings, never treat them
            // as proof that the path is clear
            if (!std::isfinite(distance) ||
                distance < scan.range_min ||
                distance > scan.range_max)
            {
                continue;
            }

            minimum = std::min(minimum, distance);
            ++valid_count;
        }

        if (valid_count == 0)
        {
            return NAN;
        }

        return minimum;
    }

    // -------------------------------------------------
    // LiDAR Callback
    // -------------------------------------------------
    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        front_ = sectorDistance(
            *msg, 0.0, 15.0 * PI / 180.0);

        left_ = sectorDistance(
            *msg, PI / 2.0, 20.0 * PI / 180.0);

        right_ = sectorDistance(
            *msg, -PI / 2.0, 20.0 * PI / 180.0);

        scan_valid_ =
            std::isfinite(front_) &&
            std::isfinite(left_) &&
            std::isfinite(right_);

        scan_received_ = true;

        last_scan_time_ =
            std::chrono::steady_clock::now();

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "LiDAR: Front=%.2f | Left=%.2f | Right=%.2f",
            front_, left_, right_);

        if (!scan_valid_)
        {
            state_ = State::STOP;
            publishVelocity(0.0, 0.0);
        }
    }

    // -------------------------------------------------
    // Publish TwistStamped velocity command
    // -------------------------------------------------
    void publishVelocity(double linear, double angular)
    {
        geometry_msgs::msg::TwistStamped cmd;

        cmd.header.stamp = now();
        cmd.header.frame_id = "base_link";

        cmd.twist.linear.x = linear;
        cmd.twist.angular.z = angular;

        cmd_pub_->publish(cmd);
    }

    // -------------------------------------------------
    // Controller and Safety Timer
    // -------------------------------------------------
    void controlLoop()
    {
        if (!scan_received_ || !scan_valid_)
        {
            state_ = State::STOP;
            publishVelocity(0.0, 0.0);

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Invalid or missing LiDAR data. STOP");

            return;
        }

        double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() -
            last_scan_time_).count();

        if (age > scan_timeout_)
        {
            state_ = State::STOP;
            publishVelocity(0.0, 0.0);

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "LiDAR timeout. STOP");

            return;
        }

        // Reactive controller with hysteresis
        if (state_ == State::TURN_LEFT ||
            state_ == State::TURN_RIGHT)
        {
            if (front_ > clear_threshold_)
            {
                state_ = State::FORWARD;
            }
        }
        else if (front_ <= obstacle_threshold_)
        {
            state_ = (left_ >= right_)
                ? State::TURN_LEFT
                : State::TURN_RIGHT;
        }
        else
        {
            state_ = State::FORWARD;
        }

        std::string state_name = "STOP";

        double linear = 0.0;
        double angular = 0.0;

        switch (state_)
        {
            case State::FORWARD:
                state_name = "FORWARD";
                linear = forward_speed_;
                break;

            case State::TURN_LEFT:
                state_name = "TURN_LEFT";
                angular = turn_speed_;
                break;

            case State::TURN_RIGHT:
                state_name = "TURN_RIGHT";
                angular = -turn_speed_;
                break;

            case State::STOP:
                break;
        }

        // DIAGNOSTIC MODE:
        // Keep the robot stationary during validation.
        // Do not publish calculated movement commands yet.
        publishVelocity(0.0, 0.0);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "State: %s | Planned v=%.2f | Planned w=%.2f",
            state_name.c_str(),
            linear,
            angular);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ObstacleAvoidanceNode>();

    rclcpp::spin(node);

    node.reset();
    rclcpp::shutdown();

    return 0;
}
