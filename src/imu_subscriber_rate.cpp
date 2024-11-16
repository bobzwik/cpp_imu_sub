#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class RateSubscriber : public rclcpp::Node
{
public:
    RateSubscriber()
        : Node("rate_subscriber"), message_count_(0)
    {
        // Use a QoS profile with Best Effort reliability
        auto qos = rclcpp::QoS(10).best_effort();

        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ap/imu/experimental/data", qos,
            std::bind(&RateSubscriber::topic_callback, this, std::placeholders::_1));

        // Timer to print the rate every second
        timer_ = this->create_wall_timer(1s, std::bind(&RateSubscriber::timer_callback, this));
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr /* msg */)
    {
        ++message_count_;  // Increment message count for each received message
    }

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Average rate: %d messages per second", message_count_);
        message_count_ = 0;  // Reset count for the next interval
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    int message_count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RateSubscriber>());
    rclcpp::shutdown();
    return 0;
}
