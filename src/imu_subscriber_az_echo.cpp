#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <atomic>
#include <cstdio>

class RateSubscriber : public rclcpp::Node
{
public:
    RateSubscriber()
        : Node("rate_subscriber"), print_flag_(true)
    {
        // Use QoS profile with Best Effort reliability and minimal history depth
        auto qos = rclcpp::QoS(1).best_effort();

        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ap/imu/experimental/data", qos,
            std::bind(&RateSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Only print if the atomic flag is true, then set it to false
        if (print_flag_.exchange(false))
        {
            // Print using printf for minimal overhead
            printf("%f\n", msg->linear_acceleration.z);
            fflush(stdout);  // Ensure immediate flush to terminal

            // Reset the flag to true for the next callback
            print_flag_ = true;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    std::atomic<bool> print_flag_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RateSubscriber>());
    rclcpp::shutdown();
    return 0;
}
