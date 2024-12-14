#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pigpiod_if2.h>
#include <chrono>

class IMUReaderNode : public rclcpp::Node
{
public:
    IMUReaderNode()
        : Node("rate_subscriber"), led_on_until_(std::chrono::steady_clock::time_point::min())
    {
        // Initialize pigpiod_if2
        pi_handle_ = pigpio_start(nullptr, nullptr); // Connect to local pigpiod daemon
        if (pi_handle_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to pigpiod daemon.");
            throw std::runtime_error("Failed to connect to pigpiod daemon.");
        }

        // Set GPIO pin mode for the LED
        set_mode(pi_handle_, LED_GPIO_PIN, PI_OUTPUT);

        // Set up IMU subscriber
        auto qos = rclcpp::QoS(1).best_effort();
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ap/imu/experimental/data", qos,
            std::bind(&IMUReaderNode::topic_callback, this, std::placeholders::_1));
    }

    ~IMUReaderNode()
    {
        // Turn off the LED and stop pigpio
        gpio_write(pi_handle_, LED_GPIO_PIN, 0);
        pigpio_stop(pi_handle_);
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto now = std::chrono::steady_clock::now();

        // Check if the z-axis acceleration exceeds the threshold
        if (msg->linear_acceleration.z > 10)
        {
            led_on_until_ = now + std::chrono::milliseconds(100); // Keep LED on for 100ms
            gpio_write(pi_handle_, LED_GPIO_PIN, 1);             // Turn LED on
            // RCLCPP_INFO(this->get_logger(), "Acceleration (z): %f", msg->linear_acceleration.z);
            // printf("Acceleration (z): %f\n", msg->linear_acceleration.z);
            // fflush(stdout); // Ensure immediate output to terminal
        }
        else if (now >= led_on_until_)
        {
            gpio_write(pi_handle_, LED_GPIO_PIN, 0); // Turn LED off
        }
    }

    static constexpr int LED_GPIO_PIN = 13; // Adjust this to the GPIO pin number used for the LED
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    std::chrono::steady_clock::time_point led_on_until_;
    int pi_handle_; // Handle for pigpiod_if2 connection
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUReaderNode>());
    rclcpp::shutdown();
    return 0;
}
