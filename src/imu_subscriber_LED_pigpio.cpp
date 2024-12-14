#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pigpio.h>
#include <chrono>

// Define the GPIO pin for the LED
#define LED_PIN 13  // Use BCM numbering for pigpio

class RateSubscriber : public rclcpp::Node
{
public:
    RateSubscriber()
        : Node("rate_subscriber"), led_on_until_(std::chrono::steady_clock::time_point::min())
    {
        // Initialize pigpio
        if (gpioInitialise() < 0)
        {
            perror("Failed to initialize pigpio");
            RCLCPP_ERROR(this->get_logger(), "gpioInitialise error: %d", gpioInitialise());
            throw std::runtime_error("pigpio setup failed");
        }
        gpioSetMode(LED_PIN, PI_OUTPUT);  // Set the LED pin as output
        gpioWrite(LED_PIN, PI_LOW);      // Ensure LED is off initially

        // Use QoS profile with Best Effort reliability and minimal history depth
        auto qos = rclcpp::QoS(1).best_effort();

        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ap/imu/experimental/data", qos,
            std::bind(&RateSubscriber::topic_callback, this, std::placeholders::_1));
    }

    ~RateSubscriber()
    {
        // Cleanup GPIO on shutdown
        gpioWrite(LED_PIN, PI_LOW);
        gpioTerminate();
    }

private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double z_acceleration = msg->linear_acceleration.z;
        auto now = std::chrono::steady_clock::now();

        if (z_acceleration > 9.81)
        {
            // Turn on the LED and reset the timer
            gpioWrite(LED_PIN, PI_HIGH);
            led_on_until_ = now + std::chrono::milliseconds(100);
            RCLCPP_INFO(this->get_logger(), "Acceleration (z): %f", msg->linear_acceleration.z);
            // printf("Acceleration (z): %f\n", msg->linear_acceleration.z);
            // fflush(stdout); // Ensure immediate output to terminal
        }
        else if (now >= led_on_until_)
        {
            // Turn off the LED if the timer has expired
            gpioWrite(LED_PIN, PI_LOW);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    std::chrono::steady_clock::time_point led_on_until_;  // Time until which the LED should stay on
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(std::make_shared<RateSubscriber>());
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "Error: %s\n", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
