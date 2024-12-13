#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mavros_msgs/srv/message_interval.hpp>
#include <pigpiod_if2.h>

class IMUReaderNode : public rclcpp::Node
{
public:
    IMUReaderNode() : Node("imu_reader_node")
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

        // Create service client to set message interval
        message_interval_client_ = this->create_client<mavros_msgs::srv::MessageInterval>("/mavros/set_message_interval");
        RCLCPP_INFO(this->get_logger(), "Waiting for MessageInterval service...");
        message_interval_client_->wait_for_service();
        RCLCPP_INFO(this->get_logger(), "MessageInterval service available.");

        // Set the message interval for IMU (ID 27) to 100 Hz
        set_message_interval(27, 200);

        // Set QoS profile to match MAVROS topic
        auto qos = rclcpp::QoS(1).best_effort();
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data_raw", qos, std::bind(&IMUReaderNode::imu_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /mavros/imu/data_raw.");
    }

    ~IMUReaderNode()
    {
        // Turn off the LED and stop pigpio
        gpio_write(pi_handle_, LED_GPIO_PIN, 0);
        pigpio_stop(pi_handle_);
    }

private:
    void set_message_interval(uint16_t message_id, int rate_hz)
    {
        auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
        request->message_id = message_id;                     // Message ID for IMU data
        request->message_rate = rate_hz;                // Interval in microseconds (100 Hz = 10,000 µs)

        auto future = message_interval_client_->async_send_request(request);

        // Wait for the service call to complete
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Successfully set message interval for ID %d to %d Hz.", message_id, rate_hz);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set message interval for ID %d.", message_id);
        }
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
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
    rclcpp::Client<mavros_msgs::srv::MessageInterval>::SharedPtr message_interval_client_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    std::chrono::steady_clock::time_point led_on_until_;
    int pi_handle_; // Handle for pigpiod_if2 connection
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUReaderNode>());
    rclcpp::shutdown();
    return 0;
}