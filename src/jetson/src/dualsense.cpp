#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "jetson/data_struct.hpp"
#include "custom_interfaces/msg/spi.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <functional>
#include <memory>

#define DOF 6
#define GET_STATUS 0x00
#define GET_DATA 0x02

class DualsenseSub : public rclcpp::Node {
public:
    DualsenseSub(): Node("dualsense_sub"){
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&DualsenseSub::joy_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Dualsense subscription started");

        publisher_ = this->create_publisher<custom_interfaces::msg::SPI>("spi_send", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&DualsenseSub::publish_command, this)
        );
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy &msg) {
        latest_wrench_.fill(0.0f);

        if (msg.axes.size() > 0) {
            latest_wrench_[0] = static_cast<float>(msg.axes[0]);  // sway
        }
        if (msg.axes.size() > 1) {
            latest_wrench_[1] =  - static_cast<float>(msg.axes[1]);  // surge
        }
        if (msg.axes.size() > 5) {
            latest_wrench_[2] =  - static_cast<float>(msg.axes[4] - msg.axes[5]);  // heave
            latest_wrench_[3] = static_cast<float>(msg.axes[3]) * 0.2;  // pitch
            latest_wrench_[4] = static_cast<float>(msg.axes[2]) * 0.2;  // roll
        }

        if (msg.buttons.size() > 10 && (msg.buttons[9] || msg.buttons[10])) {
            latest_wrench_[5] = - static_cast<float>(msg.buttons[9] - msg.buttons[10]) * 0.3f;
        }
    }

    void publish_command() {
        auto msg_out = custom_interfaces::msg::SPI();
        msg_out.synch = 0x55;
        msg_out.syncl = 0x55;
        msg_out.size = DOF * sizeof(float);
        msg_out.address = current_address_;
        std::memcpy(msg_out.data.data(), latest_wrench_.data(), msg_out.size);

        publisher_->publish(msg_out);
        current_address_ = (current_address_ == GET_STATUS) ? GET_DATA : GET_STATUS;
    }

    std::uint8_t current_address_{GET_DATA};

    std::array<float, DOF> latest_wrench_{};
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::SPI>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualsenseSub>());
    rclcpp::shutdown();
    return 0;
}
