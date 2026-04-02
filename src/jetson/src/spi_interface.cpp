#include <custom_interfaces/msg/detail/spi__struct.hpp>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <chrono>
#include <array>
#include <functional>
#include <algorithm>

#include "jetson/crc.hpp"
#include "jetson/data_struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/spi.hpp"

// Fixed byte offsets for the 64-byte SPI packet exchanged with the ESP32.
#define SYNCH_POS 0
#define SYNCL_POS 1
#define SIZE_POS 2
#define ADDR_POS 3
#define DATA_POS 4
#define CRC1_POS 62
#define CRC2_POS 63

#define SPI_PACKET_SIZE 64
#define DATA_SIZE 58

// Small RAII wrapper around a Linux spidev file descriptor.
class SpiDevice {
public:
    SpiDevice() = default;

    ~SpiDevice() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
  }

  SpiDevice(const SpiDevice&) = delete;
  SpiDevice& operator=(const SpiDevice&) = delete;

    // Open the device and configure the SPI bus settings used by this node.
  uint32_t openPort(const std::string& device, uint32_t speed_hz, uint8_t mode = SPI_MODE_0,
                    uint8_t bits_per_word = 8) {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }

        fd_ = ::open(device.c_str(), O_RDWR);
        if (fd_ < 0) {
            throw std::runtime_error("open(" + device + ") failed: " + std::strerror(errno));
        }

        mode_ = mode;
        bits_per_word_ = bits_per_word;
        speed_hz_ = speed_hz;

        if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0 || ::ioctl(fd_, SPI_IOC_RD_MODE, &mode_) < 0) {
            throw std::runtime_error("Failed to set/get SPI mode: " + std::string(std::strerror(errno)));
        }

        if (::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word_) < 0 
            || ::ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word_) < 0) {
            throw std::runtime_error("Failed to set/get bits per word: " + std::string(std::strerror(errno)));
        }

        if (::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0 
            || ::ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz_) < 0) {
            throw std::runtime_error("Failed to set/get max speed: " + std::string(std::strerror(errno)));
        }

        return speed_hz_;
  }

    // Execute one full-duplex SPI transaction.
    void transfer(std::vector<uint8_t>& tx, std::vector<uint8_t>& rx) const {
        if (fd_ < 0) {
            throw std::runtime_error("SPI port is not open. Call openPort() first.");
        }

        // The ESP side expects a fixed 64-byte frame every transfer.
        if (tx.size() < SPI_PACKET_SIZE) {
            tx.resize(SPI_PACKET_SIZE);
        return;
        }

        rx.assign(tx.size(), 0);

        spi_ioc_transfer tr{};
        tr.tx_buf = reinterpret_cast<uint64_t>(tx.data());
        tr.rx_buf = reinterpret_cast<uint64_t>(rx.data());
        tr.len = SPI_PACKET_SIZE;
        tr.speed_hz = speed_hz_;
        tr.bits_per_word = bits_per_word_;

        if (::ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 1) {
            throw std::runtime_error("SPI transfer failed: " + std::string(std::strerror(errno)));
        }
    }

private:
    int fd_{-1};
    uint8_t mode_{SPI_MODE_0};
    uint8_t bits_per_word_{8};
    uint32_t speed_hz_{500000};
};

// ROS node that bridges a ROS topic to one Linux SPI device and republishes the reply.
class SPI_Interface : public rclcpp::Node {
public:
    SPI_Interface(std::string port, std::string spi_topic) : Node("spi_interface") {
        const uint32_t speed_hz = spi1_.openPort(port, 1000000);
        RCLCPP_INFO(this->get_logger(), "Configured SPI max speed: %u Hz", speed_hz);
        
        subscription_ = this->create_subscription<custom_interfaces::msg::SPI>(
            spi_topic, 
            10,
            std::bind(&SPI_Interface::spi_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<custom_interfaces::msg::SPI>("spi_receive", 10);

        RCLCPP_INFO(this->get_logger(), "SPI Node Started");
    }
private:
    void debug_log_packet(const std::vector<uint8_t>& packet, const char* label) {
    #ifdef DEBUG
        if (packet.size() < SPI_PACKET_SIZE) {
            RCLCPP_WARN(
                this->get_logger(),
                "%s packet too short: %zu bytes (expected %d)",
                label,
                packet.size(),
                SPI_PACKET_SIZE
            );
            return;
        }

        RCLCPP_INFO(this->get_logger(), "%s packet dump (64 bytes):", label);
        RCLCPP_INFO(
            this->get_logger(),
            "[%02d..%02d] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
            0, 15,
            packet[0], packet[1], packet[2], packet[3],
            packet[4], packet[5], packet[6], packet[7],
            packet[8], packet[9], packet[10], packet[11],
            packet[12], packet[13], packet[14], packet[15]
        );
        RCLCPP_INFO(
            this->get_logger(),
            "[%02d..%02d] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
            16, 31,
            packet[16], packet[17], packet[18], packet[19],
            packet[20], packet[21], packet[22], packet[23],
            packet[24], packet[25], packet[26], packet[27],
            packet[28], packet[29], packet[30], packet[31]
        );
        RCLCPP_INFO(
            this->get_logger(),
            "[%02d..%02d] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
            32, 47,
            packet[32], packet[33], packet[34], packet[35],
            packet[36], packet[37], packet[38], packet[39],
            packet[40], packet[41], packet[42], packet[43],
            packet[44], packet[45], packet[46], packet[47]
        );
        RCLCPP_INFO(
            this->get_logger(),
            "[%02d..%02d] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
            48, 63,
            packet[48], packet[49], packet[50], packet[51],
            packet[52], packet[53], packet[54], packet[55],
            packet[56], packet[57], packet[58], packet[59],
            packet[60], packet[61], packet[62], packet[63]
        );
    #else
        (void)packet;
        (void)label;
    #endif
    }

    // Build an outbound 64-byte SPI frame from the ROS message fields.
    std::vector<uint8_t> build_tx_packet(const custom_interfaces::msg::SPI &msg) {
        spi_out_.start_frameH = msg.synch;
        spi_out_.start_frameL = msg.syncl;
        spi_out_.data_size = std::min<std::size_t>(msg.size, sizeof(spi_out_.data));
        spi_out_.device_address = msg.address;

        std::fill(std::begin(spi_out_.data), std::end(spi_out_.data), 0);
        std::memcpy(spi_out_.data, msg.data.data(), spi_out_.data_size);

        spi_out_.crc = compute_crc16(msg);
        #ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "CRC Sent: %X", spi_out_.crc);
        #endif

        std::vector<uint8_t> spi_out(
            reinterpret_cast<uint8_t*>(&spi_out_),
            reinterpret_cast<uint8_t*>(&spi_out_) + sizeof(spi_out_)
        );
        return spi_out;
    }

    // Decode the returned SPI frame into a ROS message.
    bool decode_rx_packet(
        const std::vector<uint8_t>& spi_in,
        custom_interfaces::msg::SPI& msg_out
    ) {
        msg_out = custom_interfaces::msg::SPI();
        msg_out.synch = spi_in[SYNCH_POS];
        msg_out.syncl = spi_in[SYNCL_POS];

        msg_out.size = spi_in[SIZE_POS];
        msg_out.address = spi_in[ADDR_POS];

        std::fill(msg_out.data.begin(), msg_out.data.end(), 0);
        std::copy(spi_in.begin() + DATA_POS, spi_in.begin() + DATA_POS + DATA_SIZE, msg_out.data.begin());

        // caluculate the crc in little endian format since that's how esp32 sends it
        const auto crc_le = static_cast<std::uint16_t>(spi_in[CRC1_POS])
                          | (static_cast<std::uint16_t>(spi_in[CRC2_POS]) << 8);
        msg_out.crc = crc_le;


        // check the sync bytes
        if (msg_out.synch != START_FRAMEH || msg_out.syncl != START_FRAMEL) {
            #ifdef DEBUG
            RCLCPP_WARN(this->get_logger(), "Bad sync bytes: %02X %02X", msg_out.synch, msg_out.syncl);
            #endif
            return false;
        }

        // check the crc
        if (msg_out.crc != compute_crc16(msg_out)) {
            #ifdef DEBUG
            RCLCPP_WARN(this->get_logger(), "Bad CRC: expected %X, received %X", compute_crc16(msg_out), msg_out.crc);
            #endif
            return false;
        }

        return true;
    }

    void spi_callback(const custom_interfaces::msg::SPI &msg) {
        std::vector<uint8_t> spi_out = build_tx_packet(msg);

        // RCLCPP_INFO(this->get_logger(), "CRC Sent: %X", spi_out_.crc);
        // RCLCPP_INFO(this->get_logger(), "first sync byte: %X, %X", spi_out[0], spi_out[1]);

        std::vector<uint8_t> prime_rx;
        std::vector<uint8_t> spi_in;
        
        // empty the msg out and in buffers to avoid confusion during debugging
        msg_out = {};
        msg_out_prime = {};

        // The first transfer clocks the request into the slave; the second reads back its prepared response.
        spi1_.transfer(spi_out, prime_rx);
        ::usleep(5000);
        spi1_.transfer(spi_out, spi_in);
        // debug_log_packet(prime_rx, "PRIME RX");
        // debug_log_packet(spi_in, "RX");

        // I'm not sure why, but the but sometimes the first packet is the good one
        // we will accept whichever one decodes successfully for now, but this should be investigated further.
        auto msg_out = custom_interfaces::msg::SPI();
        if (decode_rx_packet(spi_in, msg_out)) {
            publisher_->publish(msg_out);
            return;
        } else if (decode_rx_packet(prime_rx, msg_out_prime)) {
            publisher_->publish(msg_out_prime);
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Failed to decode both SPI responses:\n"
            "spi_in: sync=%X %X, address=%X, crc=%X\n"
            "prime_rx: sync=%X %X, address=%X, crc=%X",
            msg_out.synch,
            msg_out.syncl,
            msg_out.address,
            msg_out.crc,
            msg_out_prime.synch,
            msg_out_prime.syncl,
            msg_out_prime.address,
            msg_out_prime.crc
        );
        return;
    }
    custom_interfaces::msg::SPI msg_out;
    custom_interfaces::msg::SPI msg_out_prime;

    packet_t spi_out_{};
    SpiDevice spi1_;
    rclcpp::Subscription<custom_interfaces::msg::SPI>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::SPI>::SharedPtr publisher_;
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    // Run one node per SPI bus so both devices can be serviced by the same executor.
    std::shared_ptr<rclcpp::Node> node1 = std::make_shared<SPI_Interface>("/dev/spidev0.0", "spi_send");
    std::shared_ptr<rclcpp::Node> node2 = std::make_shared<SPI_Interface>("/dev/spidev1.0", "spi_monitor");
    
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node1);
    executor.add_node(node2);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
