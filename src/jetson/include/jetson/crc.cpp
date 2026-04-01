#include "crc.hpp"
#include "jetson/crc_lookup.hpp"

#include <algorithm>
#include <vector>

#define DATA_SIZE 58

namespace {

std::uint16_t compute_crc16_impl(const custom_interfaces::msg::SPI &msg) {
    std::uint8_t div = 0;
    std::uint16_t crc = 0;
    const auto payload_size = std::min<std::size_t>(msg.size, msg.data.size());

    std::vector<std::uint8_t> packet_bytes;
    packet_bytes.reserve(4 + payload_size);
    packet_bytes.push_back(msg.synch);
    packet_bytes.push_back(msg.syncl);
    packet_bytes.push_back(static_cast<std::uint8_t>(payload_size));
    packet_bytes.push_back(msg.address);
    packet_bytes.insert(packet_bytes.end(), msg.data.begin(), msg.data.begin() + payload_size);

    for (const std::uint8_t value : packet_bytes) {
        div = static_cast<std::uint8_t>((crc >> 8) ^ value);
        crc = static_cast<std::uint16_t>((crc << 8) ^ lookup_bytes[div]);
    }

    return crc;
}

}  // namespace

std::uint16_t compute_crc16(const custom_interfaces::msg::SPI &msg) {
    return compute_crc16_impl(msg);
}

bool check_crc16(const custom_interfaces::msg::SPI &msg) {
    return compute_crc16_impl(msg) == msg.crc;
}