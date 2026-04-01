#pragma once
#include <cstdint>

#include "crc_lookup.hpp"
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/spi.hpp"

std::uint16_t compute_crc16(const custom_interfaces::msg::SPI &msg);
bool check_crc16(const custom_interfaces::msg::SPI &msg);
