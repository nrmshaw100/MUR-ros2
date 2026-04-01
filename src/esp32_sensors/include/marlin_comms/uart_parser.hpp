#pragma once
#include "marlin_comms/data_struct.hpp"
#include <iostream>
#include <vector>

class UartParser {
public:
    enum class State {
        WAIT_SYNC,
        READ_SIZE,
        READ_ADDR,
        READ_DATA,
    };

    void consume(const uint8_t *data, std::size_t n) {
        for (std::size_t i = 0; i < n; i++) {
        step_(data[i]);
        }
    }

private:
    void step_(std::uint8_t b) {
        std::cout << std::hex << static_cast<unsigned int>(b) << " ";
        switch (state_) {
            case State::WAIT_SYNC:
                if (first_sync_ && b == 0x55) {

                    // both conditions met, move to next state
                    state_ = State::READ_SIZE;
                    first_sync_ = false;
                    idx_ = 0;

                } else if (b == 0x55) {
                    first_sync_ = true;
                } else {
                    first_sync_ = false;
                }
                break;

            case State::READ_SIZE:
                data_size_ = static_cast<std::size_t>(b);
                uart_packet_.data_size = b;
                state_ = State::READ_ADDR;
                break;

            case State::READ_ADDR:
                uart_packet_.device_address = b;
                state_ = State::READ_DATA;
                break;

            case State::READ_DATA:
                uart_packet_.data[idx_++] = b;
                if (idx_ == data_size_) {
                    state_ = State::WAIT_SYNC;
                    std::cout << std::endl;
                    }
                // add some sort of callback to handle the data payload
                break;
        }
    }

    // member variables and stuff
    State state_{State::WAIT_SYNC};
    uartPacket_t uart_packet_;
    std::vector<std::uint8_t> payload_;
    bool first_sync_{false};
    std::size_t data_size_{0};
    std::size_t idx_{0};
    std::uint8_t sens_addr_{0};
};
