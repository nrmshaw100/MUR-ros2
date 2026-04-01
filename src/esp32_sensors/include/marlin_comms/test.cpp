#include "serial_port.hpp"
#include "ring_buffer.hpp"

#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>
#include <atomic>


#define CHUNK 4096; 

int main() {
    SerialPort port("/dev/ttyUSB0");
    ByteRing ring(8192);
    port.config_port(B115200);

    std::atomic<bool> running{true}; 

    std::array<std::uint8_t, 512> tmp;

    std::vector<std::uint8_t> buf(CHUNK);

    return 0;
}
