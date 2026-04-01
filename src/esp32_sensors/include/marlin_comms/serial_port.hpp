#pragma once

// c std libs
#include <cstring>
#include <iostream>
#include <cstdint>
#include <cstddef>
#include <chrono>
#include <vector>

// POSIX libs
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>


class SerialPort {
public:
    // class constructor
    explicit SerialPort(const char* portname) {
        try {
            open_port(portname);
        }
        catch (...) {
            std::cerr << "Failed to open " << portname << ": " << strerror(errno) << std::endl;
        }
    }

    // prevent copy constructing and copy assignment operator
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // define move constructor and move assignment
    // YOU SHOULD IMPLEMENT THIS LATER (YOU MAY NEED IT)
    SerialPort(SerialPort&&) noexcept = delete;
    SerialPort& operator=(SerialPort&&) noexcept = delete;

    // destructor
    ~SerialPort() { close_port(); }

    // basic serial port member functions
    void open_port(const char* portname) {
        fd_ = open(portname, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ == -1) throw std::system_error(errno, std::generic_category(), "open");
    }
    void close_port() noexcept {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }
    bool is_open() const noexcept { return (fd_ == -1) ? false : true; }

    
    void config_port(speed_t baud) {
        // baud rate is usually 
    // Start from current settings
        if (tcgetattr(fd_, &tty_) != 0) {
            throw std::system_error(errno, std::generic_category(), "tcgetattr");
        }

        // Put the port in raw mode and explicitly disable flow control.
        cfmakeraw(&tty_);

        // ---- Input flags ----
        tty_.c_iflag &= ~(IXON | IXOFF | IXANY | INPCK);

        // ---- Output flags ----
        tty_.c_oflag &= ~OPOST;   // raw output

        // ---- Local flags ----
        tty_.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

        // ---- Control flags ----
        tty_.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        tty_.c_cflag |= CS8;     // 8-bit chars
        tty_.c_cflag |= CLOCAL;  // ignore modem controls
        tty_.c_cflag |= CREAD;   // enable receiver

        cfsetispeed(&tty_, baud);
        cfsetospeed(&tty_, baud);

        tty_.c_cc[VMIN]  = 0;   // non-blocking read
        tty_.c_cc[VTIME] = 0;   // no timeout (poll/select handles waiting)

        // clear stale input/output data
        tcflush(fd_, TCIOFLUSH);

        // apply changes now
        if (tcsetattr(fd_, TCSANOW, &tty_) != 0) {
            throw std::system_error(errno, std::generic_category(), "tcsetattr");
        }
    }

    std::size_t write(std::uint8_t* data, std::size_t len) {
        return ::write(fd_, data, len);
    }

    std::size_t write(std::vector<uint8_t> in) {
        return write(in.data(), in.size());
    }

    std::size_t read(std::uint8_t* out, std::size_t max_len,
                           const std::chrono::milliseconds timeout) {
        if (fd_ < 0) {
            throw std::runtime_error("SerialPort not open");
        }
        if (max_len == 0) return 0;

        // create pollfd struct
        pollfd pfd;
        pfd.fd = fd_;
        pfd.events = POLLIN;
        pfd.revents = 0;

        // poll until data available
        int rc;
        for (;;) {
            rc = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
            if (rc >= 0) break;
            if (errno == EINTR) continue;
            throw std::system_error(errno, std::generic_category(), "poll");
        }
        
        if (rc == 0) return 0;

            // poll returned >0: inspect revents
        if (pfd.revents & (POLLERR | POLLNVAL)) {
            throw std::runtime_error("SerialPort poll error / invalid fd");
        }
        if (pfd.revents & POLLHUP) {
            throw std::runtime_error("SerialPort hangup");
        }
        if (!(pfd.revents & POLLIN)) {
            // Some other event woke us; treat as "no data"
            return 0;
        }

        for (;;) {
            const ssize_t n = ::read(fd_, out, max_len);
            if (n > 0) return static_cast<std::size_t>(n);
            if (n == 0) {
                // serial ports usually shouldn't EOF; treat as disconnect
                throw std::runtime_error("SerialPort EOF");
            }
            // n < 0
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            throw std::system_error(errno, std::generic_category(), "read");
        }
    }

    std::size_t read(std::vector<std::uint8_t> out,
                     std::chrono::milliseconds timeout) {
        return read(out.data(), out.size(), timeout);
    }
private:
    int fd_{-1};
    struct termios tty_;
};
