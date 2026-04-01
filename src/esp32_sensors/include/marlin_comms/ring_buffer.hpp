#pragma once
#include <algorithm> // std::min
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <utility>
#include <vector>

// Thread-safe (internally locked) ring buffer.
// FIFO semantics:
//   - tail_ = index of oldest element (next pop)
//   - head_ = index of next write position
template <typename T> class Ring {
public:
  explicit Ring(std::size_t capacity)
      : buf_(capacity), head_(0), tail_(0), size_(0) {}

  std::size_t size() const noexcept { return size_; }

  std::size_t capacity() const noexcept { return buf_.size(); }

  bool empty() const noexcept { return size_ == 0; }

  bool full() const noexcept { return size_ == capacity(); }

  void clear() noexcept {
    head_ = 0;
    tail_ = 0;
    size_ = 0;
  }

  // Drop-on-full
  bool push_drop(const T &val) {
    if (size_ == capacity())
      return false;
    buf_[head_] = val;
    head_ = inc_(head_);
    ++size_;
    return true;
  }

  bool push_drop(T &&val) {
    if (size_ == capacity())
      return false;
    buf_[head_] = std::move(val);
    head_ = inc_(head_);
    ++size_;
    return true;
  }

  // Overwrite-on-full
  void push_overwrite(const T &val) {
    const bool was_full = (size_ == capacity());
    buf_[head_] = val;
    head_ = inc_(head_);

    if (was_full) {
      tail_ = inc_(tail_);
    } else {
      ++size_;
    }
  }

  void push_overwrite(T &&val) {
    const bool was_full = (size_ == capacity());
    buf_[head_] = std::move(val);
    head_ = inc_(head_);

    if (was_full) {
      tail_ = inc_(tail_);
    } else {
      ++size_;
    }
  }

  bool pop(T &out) {
    if (size_ == 0)
      return false;

    out = std::move(buf_[tail_]);
    tail_ = inc_(tail_);
    --size_;
    return true;
  }

  // Returns pointer to oldest element, or nullptr if empty.
  const T *peek_oldest() const noexcept {
    return (size_ == 0) ? nullptr : &buf_[tail_];
  }

  // Returns pointer to newest element, or nullptr if empty.
  // Newest is the element just before head_ (wrapping).
  const T *peek_newest() const noexcept {
    if (size_ == 0)
      return nullptr;
    const std::size_t newest_idx =
        (head_ == 0) ? (capacity() - 1) : (head_ - 1);
    return &buf_[newest_idx];
  }

  // Offset from OLDEST (tail_). offset=0 => oldest.
  const T *peek_offset(std::size_t offset_from_oldest) const noexcept {
    if (offset_from_oldest >= size_)
      return nullptr;
    const std::size_t idx = phys_index_from_tail_(offset_from_oldest);
    return &buf_[idx];
  }

private:
  std::size_t inc_(std::size_t i) const noexcept {
    ++i;
    if (i == capacity())
      i = 0;
    return i;
  }

  std::size_t phys_index_from_tail_(std::size_t offset) const noexcept {
    // assumes offset < size_
    std::size_t idx = tail_ + offset;
    const std::size_t cap = capacity();
    if (idx >= cap)
      idx -= cap;
    return idx;
  }

  std::vector<T> buf_;
  std::size_t head_;
  std::size_t tail_;
  std::size_t size_;
};

// Byte-oriented wrapper (no std::span; C++17-friendly).
class ByteRing {
public:
  explicit ByteRing(std::size_t capacity) : ring_(capacity) {}

  std::size_t size() const noexcept {
    std::lock_guard<std::mutex> lock(r_lock_);
    return ring_.size();
  }
  std::size_t capacity() const noexcept { return ring_.capacity(); }
  void clear() noexcept {
    std::lock_guard<std::mutex> lock(r_lock_);
    ring_.clear();
  }

  bool full() const noexcept {
    std::lock_guard<std::mutex> lock(r_lock_);
    return ring_.full();
  }
  bool empty() const noexcept {
    std::lock_guard<std::mutex> lock(r_lock_);
    return ring_.empty();
  }

  // Drop-on-full write
  std::size_t write(const std::uint8_t *in, std::size_t len) {
    std::unique_lock<std::mutex> lock(r_lock_);
    if (!in || len == 0)
      return 0;
    std::size_t n_written = 0;
    for (; n_written < len; ++n_written) {
      if (!ring_.push_drop(in[n_written]))
        break;
    }
    cv_.notify_all();
    return n_written;
  }

  // Convenience overload
  std::size_t write(const std::vector<std::uint8_t> &in) {
    return write(in.data(), in.size());
  }

  std::size_t read(std::uint8_t *out, std::size_t len) {
    if (!out || len == 0)
      return 0;

    // lock and wait until notified by write
    std::unique_lock<std::mutex> lock(r_lock_);
    cv_.wait(lock, [this]() { return !ring_.empty(); });

    std::size_t n = 0;
    for (; n < len; ++n) {
      std::uint8_t b{};
      if (!ring_.pop(b))
        break;
      out[n] = b;
    }
    return n;
  }

  // Convenience overload: read up to out.size()
  std::size_t read(std::vector<std::uint8_t> &out) {
    return read(out.data(), out.size());
  }

  std::size_t discard(std::size_t n) {
    std::lock_guard<std::mutex> lock(r_lock_);
    std::size_t discarded = 0;
    std::uint8_t b{};
    for (; discarded < n; ++discarded) {
      if (!ring_.pop(b))
        break;
    }
    return discarded;
  }

  // Copy (without removing) up to len bytes starting at oldest.
  std::size_t peek(std::uint8_t *out, std::size_t len) const {
    std::lock_guard<std::mutex> lock(r_lock_);
    if (!out || len == 0)
      return 0;
    const std::size_t avail = ring_.size();
    const std::size_t n = std::min(avail, len);

    for (std::size_t i = 0; i < n; ++i) {
      const auto *p = ring_.peek_offset(i);
      out[i] =
          p ? *p
            : std::uint8_t{0}; // p should never be null here, but keep it safe
    }
    return n;
  }

  static constexpr std::size_t no_val = static_cast<std::size_t>(-1);

  // Find a byte pattern in the current buffer (offset from oldest).
  std::size_t find_pattern(const std::uint8_t *pattern,
                           std::size_t pattern_len) const {
    // check for empty pattern
    if (!pattern || pattern_len == 0)
      return 0;

    std::unique_lock<std::mutex> lock(r_lock_);

    // check that the ring insn't empyt
    const std::size_t avail = ring_.size();
    if (pattern_len > avail)
      return no_val;

    // check max length that can be checked
    const std::size_t limit = avail - pattern_len;
    for (std::size_t i = 0; i <= limit; ++i) {
      bool match = true;
      for (std::size_t j = 0; j < pattern_len; ++j) {
        const auto *p = ring_.peek_offset(i + j);
        if (!p || *p != pattern[j]) { // p should not be null if i+j < avail
          match = false;
          break;
        }
      }
      if (match)
        return i;
    }
    return no_val;
  }

  // Copy an exact window without removing. offset is from oldest.
  bool copy_window(std::size_t offset, std::uint8_t *out,
                   std::size_t out_len) const {
    std::lock_guard<std::mutex> lock(r_lock_);
    if (!out)
      return false;
    const std::size_t avail = ring_.size();
    if (offset > avail)
      return false;
    if (out_len > (avail - offset))
      return false;

    for (std::size_t i = 0; i < out_len; ++i) {
      const auto *p = ring_.peek_offset(offset + i);
      if (!p)
        return false;
      out[i] = *p;
    }
    return true;
  }

private:
  Ring<std::uint8_t> ring_;
  mutable std::mutex r_lock_;
  std::condition_variable cv_;
};
