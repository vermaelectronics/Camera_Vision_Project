// Minimal hls::stream model for C simulation with plain g++ (no Vitis install).
// Vitis HLS uses its own hls_stream.h; this file is only on the csim include path.
#ifndef CSIM_HLS_STREAM_H
#define CSIM_HLS_STREAM_H

#include <deque>
#include <string>
#include <iostream>
#include <cstdlib>

namespace hls {

template <typename T>
class stream {
public:
    stream() {}
    explicit stream(const char *name) : name_(name) {}

    bool empty() const { return q_.empty(); }
    bool full() const { return false; }
    std::size_t size() const { return q_.size(); }

    void write(const T &v) { q_.push_back(v); }
    void operator<<(const T &v) { write(v); }

    T read() {
        if (q_.empty()) {
            std::cerr << "ERROR: read from empty hls::stream '" << name_ << "'\n";
            std::abort();
        }
        T v = q_.front();
        q_.pop_front();
        return v;
    }
    void operator>>(T &v) { v = read(); }
    bool read_nb(T &v) {
        if (q_.empty()) return false;
        v = read();
        return true;
    }
    bool write_nb(const T &v) { write(v); return true; }

private:
    std::string name_;
    std::deque<T> q_;
};

} // namespace hls

#endif
