// Minimal stand-in for Vitis HLS hls_stream.h, for a plain g++ C simulation only.
#ifndef HLS_STREAM_STUB_H
#define HLS_STREAM_STUB_H
#include <deque>
#include <cassert>
namespace hls {
template <typename T> class stream {
  std::deque<T> q;
public:
  stream() {}
  stream(const char *) {}
  bool empty() const { return q.empty(); }
  bool full() const { return false; }
  T read() { assert(!q.empty()); T v = q.front(); q.pop_front(); return v; }
  void read(T &v) { v = read(); }
  void write(const T &v) { q.push_back(v); }
  T operator>>(T &v) { v = read(); return v; }
  void operator<<(const T &v) { write(v); }
  size_t size() const { return q.size(); }
};
}
#endif
