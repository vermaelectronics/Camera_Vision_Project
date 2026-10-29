// Minimal stand-in for Vitis HLS hls_stream.h, for g++ C simulation only.
#ifndef HLS_STREAM_SHIM_H
#define HLS_STREAM_SHIM_H
#include <deque>
namespace hls {
template <typename T> class stream {
  std::deque<T> q;
public:
  bool empty() const { return q.empty(); }
  T read() { T v = q.front(); q.pop_front(); return v; }
  void write(const T &v) { q.push_back(v); }
  void read(T &v) { v = read(); }
  size_t size() const { return q.size(); }
};
}
#endif
