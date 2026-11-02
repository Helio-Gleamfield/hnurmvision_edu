#pragma once
#include "config.hpp"
#include "types.hpp"
#include <optional>
#include <memory>

namespace training::io {
struct IoContext {
  Config config;
  explicit IoContext(Config configuration);
  ~IoContext();
  IoContext(const IoContext&) = delete;
  IoContext& operator=(const IoContext&) = delete;
  void open();
  std::optional<SensorData> read();
  void write(const Command& command);
  void close();
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace training::io
