// Persistent local Python inference worker: binary socket protocol, no shell
// execution.
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "armor_detector.hpp"
namespace training {
namespace {
const char* env(const char* name, const char* fallback) {
  auto p = std::getenv(name);
  return p ? p : fallback;
}
class Worker {
  int fd = -1;
  pid_t pid = -1;
  void receive(void* data, size_t size) {
    auto p = static_cast<char*>(data);
    while (size) {
      pollfd poller{fd, POLLIN, 0};
      int ready;
      do {
        ready = poll(&poller, 1, 120000);
      } while (ready < 0 && errno == EINTR);
      if (ready <= 0)
        throw std::runtime_error("Neural worker response timeout");
      auto n = recv(fd, p, size, 0);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0)
        throw std::runtime_error(
            "Neural worker exited: inspect stderr and RM_POSE_WEIGHTS");
      p += n;
      size -= n;
    }
  }
  void send_all(const void* data, size_t size) {
    auto p = static_cast<const char*>(data);
    while (size) {
      auto n = send(fd, p, size, MSG_NOSIGNAL);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0)
        throw std::runtime_error("Cannot send image to neural worker");
      p += n;
      size -= n;
    }
  }

 public:
  Worker() {
    int sockets[2];
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sockets))
      throw std::runtime_error("socketpair failed");
    pid = fork();
    if (pid < 0) {
      close(sockets[0]);
      close(sockets[1]);
      throw std::runtime_error("fork failed");
    }
    if (pid == 0) {
      close(sockets[0]);
      dup2(sockets[1], 0);
      dup2(sockets[1], 1);
      close(sockets[1]);
      unsetenv("PYTHONPATH");
      execlp(env("RM_PYTHON", "python3"), env("RM_PYTHON", "python3"),
             env("RM_POSE_WORKER", "lesson03/scripts/pose_worker.py"),
             env("RM_POSE_WEIGHTS", "lesson03/artifacts/best.pt"),
             static_cast<char*>(nullptr));
      _exit(127);
    }
    close(sockets[1]);
    fd = sockets[0];
    try {
      uint32_t magic;
      receive(&magic, 4);
      if (magic != 0x41524d52)
        throw std::runtime_error("Invalid neural worker handshake");
    } catch (...) {
      stop();
      throw;
    }
  }
  void stop() {
    if (fd >= 0) {
      close(fd);
      fd = -1;
    }
    if (pid > 0) {
      kill(pid, SIGTERM);
      while (waitpid(pid, nullptr, 0) < 0 && errno == EINTR) {
      }
      pid = -1;
    }
  }
  ~Worker() { stop(); }
  std::vector<Armor> detect(const cv::Mat& image) {
    cv::Mat pixels = image.isContinuous() ? image : image.clone();
    uint32_t header[3] = {uint32_t(image.rows), uint32_t(image.cols),
                          uint32_t(image.total() * 3)};
    send_all(header, sizeof(header));
    send_all(pixels.data, header[2]);
    uint32_t count;
    receive(&count, 4);
    if (count > 1024) throw std::runtime_error("Invalid neural response count");
    std::vector<Armor> out;
    for (uint32_t i = 0; i < count; ++i) {
      float values[11];
      receive(values, sizeof(values));
      for (float v : values)
        if (!std::isfinite(v))
          throw std::runtime_error("Nonfinite neural response");
      Armor a;
      for (int j = 0; j < 4; ++j)
        a.corners[j] = {values[j * 2], values[j * 2 + 1]};
      a.confidence = values[8];
      a.class_id = int(values[9]);
      a.large_armor = values[10] > .5;
      out.push_back(a);
    }
    return out;
  }
};
}  // namespace
std::vector<Armor> detect_neural(const cv::Mat& image) {
  if (image.empty()) return {};
  if (image.type() != CV_8UC3)
    throw std::invalid_argument("Expected CV_8UC3 BGR image");
  static Worker worker;
  return worker.detect(image);
}
}  // namespace training
