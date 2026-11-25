#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "servo_bridge/msg/setpoint_reached.hpp"

namespace servo_bridge {
namespace {

using SteadyClock = std::chrono::steady_clock;
using namespace std::chrono_literals;

constexpr size_t kMaxServos = 3;  // configured slots: joint1, joint2, gripper - matches SetpointReached.msg
constexpr double kTickToRad = 2.0 * M_PI / 4096.0;              // 12-bit encoder, one turn
constexpr double kRawVelToRadS = 0.229 * 2.0 * M_PI / 60.0;    // 1 raw unit = 0.229 rpm

// Opens the port raw at 115200 baud. read() returns after 0.1 s without data.
int openSerial(const std::string& port, std::string& err) {
  int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) {
    err = std::strerror(errno);
    return -1;
  }
  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    err = std::string("tcgetattr: ") + std::strerror(errno);
    ::close(fd);
    return -1;
  }
  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    err = std::string("tcsetattr: ") + std::strerror(errno);
    ::close(fd);
    return -1;
  }
  tcflush(fd, TCIOFLUSH);
  return fd;
}

}  // namespace

class ServoBridge : public rclcpp::Node {
public:
  ServoBridge() : Node("servo_bridge") {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    const auto names = declare_parameter<std::vector<std::string>>(
        "joint_names", std::vector<std::string>{"arm_joint1", "arm_joint2", "gripper"});
    const auto ids = declare_parameter<std::vector<int64_t>>(
        "servo_ids", std::vector<int64_t>{1, 2, 3});
    center_tick_ = declare_parameter<int64_t>("center_tick", 2048);
    settle_timeout_ = std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(declare_parameter<double>("settle_timeout", 0.3)));
    rx_timeout_ = std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(declare_parameter<double>("rx_timeout", 2.0)));

    if (names.size() != kMaxServos || ids.size() != kMaxServos) {
      throw std::invalid_argument("joint_names and servo_ids must each have exactly 3 entries "
                                  "(joint1, joint2, gripper)");
    }
    for (size_t i = 0; i < kMaxServos; ++i) {
      servos_[i].id = static_cast<int>(ids[i]);
      servos_[i].name = names[i];
    }

    state_pub_ = create_publisher<sensor_msgs::msg::JointState>("arm/joint_states", 10);
    reached_pub_ = create_publisher<msg::SetpointReached>("arm/setpoint_reached", 10);
    goal_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "arm/goal", 10,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr m) { onGoal(*m); });

    reader_ = std::thread([this] { readLoop(); });
  }

  ~ServoBridge() override {
    running_ = false;
    if (reader_.joinable()) reader_.join();
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ >= 0) ::close(fd_);
  }

private:
  struct Servo {
    int id = 0;
    std::string name;
    bool connected = false;              // found by the board's startup scan
    bool goal_pending = false;           // goal sent, motion not yet seen
    SteadyClock::time_point goal_time;
  };

  // ---------------------------------------------------------------- serial port
  bool reopenPort() {
    std::string err;
    int fd = openSerial(port_, err);
    if (fd < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot open %s: %s (retrying)",
                           port_.c_str(), err.c_str());
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(fd_mutex_);
      fd_ = fd;
    }
    RCLCPP_INFO(get_logger(), "Connected to %s", port_.c_str());

    // The board may be a different one (or have been reset with other servos plugged in):
    // forget the old scan result and ask for it.
    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      scan_known_ = false;
    }
    requestScanResult();
    return true;
  }

  void requestScanResult() {
    last_scan_request_ = SteadyClock::now();
    writeSerial("C\n");
  }

  void closePort() {
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }

  bool writeSerial(const std::string& data) {
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Serial port not open, command dropped");
      return false;
    }
    size_t done = 0;
    while (done < data.size()) {
      ssize_t n = ::write(fd_, data.data() + done, data.size() - done);
      if (n < 0) {
        if (errno == EINTR) continue;
        RCLCPP_ERROR(get_logger(), "Serial write failed: %s", std::strerror(errno));
        return false;
      }
      done += static_cast<size_t>(n);
    }
    return true;
  }

  // ------------------------------------------------------------ serial -> ROS
  // Runs in its own thread so blocking reads never hold up the executor.
  void readLoop() {
    std::string buf;
    char chunk[256];
    auto last_rx = SteadyClock::now();

    while (running_ && rclcpp::ok()) {
      int fd;
      {
        std::lock_guard<std::mutex> lock(fd_mutex_);
        fd = fd_;
      }
      if (fd < 0) {
        if (!reopenPort()) {
          std::this_thread::sleep_for(1s);
          continue;
        }
        buf.clear();
        last_rx = SteadyClock::now();
        continue;
      }

      ssize_t n = ::read(fd, chunk, sizeof(chunk));
      if (n < 0) {
        if (errno == EINTR || errno == EAGAIN) continue;
        RCLCPP_ERROR(get_logger(), "Serial read failed: %s - reconnecting", std::strerror(errno));
        closePort();
        continue;
      }
      if (n == 0) {  // 0.1 s without data
        if (SteadyClock::now() - last_rx > rx_timeout_) {
          RCLCPP_WARN(get_logger(), "No data from %s - reconnecting", port_.c_str());
          closePort();
        }
        continue;
      }

      last_rx = SteadyClock::now();
      buf.append(chunk, static_cast<size_t>(n));
      for (size_t nl; (nl = buf.find('\n')) != std::string::npos; buf.erase(0, nl + 1)) {
        std::string line = buf.substr(0, nl);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        handleLine(line);
      }
      if (buf.size() > 1024) buf.clear();  // garbage without newlines
    }
  }

  void handleLine(const std::string& line) {
    if (line.empty()) return;
    const char* text = line.size() > 2 ? line.c_str() + 2 : "";
    switch (line[0]) {
      case 'C': handleScanResult(line.substr(1)); break;
      case 'S': handleState(line.substr(1)); break;
      case '#': RCLCPP_INFO(get_logger(), "board: %s", text); break;
      case 'E': RCLCPP_WARN(get_logger(), "board error: %s", text); break;
      default:  RCLCPP_DEBUG(get_logger(), "ignored line: %s", line.c_str());
    }
  }

  // Body of "C <n> <id> ..." - stores which configured servos the board found.
  void handleScanResult(const std::string& body) {
    std::istringstream in(body);
    size_t n = 0;
    if (!(in >> n)) {
      RCLCPP_WARN(get_logger(), "malformed scan line: C%s", body.c_str());
      return;
    }
    std::vector<int> ids;
    for (int id; in >> id;) ids.push_back(id);
    if (ids.size() != n) {
      RCLCPP_WARN(get_logger(), "malformed scan line (count %zu, %zu ids): C%s", n, ids.size(),
                  body.c_str());
      return;
    }

    std::string found;
    std::lock_guard<std::mutex> lock(servo_mutex_);
    for (Servo& s : servos_) {
      s.connected = false;
      s.goal_pending = false;
    }
    num_connected_ = 0;
    for (int id : ids) {
      const int idx = indexOfId(id);
      if (idx < 0) {
        RCLCPP_WARN(get_logger(), "board reports ID %d, which is not in servo_ids - ignored", id);
        continue;
      }
      if (servos_[idx].connected) continue;  // listed twice
      servos_[idx].connected = true;
      ++num_connected_;
      found += (found.empty() ? "" : ", ") + servos_[idx].name + " (ID " + std::to_string(id) + ")";
    }
    scan_known_ = true;
    RCLCPP_INFO(get_logger(), "%zu of %zu servos connected: %s", num_connected_, kMaxServos,
                found.empty() ? "none" : found.c_str());
  }

  // Body of "S <ms> <id> <pos> <vel> <moving> <hwerr> ..."
  void handleState(const std::string& body) {
    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      if (!scan_known_) {
        // Scan result not received yet (e.g. the C line was lost): ask again, at most once a second.
        if (SteadyClock::now() - last_scan_request_ > 1s) requestScanResult();
        return;
      }
    }

    std::istringstream in(body);
    unsigned long board_ms = 0;
    if (!(in >> board_ms)) {
      RCLCPP_DEBUG(get_logger(), "malformed state line: S%s", body.c_str());
      return;
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    std::array<bool, kMaxServos> reached{};  // false unless the servo reports in fine
    const auto t = SteadyClock::now();

    int id = 0, moving = 0, hwerr = 0;
    long pos = 0, vel = 0;
    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      while (in >> id >> pos >> vel >> moving >> hwerr) {
        const int idx = indexOfId(id);
        if (idx < 0 || !servos_[idx].connected) continue;
        Servo& s = servos_[idx];

        if (hwerr < 0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s (ID %d) not responding",
                               s.name.c_str(), id);
          continue;
        }
        if (hwerr > 0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "%s (ID %d) hardware error 0x%02X", s.name.c_str(), id, hwerr);
        }

        js.name.push_back(s.name);
        js.position.push_back(static_cast<double>(pos - center_tick_) * kTickToRad);
        js.velocity.push_back(static_cast<double>(vel) * kRawVelToRadS);

        // The MOVING flag can lag a fresh goal by a cycle, so after a goal wait until motion is
        // seen (or settle_timeout passes without any: already there, or blocked) before
        // reporting the setpoint as reached.
        if (moving) {
          s.goal_pending = false;
        } else if (s.goal_pending && t - s.goal_time >= settle_timeout_) {
          s.goal_pending = false;
        }
        reached[idx] = !moving && !s.goal_pending && hwerr == 0;
      }
    }

    msg::SetpointReached r;
    r.header.stamp = js.header.stamp;
    r.joint1 = reached[0];
    r.joint2 = reached[1];
    r.gripper = reached[2];

    state_pub_->publish(js);
    reached_pub_->publish(r);
  }

  // ------------------------------------------------------------ ROS -> serial
  // A goal must give exactly one position for every CONNECTED servo (as found by the board's
  // startup scan): by name, or - if `name` is empty - in servo_ids order, skipping servos that
  // are not connected. Anything else is rejected as a whole - no servo moves - so a partial or
  // malformed command can never leave the arm half-commanded.
  void onGoal(const sensor_msgs::msg::JointState& m) {
    const auto reject = [this](const std::string& why) {
      RCLCPP_WARN(get_logger(), "arm/goal ignored: %s", why.c_str());
    };

    std::string cmds;
    {
      std::lock_guard<std::mutex> lock(servo_mutex_);
      if (!scan_known_) {
        reject("connected servos not known yet (waiting for the board)");
        return;
      }
      if (num_connected_ == 0) {
        reject("no servos connected");
        return;
      }
      if (m.position.size() != num_connected_) {
        reject("expected " + std::to_string(num_connected_) + " positions (connected servos), got " +
               std::to_string(m.position.size()));
        return;
      }

      // goal[k] = target for servos_[k]; has_goal[k] marks the servos this message sets.
      std::array<double, kMaxServos> goal{};
      std::array<bool, kMaxServos> has_goal{};
      if (m.name.empty()) {
        size_t next = 0;
        for (size_t k = 0; k < kMaxServos; ++k) {
          if (!servos_[k].connected) continue;
          goal[k] = m.position[next++];
          has_goal[k] = true;
        }
      } else {
        if (m.name.size() != m.position.size()) {
          reject(std::to_string(m.name.size()) + " names for " +
                 std::to_string(m.position.size()) + " positions");
          return;
        }
        for (size_t i = 0; i < m.name.size(); ++i) {
          const int idx = indexOfName(m.name[i]);
          if (idx < 0) {
            reject("unknown joint '" + m.name[i] + "'");
            return;
          }
          if (!servos_[idx].connected) {
            reject("joint '" + m.name[i] + "' is not connected");
            return;
          }
          if (has_goal[idx]) {
            reject("joint '" + m.name[i] + "' given twice");
            return;
          }
          goal[idx] = m.position[i];
          has_goal[idx] = true;
        }
        // Same count + no duplicates + only connected joints => every connected joint is covered.
      }

      for (size_t k = 0; k < kMaxServos; ++k) {
        if (has_goal[k] && !std::isfinite(goal[k])) {
          reject("non-finite position for " + servos_[k].name);
          return;
        }
      }

      // Valid: send all goals in one write so the servos start together.
      const auto t = SteadyClock::now();
      for (size_t k = 0; k < kMaxServos; ++k) {
        if (!has_goal[k]) continue;
        Servo& s = servos_[k];
        const long ticks = std::lround(goal[k] / kTickToRad) + center_tick_;
        cmds += "G " + std::to_string(s.id) + " " + std::to_string(ticks) + "\n";
        s.goal_pending = true;
        s.goal_time = t;
      }
    }
    writeSerial(cmds);
  }

  // Callers hold servo_mutex_ (servos_ itself is only written in the constructor).
  int indexOfId(int id) const {
    for (size_t i = 0; i < kMaxServos; ++i) {
      if (servos_[i].id == id) return static_cast<int>(i);
    }
    return -1;
  }
  int indexOfName(const std::string& name) const {
    for (size_t i = 0; i < kMaxServos; ++i) {
      if (servos_[i].name == name) return static_cast<int>(i);
    }
    return -1;
  }

  std::string port_;
  int64_t center_tick_ = 2048;
  SteadyClock::duration settle_timeout_{};
  SteadyClock::duration rx_timeout_{};

  // Guarded by servo_mutex_ (shared by the reader thread and the goal callback):
  // the servos' connected / goal_pending / goal_time and the scan result below.
  std::array<Servo, kMaxServos> servos_;
  bool scan_known_ = false;      // C line received since the port was opened
  size_t num_connected_ = 0;     // number of servos found by the board's startup scan
  std::mutex servo_mutex_;

  SteadyClock::time_point last_scan_request_{};  // reader thread only

  int fd_ = -1;
  std::mutex fd_mutex_;
  std::atomic<bool> running_{true};
  std::thread reader_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Publisher<msg::SetpointReached>::SharedPtr reached_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr goal_sub_;
};

}  // namespace servo_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<servo_bridge::ServoBridge>());
  rclcpp::shutdown();
  return 0;
}
