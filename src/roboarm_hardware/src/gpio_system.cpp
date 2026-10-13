#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gpiod.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>

namespace roboarm_hardware {
using Clock = std::chrono::steady_clock;
using CallbackReturn = hardware_interface::CallbackReturn;
using Return = hardware_interface::return_type;
using namespace std::chrono_literals;

class GpioSystem : public hardware_interface::SystemInterface {
  struct Axis {
    std::string name;
    double command{}, target{}, state{}, velocity{}, actual{}, scale{}, lower{}, upper{}, rate{}, speed{};
    bool inverted{};
    std::vector<int> pins;
    std::vector<gpiod_line*> lines; // DIR, STEP, MS1, MS2, MS3, /ENABLE, MIN, MAX
    Clock::time_point next = Clock::now();
  };
  std::vector<Axis> axes_;
  gpiod_chip* chip_ = nullptr;
  std::vector<gpiod_line*> claimed_;
  std::mutex mutex_;
  std::thread worker_;
  std::atomic<bool> running_{false}, fault_{false};
  Clock::time_point last_write_;
  double watchdog_{};

  static bool truth(const std::string& s) {return s == "true" || s == "True";}
  static double number(const std::string& s) {
    size_t end = 0;
    const double n = std::stod(s, &end);
    if (end != s.size() || !std::isfinite(n)) throw std::runtime_error("Invalid numeric parameter");
    return n;
  }
  void disable() {
    for (auto& a : axes_) if (a.lines.size() >= 6) {
      gpiod_line_set_value(a.lines[1], 0);
      gpiod_line_set_value(a.lines[5], 1);
    }
  }
  void stop() {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    disable();
  }
  void release() {
    stop();
    for (auto* line : claimed_) gpiod_line_release(line);
    claimed_.clear();
    for (auto& a : axes_) a.lines.clear();
    if (chip_) gpiod_chip_close(chip_);
    chip_ = nullptr;
  }
  void run() {
    auto previous = Clock::now();
    while (running_) {
      const auto now = Clock::now();
      const double dt = std::min(0.01, std::chrono::duration<double>(now - previous).count());
      previous = now;
      {
        std::lock_guard<std::mutex> guard(mutex_);
        if (std::chrono::duration<double>(now - last_write_).count() > watchdog_) fault_ = true;
        // NC switches to GND, external 3.3V pull-ups: a broken wire reads HIGH => fault.
        for (auto& a : axes_)
          for (int i : {6, 7}) if (a.lines[i] && gpiod_line_get_value(a.lines[i]) != 0) fault_ = true;
        if (fault_) { disable(); break; }
        for (auto& a : axes_) {
          const double error = (a.target - a.actual) * a.scale;
          if (std::abs(error) < 0.5) {a.speed = 0; continue;}
          const int sign = error > 0 ? 1 : -1;
          // Conservative ramp for discontinuous gripper commands. No catch-up pulse bursts.
          const double speed = std::min(a.rate, std::sqrt(2.0 * 800.0 * std::abs(error)));
          if (a.speed * sign < 0) { a.speed = 0; }
          a.speed = sign * std::min(speed, std::abs(a.speed) + 800.0 * dt);
          if (now < a.next || std::abs(a.speed) < 1.0) continue;
          const double next_position = a.actual + sign / a.scale;
          if (next_position < a.lower - 1e-10 || next_position > a.upper + 1e-10) {
            fault_ = true; break;
          }
          if (gpiod_line_set_value(a.lines[0], (sign > 0) != a.inverted) < 0) {fault_ = true; break;}
          std::this_thread::sleep_for(20us); // DIR setup before rising STEP
          if (!running_) break;
          if (gpiod_line_set_value(a.lines[1], 1) < 0) {fault_ = true; break;}
          std::this_thread::sleep_for(5us);
          if (gpiod_line_set_value(a.lines[1], 0) < 0) {fault_ = true; break;}
          a.actual = next_position; // emitted pulse estimate, NOT encoder feedback
          a.next = Clock::now() + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(1.0 / std::abs(a.speed)));
        }
        if (fault_) {disable(); break;}
      }
      std::this_thread::sleep_for(200us);
    }
    disable();
  }

public:
  ~GpioSystem() override {release();}
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
    try {
      if (!truth(info_.hardware_parameters.at("commissioned")) ||
          !truth(info_.hardware_parameters.at("reference_confirmed")))
        throw std::runtime_error("Commissioning and manual position reference must be confirmed before GPIO activation");
      watchdog_ = number(info_.hardware_parameters.at("watchdog_seconds"));
      if (watchdog_ < 0.05 || watchdog_ > 1.0) throw std::runtime_error("Watchdog must be 0.05..1 seconds");
      std::set<int> used;
      for (const auto& j : info_.joints) {
        if (j.command_interfaces.size() != 1 || j.command_interfaces[0].name != "position" ||
            j.state_interfaces.size() != 2 || j.state_interfaces[0].name != "position" ||
            j.state_interfaces[1].name != "velocity") throw std::runtime_error("Expected position command and position/velocity states");
        Axis a;
        a.name = j.name;
        a.scale = number(j.parameters.at("steps_per_unit"));
        a.rate = number(j.parameters.at("max_step_rate"));
        a.lower = number(j.command_interfaces[0].min);
        a.upper = number(j.command_interfaces[0].max);
        a.actual = a.state = a.command = a.target = number(j.state_interfaces[0].initial_value);
        a.inverted = truth(j.parameters.at("direction_inverted"));
        if (a.scale <= 0 || a.rate < 1 || a.rate > 500 || a.lower >= a.upper ||
            a.state < a.lower || a.state > a.upper) throw std::runtime_error("Invalid joint scale, rate, limits or reference");
        for (const auto* key : {"direction_pin", "step_pin", "ms1_pin", "ms2_pin", "ms3_pin", "enable_pin", "min_limit_pin", "max_limit_pin"}) {
          const double raw = number(j.parameters.at(key));
          const bool optional = std::string(key) == "min_limit_pin" || std::string(key) == "max_limit_pin";
          if (optional && raw == -1) {a.pins.push_back(-1); continue;}
          if (raw < 0 || raw > 27 || raw != std::floor(raw) || !used.insert(static_cast<int>(raw)).second)
            throw std::runtime_error("All GPIO lines must be assigned, unique BCM offsets 0..27");
          a.pins.push_back(static_cast<int>(raw));
        }
        axes_.push_back(std::move(a));
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("GpioSystem"), "%s", e.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> out;
    for (auto& a : axes_) {
      out.emplace_back(a.name, "position", &a.state);
      out.emplace_back(a.name, "velocity", &a.velocity);
    }
    return out;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> out;
    for (auto& a : axes_) out.emplace_back(a.name, "position", &a.command);
    return out;
  }
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
    try {
      chip_ = gpiod_chip_open(info_.hardware_parameters.at("gpio_chip").c_str());
      if (!chip_) throw std::runtime_error("Cannot open GPIO chip");
      for (auto& a : axes_) {
        // Claim /ENABLE first and keep the driver disabled while configuring other lines.
        a.lines.resize(8, nullptr);
        for (int i : {5, 0, 1, 2, 3, 4, 6, 7}) {
          if (a.pins[i] == -1) continue;
          auto* line = gpiod_chip_get_line(chip_, a.pins[i]);
          if (!line || (i < 6 ? gpiod_line_request_output(line, "roboarm", i == 5 ? 1 : 0)
                             : gpiod_line_request_input(line, "roboarm")) < 0)
            throw std::runtime_error("GPIO claim failed; check permissions, chip and exclusive access");
          claimed_.push_back(line);
          a.lines[i] = line;
        }
      }
      return CallbackReturn::SUCCESS;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("GpioSystem"), "%s", e.what());
      // Partial claims must be released without dereferencing unclaimed lines.
      for (auto& a : axes_) {
        if (a.lines.size() > 5 && a.lines[5]) gpiod_line_set_value(a.lines[5], 1);
        a.lines.clear();
      }
      release();
      return CallbackReturn::ERROR;
    }
  }
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    if (fault_ || !chip_ || worker_.joinable()) return CallbackReturn::ERROR;
    for (auto& a : axes_) {
      for (int i : {6, 7}) if (a.lines[i] && gpiod_line_get_value(a.lines[i]) != 0) {
          fault_ = true; disable(); return CallbackReturn::ERROR;
        }
      a.command = a.target = a.state = a.actual;
      a.speed = 0;
      if (gpiod_line_set_value(a.lines[5], 0) < 0) {fault_ = true; disable(); return CallbackReturn::ERROR;}
    }
    last_write_ = Clock::now();
    running_ = true;
    worker_ = std::thread(&GpioSystem::run, this);
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    stop(); fault_ = true; // Loss of holding torque invalidates the reference. Restart/re-reference.
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {release(); return CallbackReturn::SUCCESS;}
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {release(); return CallbackReturn::SUCCESS;}
  CallbackReturn on_error(const rclcpp_lifecycle::State&) override {stop(); fault_ = true; return CallbackReturn::SUCCESS;}
  Return read(const rclcpp::Time&, const rclcpp::Duration& period) override {
    std::lock_guard<std::mutex> guard(mutex_);
    for (auto& a : axes_) {
      a.velocity = period.seconds() > 0 ? (a.actual - a.state) / period.seconds() : 0;
      a.state = a.actual;
    }
    return fault_ ? Return::ERROR : Return::OK;
  }
  Return write(const rclcpp::Time&, const rclcpp::Duration&) override {
    std::lock_guard<std::mutex> guard(mutex_);
    for (auto& a : axes_) {
      if (!std::isfinite(a.command) || a.command < a.lower || a.command > a.upper) fault_ = true;
      else a.target = a.command;
    }
    last_write_ = Clock::now();
    return fault_ ? Return::ERROR : Return::OK;
  }
};
} // namespace roboarm_hardware
PLUGINLIB_EXPORT_CLASS(roboarm_hardware::GpioSystem, hardware_interface::SystemInterface)
