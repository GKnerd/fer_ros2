#ifndef FER_MOVEIT_CONFIG__CORE__CHECKS_HPP_
#define FER_MOVEIT_CONFIG__CORE__CHECKS_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace fer_moveit_config
{
/// \brief How often blocking waits look at the interruption flag.
constexpr std::chrono::milliseconds POLL_PERIOD{50};

/// \brief The goal was cancelled by its client or replaced by a newer goal.
class Interrupted : public std::runtime_error
{
public:
  Interrupted()
  : std::runtime_error("interrupted") {}
};

/// \brief A step failed; carries the fer_interfaces Outcome code the goal ends with.
class MotionError : public std::runtime_error
{
public:
  MotionError(uint8_t code, const std::string & message)
  : std::runtime_error(message), code_(code) {}

  uint8_t code() const {return code_;}

private:
  uint8_t code_;
};

/// \brief True if \p speed_scaling lies in (0, 1].
bool speed_valid(double speed_scaling);

/// \brief True if every velocity is below \p threshold in magnitude.
bool at_rest(const std::vector<double> & velocities, double threshold);

/// \brief Fraction of \p duration covered by \p elapsed, clamped to [0, 1].
double progress(double elapsed, double duration);

/// \brief Wait for \p future up to \p timeout; throws Interrupted as soon as \p interrupted.
/// \return false on timeout.
template<typename FutureT>
bool wait_for(
  const FutureT & future, std::chrono::duration<double> timeout,
  const std::function<bool()> & interrupted = [] {return false;})
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (future.wait_for(POLL_PERIOD) != std::future_status::ready) {
    if (interrupted()) {
      throw Interrupted();
    }
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
  }
  return true;
}

}  // namespace fer_moveit_config

#endif  // FER_MOVEIT_CONFIG__CORE__CHECKS_HPP_
