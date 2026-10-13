#ifndef FER_BEHAVIOR_TREES__PAYLOAD_HPP_
#define FER_BEHAVIOR_TREES__PAYLOAD_HPP_

#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "behaviortree_cpp/blackboard.h"

namespace fer_behavior_trees
{
/// \brief The goal payload is not a flat JSON object of string or number values.
class PayloadError : public std::runtime_error
{
public:
  explicit PayloadError(const std::string & message) : std::runtime_error(message) {}
};

/// \brief Replace the previous goal's payload keys on \p blackboard with those of \p payload.
/// \param payload JSON object with string, integer or floating-point values; empty means no
///   keys. Integers are set as int, floating-point numbers as double.
/// \param previous_keys Keys set by the previous goal; removed first.
/// \param reserved Keys a payload must not set.
/// \param defaults Keys set to these values when the payload lacks them; a payload value for
///   such a key is set as double.
/// \return The keys set now, defaults included.
/// \throws PayloadError, leaving \p blackboard unchanged.
std::vector<std::string> apply_payload(
  BT::Blackboard & blackboard, const std::string & payload,
  const std::vector<std::string> & previous_keys, const std::set<std::string> & reserved,
  const std::map<std::string, double> & defaults = {});
}  // namespace fer_behavior_trees

#endif  // FER_BEHAVIOR_TREES__PAYLOAD_HPP_
