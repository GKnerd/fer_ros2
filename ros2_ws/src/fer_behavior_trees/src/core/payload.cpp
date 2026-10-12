#include "fer_behavior_trees/payload.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "behaviortree_cpp/contrib/json.hpp"

namespace fer_behavior_trees
{
std::vector<std::string> apply_payload(
  BT::Blackboard & blackboard, const std::string & payload,
  const std::vector<std::string> & previous_keys, const std::set<std::string> & reserved,
  const std::map<std::string, double> & defaults)
{
  std::vector<std::pair<std::string, std::variant<std::string, int, double>>> entries;
  if (!payload.empty())
  {
    nlohmann::json json;
    try
    {
      json = nlohmann::json::parse(payload);
    }
    catch (const nlohmann::json::parse_error & e)
    {
      throw PayloadError(std::string("payload is not JSON: ") + e.what());
    }
    if (!json.is_object())
    {
      throw PayloadError("payload must be a JSON object");
    }
    for (const auto & item : json.items())
    {
      const auto & key = item.key();
      const auto & value = item.value();
      if (reserved.count(key) > 0)
      {
        throw PayloadError("payload key '" + key + "' is reserved");
      }
      if (value.is_string())
      {
        entries.emplace_back(key, value.get<std::string>());
      }
      else if (value.is_number() && defaults.count(key) > 0)
      {
        entries.emplace_back(key, value.get<double>());
      }
      else if (
        value.is_number_integer() && value.get<double>() >= std::numeric_limits<int>::min() &&
        value.get<double>() <= std::numeric_limits<int>::max())
      {
        entries.emplace_back(key, value.get<int>());
      }
      else if (value.is_number_float())
      {
        entries.emplace_back(key, value.get<double>());
      }
      else
      {
        throw PayloadError("payload value of '" + key + "' must be a string or a number");
      }
    }
  }
  for (const auto & item : defaults)
  {
    const auto & key = item.first;
    const bool given = std::any_of(
      entries.begin(), entries.end(), [&key](const auto & entry) { return entry.first == key; });
    if (!given)
    {
      entries.emplace_back(key, item.second);
    }
  }

  for (const auto & key : previous_keys)
  {
    blackboard.unset(key);
  }
  std::vector<std::string> keys;
  for (const auto & entry : entries)
  {
    const auto & key = entry.first;
    std::visit([&blackboard, &key](const auto & v) { blackboard.set(key, v); }, entry.second);
    keys.push_back(key);
  }
  return keys;
}
}  // namespace fer_behavior_trees
