#include <gtest/gtest.h>

#include <set>
#include <string>
#include <vector>

#include "behaviortree_cpp/blackboard.h"
#include "fer_behavior_trees/payload.hpp"

namespace fer_behavior_trees
{
const std::set<std::string> RESERVED{"home", "view"};

TEST(Payload, StringsAndIntegersGoToTheBlackboard)
{
  auto blackboard = BT::Blackboard::create();
  const auto keys = apply_payload(
    *blackboard, R"({"target_class": "box", "pick_attempts": 3})", {}, RESERVED);
  EXPECT_EQ(keys.size(), 2u);
  EXPECT_EQ(blackboard->get<std::string>("target_class"), "box");
  EXPECT_EQ(blackboard->get<int>("pick_attempts"), 3);
}

TEST(Payload, EmptyPayloadSetsNothing)
{
  auto blackboard = BT::Blackboard::create();
  EXPECT_TRUE(apply_payload(*blackboard, "", {}, RESERVED).empty());
  EXPECT_TRUE(apply_payload(*blackboard, "{}", {}, RESERVED).empty());
}

TEST(Payload, PreviousKeysAreRemoved)
{
  auto blackboard = BT::Blackboard::create();
  auto keys = apply_payload(
    *blackboard, R"({"target_class": "box", "pick_attempts": 3})", {}, RESERVED);
  keys = apply_payload(*blackboard, R"({"target_class": "cylinder"})", keys, RESERVED);
  EXPECT_EQ(keys, std::vector<std::string>{"target_class"});
  EXPECT_EQ(blackboard->get<std::string>("target_class"), "cylinder");
  EXPECT_EQ(blackboard->getEntry("pick_attempts"), nullptr);
}

TEST(Payload, OtherKeysStay)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("home", std::vector<double>(7, 0.0));
  apply_payload(*blackboard, R"({"target_class": "box"})", {}, RESERVED);
  EXPECT_NE(blackboard->getEntry("home"), nullptr);
}

TEST(Payload, RejectedPayloadLeavesTheBlackboardUnchanged)
{
  auto blackboard = BT::Blackboard::create();
  const auto keys = apply_payload(*blackboard, R"({"target_class": "box"})", {}, RESERVED);
  for (const std::string payload : {
      "not json", "[1, 2]", R"({"a": 1.5})", R"({"a": true})", R"({"a": [1]})",
      R"({"a": {"b": 1}})", R"({"a": null})", R"({"a": 99999999999})", R"({"home": "x"})"})
  {
    EXPECT_THROW(apply_payload(*blackboard, payload, keys, RESERVED), PayloadError) << payload;
    EXPECT_EQ(blackboard->get<std::string>("target_class"), "box") << payload;
  }
}
}  // namespace fer_behavior_trees
