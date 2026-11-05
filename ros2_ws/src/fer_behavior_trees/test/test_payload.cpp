#include <gtest/gtest.h>

#include <map>
#include <set>
#include <string>
#include <vector>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include "fer_behavior_trees/payload.hpp"

namespace fer_behavior_trees
{
const std::set<std::string> RESERVED{"home", "view"};
const std::map<std::string, double> DEFAULTS{{"speed_scaling", 0.3}};

TEST(Payload, StringsAndNumbersGoToTheBlackboardAsText)
{
  auto blackboard = BT::Blackboard::create();
  const auto keys = apply_payload(
    *blackboard, R"({"target_class": "box", "pick_attempts": 3, "offset": 1.5})", {}, RESERVED);
  EXPECT_EQ(keys.size(), 3u);
  EXPECT_EQ(blackboard->get<std::string>("target_class"), "box");
  ASSERT_TRUE(blackboard->getEntry("pick_attempts")->value.isString());
  EXPECT_EQ(blackboard->get<std::string>("pick_attempts"), "3");
  EXPECT_EQ(blackboard->get<std::string>("offset"), "1.5");
}

TEST(Payload, AnIntegerFitsIntAndUnsignedPorts)
{
  // Repeat's num_cycles is int, Delay's delay_msec unsigned: a typed int entry fails the
  // second at tree creation.
  auto blackboard = BT::Blackboard::create();
  apply_payload(*blackboard, R"({"cycles": 2, "delay_ms": 1})", {}, RESERVED);
  BT::BehaviorTreeFactory factory;
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="T">
         <Repeat num_cycles="{@cycles}">
           <Delay delay_msec="{@delay_ms}"><AlwaysSuccess/></Delay>
         </Repeat>
       </BehaviorTree></root>)",
    blackboard);
  EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
}

TEST(Payload, DefaultFillsAMissingKey)
{
  auto blackboard = BT::Blackboard::create();
  const auto keys =
    apply_payload(*blackboard, R"({"target_class": "box"})", {}, RESERVED, DEFAULTS);
  EXPECT_EQ(keys, (std::vector<std::string>{"target_class", "speed_scaling"}));
  EXPECT_DOUBLE_EQ(blackboard->get<double>("speed_scaling"), 0.3);
}

TEST(Payload, PayloadValueReplacesTheDefault)
{
  auto blackboard = BT::Blackboard::create();
  const auto keys = apply_payload(*blackboard, R"({"speed_scaling": 0.1})", {}, RESERVED, DEFAULTS);
  EXPECT_EQ(keys, std::vector<std::string>{"speed_scaling"});
  EXPECT_DOUBLE_EQ(blackboard->get<double>("speed_scaling"), 0.1);
}

TEST(Payload, IntegerForADefaultKeyIsSetAsDouble)
{
  auto blackboard = BT::Blackboard::create();
  apply_payload(*blackboard, R"({"speed_scaling": 1})", {}, RESERVED, DEFAULTS);
  ASSERT_TRUE(blackboard->getEntry("speed_scaling")->value.isType<double>());
  EXPECT_DOUBLE_EQ(blackboard->get<double>("speed_scaling"), 1.0);
}

TEST(Payload, DefaultReturnsForTheNextGoal)
{
  auto blackboard = BT::Blackboard::create();
  auto keys = apply_payload(*blackboard, R"({"speed_scaling": 0.1})", {}, RESERVED, DEFAULTS);
  keys = apply_payload(*blackboard, "", keys, RESERVED, DEFAULTS);
  EXPECT_EQ(keys, std::vector<std::string>{"speed_scaling"});
  EXPECT_DOUBLE_EQ(blackboard->get<double>("speed_scaling"), 0.3);
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
  auto keys =
    apply_payload(*blackboard, R"({"target_class": "box", "pick_attempts": 3})", {}, RESERVED);
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
  for (const std::string payload :
       {"not json", "[1, 2]", R"({"a": true})", R"({"a": [1]})", R"({"a": {"b": 1}})",
        R"({"a": null})", R"({"home": "x"})"})
  {
    EXPECT_THROW(apply_payload(*blackboard, payload, keys, RESERVED), PayloadError) << payload;
    EXPECT_EQ(blackboard->get<std::string>("target_class"), "box") << payload;
  }
}
}  // namespace fer_behavior_trees
