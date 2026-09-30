#include <gtest/gtest.h>

#include <algorithm>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "fer_behavior_trees/nodes/candidate_nodes.hpp"
#include "fer_behavior_trees/nodes/find_reachable.hpp"
#include "fer_behavior_trees/nodes/gripper_nodes.hpp"
#include "fer_behavior_trees/nodes/motion_nodes.hpp"
#include "fer_behavior_trees/nodes/world_model_nodes.hpp"

namespace fer_behavior_trees
{
using BT::NodeStatus;

/// Fake nodes under the real IDs and ports: scripted results, a log of the calls.
class Trees : public ::testing::Test
{
protected:
  void SetUp() override
  {
    add<MoveToPoseNode>("MoveToPose", [](BT::TreeNode & n) {
        return std::string(n.getRawPortValue("pose")) + " " +
               n.getInput<std::string>("path").value() + " " +
               n.getInput<std::string>("may_touch").value();
      });
    add<MoveToJointsNode>("MoveToJoints", [](BT::TreeNode & n) {
        return std::string(n.getRawPortValue("joints"));
      });
    add<MoveGripperNode>("MoveGripper");
    add<GraspNode>("Grasp", object_id);
    add<ReleaseNode>("Release", object_id);
    add<DetectObjectsNode>("DetectObjects");
    add<RefineObjectNode>("RefineObject", object_id);
    add<GetGraspCandidatesNode>("GetGraspCandidates", object_id);
    add<FindReachableGrasp>("FindReachableGrasp", object_id);
    add<GetPlaceCandidatesNode>("GetPlaceCandidates", [](BT::TreeNode & n) {
        return n.getInput<std::string>("object_id").value() + " " +
               std::to_string(n.getInput<double>("x").value()).substr(0, 4) + " " +
               std::to_string(n.getInput<double>("y").value()).substr(0, 4);
      });
    add<FindReachablePlace>("FindReachablePlace", object_id);
    factory_.registerSimpleAction(
      "QueryObjects", [this](BT::TreeNode & n) {
        log_.push_back("QueryObjects");
        n.setOutput("ids", ids_);
        return NodeStatus::SUCCESS;
      }, QueryObjectsNode::providedPorts());

    // Registered in reverse, as the server's directory scan has no order.
    for (const auto * file : {"pick_place.xml", "place.xml", "pick.xml"}) {
      factory_.registerBehaviorTreeFromFile(std::string(TREES_DIR) + "/" + file);
    }

    global_ = BT::Blackboard::create();
    global_->set("home", std::vector<double>(7, 0.0));
    global_->set("view", std::vector<double>(7, 0.1));
    global_->set("target_class", std::string("box"));
    global_->set("pick_attempts", 1);
  }

  static std::string object_id(BT::TreeNode & n)
  {
    return n.getInput<std::string>("object_id").value();
  }

  template<class NodeT>
  void add(
    const std::string & id,
    std::function<std::string(BT::TreeNode &)> describe = [] (BT::TreeNode &) {return "";})
  {
    factory_.registerSimpleAction(
      id, [this, id, describe](BT::TreeNode & n) {
        const auto detail = describe(n);
        log_.push_back(detail.empty() ? id : id + " " + detail);
        auto & results = results_[id];
        if (results.empty()) {
          return NodeStatus::SUCCESS;
        }
        const auto status = results.front();
        results.pop_front();
        return status;
      }, NodeT::providedPorts());
  }

  NodeStatus run(const std::string & tree_id, const std::string & object = "")
  {
    auto blackboard = BT::Blackboard::create(global_);
    if (!object.empty()) {
      blackboard->set("object_id", object);
    }
    auto tree = factory_.createTree(tree_id, blackboard);
    return tree.tickWhileRunning();
  }

  size_t count(const std::string & entry) const
  {
    return static_cast<size_t>(std::count(log_.begin(), log_.end(), entry));
  }

  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr global_;
  std::map<std::string, std::deque<NodeStatus>> results_;
  std::vector<std::string> ids_;
  std::vector<std::string> log_;
};

TEST_F(Trees, PickRunsInOrder)
{
  EXPECT_EQ(run("Pick", "box_1"), NodeStatus::SUCCESS);
  const std::vector<std::string> expected{
    "MoveToJoints {@view}",
    "MoveGripper",
    "RefineObject box_1",
    "GetGraspCandidates box_1",
    "FindReachableGrasp box_1",
    "MoveToJoints {pregrasp_joints}",
    "MoveToPose {grasp_pose} straight box_1",
    "Grasp box_1",
    "MoveToPose {lift_pose} straight ",
  };
  EXPECT_EQ(log_, expected);
}

TEST_F(Trees, FailedGraspBacksOutAndFails)
{
  results_["Grasp"] = {NodeStatus::FAILURE};
  EXPECT_EQ(run("Pick", "box_1"), NodeStatus::FAILURE);
  ASSERT_GE(log_.size(), 2u);
  EXPECT_EQ(log_[log_.size() - 2], "Grasp box_1");
  EXPECT_EQ(log_.back(), "MoveToPose {pregrasp_pose} straight box_1");
}

TEST_F(Trees, NoReachableGraspFailsBeforeMoving)
{
  results_["FindReachableGrasp"] = {NodeStatus::FAILURE};
  EXPECT_EQ(run("Pick", "box_1"), NodeStatus::FAILURE);
  EXPECT_EQ(log_.back(), "FindReachableGrasp box_1");
}

TEST_F(Trees, PlaceRunsInOrder)
{
  auto blackboard = BT::Blackboard::create(global_);
  blackboard->set("object_id", std::string("box_1"));
  blackboard->set("x", 0.5);
  blackboard->set("y", -0.2);
  auto tree = factory_.createTree("Place", blackboard);
  EXPECT_EQ(tree.tickWhileRunning(), NodeStatus::SUCCESS);
  const std::vector<std::string> expected{
    "GetPlaceCandidates box_1 0.50 -0.2",
    "FindReachablePlace box_1",
    "MoveToJoints {preplace_joints}",
    "MoveToPose {place_pose} straight ",
    "Release box_1",
    "MoveToPose {retreat_pose} straight box_1",
  };
  EXPECT_EQ(log_, expected);
}

TEST_F(Trees, PickPlacePlacesEveryObjectInARow)
{
  ids_ = {"box_1", "box_2"};
  EXPECT_EQ(run("PickPlace"), NodeStatus::SUCCESS);
  EXPECT_EQ(count("GetPlaceCandidates box_1 0.35 0.30"), 1u);
  EXPECT_EQ(count("GetPlaceCandidates box_2 0.43 0.30"), 1u);
  EXPECT_EQ(log_.back(), "MoveToJoints {@home}");
}

TEST_F(Trees, FailedPickIsRetriedThenTheNextObjectFollows)
{
  global_->set("pick_attempts", 2);
  ids_ = {"box_1", "box_2"};
  results_["Grasp"] = {NodeStatus::FAILURE, NodeStatus::FAILURE};
  EXPECT_EQ(run("PickPlace"), NodeStatus::FAILURE);
  EXPECT_EQ(count("Grasp box_1"), 2u);
  EXPECT_EQ(count("Release box_1"), 0u);
  EXPECT_EQ(count("GetPlaceCandidates box_2 0.35 0.30"), 1u);
  EXPECT_EQ(count("Release box_2"), 1u);
  EXPECT_EQ(count("MoveToJoints {@home}"), 1u);
}

TEST_F(Trees, PickSucceedingOnARetryIsPlaced)
{
  global_->set("pick_attempts", 2);
  ids_ = {"box_1"};
  results_["Grasp"] = {NodeStatus::FAILURE};
  EXPECT_EQ(run("PickPlace"), NodeStatus::SUCCESS);
  EXPECT_EQ(count("Grasp box_1"), 2u);
  EXPECT_EQ(count("Release box_1"), 1u);
}

TEST_F(Trees, FailedPlaceEndsTheTree)
{
  ids_ = {"box_1", "box_2"};
  results_["Release"] = {NodeStatus::FAILURE};
  EXPECT_EQ(run("PickPlace"), NodeStatus::FAILURE);
  EXPECT_EQ(log_.back(), "Release box_1");
  EXPECT_EQ(count("Grasp box_2"), 0u);
}

TEST_F(Trees, NoObjectsIsSuccess)
{
  EXPECT_EQ(run("PickPlace"), NodeStatus::SUCCESS);
  EXPECT_EQ(count("Grasp box_1"), 0u);
  EXPECT_EQ(log_.back(), "MoveToJoints {@home}");
}
}  // namespace fer_behavior_trees
