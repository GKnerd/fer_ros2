#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "fer_behavior_trees/server.hpp"
#include "fer_interfaces/action/move_to_joints.hpp"
#include "fer_interfaces/action/move_to_pose.hpp"
#include "fer_interfaces/msg/grasp_candidate.hpp"
#include "fer_interfaces/msg/outcome.hpp"
#include "fer_interfaces/msg/pose_target.hpp"
#include "fer_interfaces/msg/world_object.hpp"
#include "fer_interfaces/srv/check_reachable.hpp"
#include "fer_interfaces/srv/get_place_candidates.hpp"
#include "fer_interfaces/srv/query_objects.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace fer_behavior_trees
{
using namespace std::chrono_literals;
using fer_interfaces::msg::GraspCandidate;
using fer_interfaces::msg::Outcome;
using fer_interfaces::msg::PoseTarget;
using geometry_msgs::msg::PoseStamped;

/// Action server that records goals and ends each with `code`, or holds it until cancelled.
template<class ActionT>
class FakeAction
{
public:
  using Handle = rclcpp_action::ServerGoalHandle<ActionT>;

  FakeAction(const rclcpp::Node::SharedPtr & node, const std::string & name)
  {
    server_ = rclcpp_action::create_server<ActionT>(node, 
      name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal> goal)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        goals_.push_back(*goal);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<Handle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<Handle> handle)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        threads_.emplace_back([this, handle] {execute(handle);});
      }
    );
  }

  ~FakeAction()
  {
    for (auto & thread : threads_) 
    {
      thread.join();
    }
  }

  std::vector<typename ActionT::Goal> goals()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return goals_;
  }

  uint8_t code{Outcome::OK};
  bool hold{false};
  std::atomic<bool> cancelled{false};

private:
  void execute(const std::shared_ptr<Handle> handle)
  {
    auto result = std::make_shared<typename ActionT::Result>();
    if (hold)
    {
      while (!handle->is_canceling() && rclcpp::ok())
      {
        std::this_thread::sleep_for(10ms);
      }
      cancelled = true;
      result->outcome.code = Outcome::CANCELLED;
      handle->canceled(result);
      return;
    }
    result->outcome.code = code;
    if (code == Outcome::OK)
    {
      handle->succeed(result);
    } else 
    {
      handle->abort(result);
    }
  }

  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  std::mutex mutex_;
  std::vector<typename ActionT::Goal> goals_;
  std::vector<std::thread> threads_;
};

class Nodes : public ::testing::Test
{
protected:
  using CheckReachable = fer_interfaces::srv::CheckReachable;
  using GetPlaceCandidates = fer_interfaces::srv::GetPlaceCandidates;
  using QueryObjects = fer_interfaces::srv::QueryObjects;

  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("bt_under_test");
    fakes_ = std::make_shared<rclcpp::Node>("fakes");
    move_to_pose_ = std::make_unique<FakeAction<fer_interfaces::action::MoveToPose>>(fakes_,
      "/motion/move_to_pose"
    );
    move_to_joints_ = std::make_unique<FakeAction<fer_interfaces::action::MoveToJoints>>(fakes_, 
      "/motion/move_to_joints"
    );
    check_reachable_ = fakes_->create_service<CheckReachable>("/motion/check_reachable",
      [this](const std::shared_ptr<CheckReachable::Request> request,
        std::shared_ptr<CheckReachable::Response> response) 
        {
          std::lock_guard<std::mutex> lock(mutex_);
          reachable_requests_.push_back(*request);
          const auto call = reachable_requests_.size() - 1;
          response->outcome.code = call < reachable_codes_.size() ? reachable_codes_[call] : Outcome::OK;
          if (response->outcome.code == Outcome::OK)
          {
            for (size_t i = 0; i < request->targets.size(); ++i) 
            {
              fer_interfaces::msg::JointConfiguration configuration;
              configuration.positions.fill(static_cast<double>(call) + 0.1 * i);
              response->configurations.push_back(configuration);
            }
          }
        }
    );

    query_objects_ = fakes_->create_service<QueryObjects>(
      "/world_model/query_objects",
      [this](const std::shared_ptr<QueryObjects::Request> request,
      std::shared_ptr<QueryObjects::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        query_requests_.push_back(*request);
        for (const auto * id : {"box_1", "box_2"}) {
          fer_interfaces::msg::WorldObject object;
          object.id = id;
          response->objects.push_back(object);
        }
      });
    place_candidates_ = fakes_->create_service<GetPlaceCandidates>(
      "/place/candidates",
      [this](const std::shared_ptr<GetPlaceCandidates::Request> request,
      std::shared_ptr<GetPlaceCandidates::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        place_requests_.push_back(*request);
        response->candidates.resize(2);
      });

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 4);
    executor_->add_node(fakes_);
    spinner_ = std::thread([this] {executor_->spin();});
    register_nodes(factory_, node_, 2000ms, 2000ms, 5000ms);
    blackboard_ = BT::Blackboard::create();
  }

  void TearDown() override
  {
    tree_ = BT::Tree();
    executor_->cancel();
    spinner_.join();
    move_to_pose_.reset();
    move_to_joints_.reset();
  }

  void create(const std::string & body)
  {
    tree_ = factory_.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\">" + body + "</BehaviorTree></root>",
      blackboard_);
  }

  BT::NodeStatus run(const std::string & body)
  {
    create(body);
    return tree_.tickWhileRunning();
  }

  int outcome() {return blackboard_->get<int>("outcome");}

  static PoseStamped pose(double x)
  {
    PoseStamped pose;
    pose.header.frame_id = "base";
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  static GraspCandidate candidate(double x)
  {
    GraspCandidate candidate;
    candidate.pregrasp_pose = pose(x);
    candidate.grasp_pose = pose(x + 0.01);
    candidate.lift_pose = pose(x + 0.02);
    candidate.width = x;
    candidate.force = 10.0 * x;
    return candidate;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr fakes_;
  std::unique_ptr<FakeAction<fer_interfaces::action::MoveToPose>> move_to_pose_;
  std::unique_ptr<FakeAction<fer_interfaces::action::MoveToJoints>> move_to_joints_;
  rclcpp::Service<CheckReachable>::SharedPtr check_reachable_;
  rclcpp::Service<QueryObjects>::SharedPtr query_objects_;
  rclcpp::Service<GetPlaceCandidates>::SharedPtr place_candidates_;
  std::mutex mutex_;
  std::vector<uint8_t> reachable_codes_;
  std::vector<CheckReachable::Request> reachable_requests_;
  std::vector<QueryObjects::Request> query_requests_;
  std::vector<GetPlaceCandidates::Request> place_requests_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spinner_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
};

TEST_F(Nodes, MoveToPoseSendsItsPorts)
{
  blackboard_->set("pose", pose(0.4));
  EXPECT_EQ(
    run(
      R"(<MoveToPose pose="{pose}" path="straight" may_touch="box_1" speed_scaling="0.5"
               outcome="{outcome}"/>)"),
    BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outcome(), Outcome::OK);
  const auto goals = move_to_pose_->goals();
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_EQ(goals[0].target.path, PoseTarget::PATH_STRAIGHT);
  EXPECT_EQ(goals[0].target.may_touch, std::vector<std::string>{"box_1"});
  EXPECT_DOUBLE_EQ(goals[0].target.pose.pose.position.x, 0.4);
  EXPECT_DOUBLE_EQ(goals[0].speed_scaling, 0.5);
}

TEST_F(Nodes, MoveToPoseDefaults)
{
  blackboard_->set("pose", pose(0.4));
  EXPECT_EQ(run(R"(<MoveToPose pose="{pose}"/>)"), BT::NodeStatus::SUCCESS);
  const auto goals = move_to_pose_->goals();
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_EQ(goals[0].target.path, PoseTarget::PATH_FREE);
  EXPECT_TRUE(goals[0].target.may_touch.empty());
  EXPECT_DOUBLE_EQ(goals[0].speed_scaling, 0.3);
}

TEST_F(Nodes, ServerOutcomeReachesThePort)
{
  move_to_pose_->code = Outcome::NO_PATH;
  blackboard_->set("pose", pose(0.4));
  EXPECT_EQ(run(R"(<MoveToPose pose="{pose}" outcome="{outcome}"/>)"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome(), Outcome::NO_PATH);
}

TEST_F(Nodes, InvalidPathIsNotSent)
{
  blackboard_->set("pose", pose(0.4));
  EXPECT_EQ(
    run(R"(<MoveToPose pose="{pose}" path="sideways" outcome="{outcome}"/>)"),
    BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome(), Outcome::INVALID_GOAL);
  EXPECT_TRUE(move_to_pose_->goals().empty());
}

TEST_F(Nodes, MoveToJointsNeedsSevenValues)
{
  EXPECT_EQ(
    run(R"(<MoveToJoints joints="1;2;3" outcome="{outcome}"/>)"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome(), Outcome::INVALID_GOAL);
  EXPECT_TRUE(move_to_joints_->goals().empty());

  EXPECT_EQ(run(R"(<MoveToJoints joints="1;2;3;4;5;6;7"/>)"), BT::NodeStatus::SUCCESS);
  const auto goals = move_to_joints_->goals();
  ASSERT_EQ(goals.size(), 1u);
  EXPECT_DOUBLE_EQ(goals[0].joints.positions[6], 7.0);
}

TEST_F(Nodes, HaltCancelsTheGoal)
{
  move_to_pose_->hold = true;
  blackboard_->set("pose", pose(0.4));
  create(R"(<MoveToPose pose="{pose}"/>)");
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (move_to_pose_->goals().empty() && std::chrono::steady_clock::now() < deadline) {
    tree_.tickOnce();
    std::this_thread::sleep_for(10ms);
  }
  for (int i = 0; i < 20; ++i) {
    tree_.tickOnce();
    std::this_thread::sleep_for(10ms);
  }
  tree_.haltTree();
  while (!move_to_pose_->cancelled && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(move_to_pose_->cancelled);
}

TEST_F(Nodes, QueryObjectsOutputsTheIdsOfFreeObjects)
{
  EXPECT_EQ(run(R"(<QueryObjects class_id="box" ids="{ids}"/>)"), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(
    blackboard_->get<std::vector<std::string>>("ids"),
    (std::vector<std::string>{"box_1", "box_2"}));
  ASSERT_EQ(query_requests_.size(), 1u);
  EXPECT_EQ(query_requests_[0].class_ids, std::vector<std::string>{"box"});
  EXPECT_EQ(
    query_requests_[0].statuses,
    std::vector<uint8_t>{fer_interfaces::msg::WorldObject::FREE});
  EXPECT_FALSE(query_requests_[0].include_fixed);
}

TEST_F(Nodes, GetPlaceCandidatesBuildsTheTarget)
{
  EXPECT_EQ(
    run(
      R"(<GetPlaceCandidates object_id="box_1" x="0.5" y="-0.2" yaw="1.5707963"
                              candidates="{candidates}"/>)"),
    BT::NodeStatus::SUCCESS);
  ASSERT_EQ(place_requests_.size(), 1u);
  const auto & target = place_requests_[0].target;
  EXPECT_EQ(place_requests_[0].object_id, "box_1");
  EXPECT_EQ(target.header.frame_id, "base");
  EXPECT_DOUBLE_EQ(target.pose.position.x, 0.5);
  EXPECT_DOUBLE_EQ(target.pose.position.y, -0.2);
  EXPECT_DOUBLE_EQ(target.pose.position.z, 0.0);
  EXPECT_NEAR(target.pose.orientation.z, std::sqrt(0.5), 1e-6);
  EXPECT_NEAR(target.pose.orientation.w, std::sqrt(0.5), 1e-6);
  EXPECT_EQ(
    blackboard_->get<std::vector<fer_interfaces::msg::PlaceCandidate>>("candidates").size(), 2u);
}

TEST_F(Nodes, MissingServiceIsTimeout)
{
  EXPECT_EQ(
    run(R"(<GetGraspCandidates object_id="box_1" candidates="{c}" outcome="{outcome}"/>)"),
    BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome(), Outcome::TIMEOUT);
}

const char * FIND_GRASP =
  R"(
  <FindReachableGrasp object_id="box_1" candidates="{candidates}" outcome="{outcome}"
                      pregrasp_joints="{joints}" pregrasp_pose="{pregrasp}"
                      grasp_pose="{grasp}" lift_pose="{lift}" width="{width}" force="{force}"/>)";

TEST_F(Nodes, FindReachableGraspTakesTheFirstReachableCandidate)
{
  reachable_codes_ = {Outcome::NO_PATH, Outcome::OK};
  blackboard_->set("candidates", std::vector<GraspCandidate>{candidate(0.3), candidate(0.5)});
  EXPECT_EQ(run(FIND_GRASP), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(outcome(), Outcome::OK);

  ASSERT_EQ(reachable_requests_.size(), 2u);
  const auto & targets = reachable_requests_[1].targets;
  ASSERT_EQ(targets.size(), 3u);
  EXPECT_EQ(targets[0].path, PoseTarget::PATH_FREE);
  EXPECT_EQ(targets[1].path, PoseTarget::PATH_STRAIGHT);
  EXPECT_EQ(targets[2].path, PoseTarget::PATH_STRAIGHT);
  EXPECT_TRUE(targets[0].may_touch.empty());
  EXPECT_EQ(targets[1].may_touch, std::vector<std::string>{"box_1"});
  EXPECT_EQ(targets[2].may_touch, std::vector<std::string>{"box_1"});
  EXPECT_DOUBLE_EQ(targets[0].pose.pose.position.x, 0.5);
  EXPECT_DOUBLE_EQ(targets[1].pose.pose.position.x, 0.51);
  EXPECT_DOUBLE_EQ(targets[2].pose.pose.position.x, 0.52);

  EXPECT_EQ(blackboard_->get<std::vector<double>>("joints"), std::vector<double>(7, 1.0));
  EXPECT_DOUBLE_EQ(blackboard_->get<PoseStamped>("grasp").pose.position.x, 0.51);
  EXPECT_DOUBLE_EQ(blackboard_->get<PoseStamped>("lift").pose.position.x, 0.52);
  EXPECT_DOUBLE_EQ(blackboard_->get<double>("width"), 0.5);
  EXPECT_DOUBLE_EQ(blackboard_->get<double>("force"), 5.0);
}

TEST_F(Nodes, FindReachableGraspFailsWhenNoCandidateIsReachable)
{
  reachable_codes_ = {Outcome::NO_PATH, Outcome::UNREACHABLE};
  blackboard_->set("candidates", std::vector<GraspCandidate>{candidate(0.3), candidate(0.5)});
  EXPECT_EQ(run(FIND_GRASP), BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome(), Outcome::UNREACHABLE);
  EXPECT_EQ(reachable_requests_.size(), 2u);
}

TEST_F(Nodes, FindReachableGraspWithoutCandidatesFails)
{
  blackboard_->set("candidates", std::vector<GraspCandidate>{});
  EXPECT_EQ(run(FIND_GRASP), BT::NodeStatus::FAILURE);
  EXPECT_EQ(outcome(), Outcome::NO_PATH);
  EXPECT_TRUE(reachable_requests_.empty());
}

TEST_F(Nodes, FindReachableSendsItsSpeedScaling)
{
  blackboard_->set("candidates", std::vector<GraspCandidate>{candidate(0.3)});
  EXPECT_EQ(
    run(
      R"(
  <FindReachableGrasp object_id="box_1" candidates="{candidates}" outcome="{outcome}"
                      speed_scaling="0.5"
                      pregrasp_joints="{joints}" pregrasp_pose="{pregrasp}"
                      grasp_pose="{grasp}" lift_pose="{lift}" width="{width}" force="{force}"/>)"),
    BT::NodeStatus::SUCCESS);
  ASSERT_EQ(reachable_requests_.size(), 1u);
  EXPECT_DOUBLE_EQ(reachable_requests_[0].speed_scaling, 0.5);
}

TEST_F(Nodes, FindReachableSpeedScalingDefault)
{
  blackboard_->set("candidates", std::vector<GraspCandidate>{candidate(0.3)});
  EXPECT_EQ(run(FIND_GRASP), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(reachable_requests_.size(), 1u);
  EXPECT_DOUBLE_EQ(reachable_requests_[0].speed_scaling, 0.3);
}
}  // namespace fer_behavior_trees
