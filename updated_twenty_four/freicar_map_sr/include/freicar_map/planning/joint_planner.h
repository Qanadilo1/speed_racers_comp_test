#ifndef  __JOINT_PLANNER_H__
#define  __JOINT_PLANNER_H__

#include "freicar_map/planning/plan.h"
#include "freicar_map/planning/lane_follower.h"
#include "freicar_map/planning/lane_star.h"
namespace freicar
{
namespace planning
{
namespace strategy
{
/* base abstract class for planning strategies */
class PlanningStrategy
{
public:
    PlanningStrategy() {}
    PlanningStrategy(PlanningStrategy&&) = delete;
    virtual ~PlanningStrategy() {}
    virtual planning::Plan GetPlan() = 0;
    virtual planning::Plan Replan(float x, float y, float z, float heading = 0) = 0;
    virtual planning::Plan Extend() = 0;
    virtual enums::PlannerCommand GetCommand() = 0;
    virtual std::string GetStrategyString() = 0;
    virtual bool IsExtendable() = 0;
private:
};
/* lane follower planning strategy, starting at a position
   and following the lanes based on the given command */
class LaneFollowerStrategy : public PlanningStrategy
{
public:
    LaneFollowerStrategy(const LaneFollowerStrategy&) = delete;
    LaneFollowerStrategy(LaneFollowerStrategy&&) = delete;
    LaneFollowerStrategy(const std::string& lane_uuid, float req_loffset,enums::PlannerCommand command,
                         float distance,unsigned int step_count) : lane_uuid_(lane_uuid), req_loffset_(req_loffset),
                         command_(command), distance_(distance), step_count_(step_count) {}
    LaneFollowerStrategy(const mapobjects::Point3D& current_position,enums::PlannerCommand command,
                         float distance,unsigned int step_count) : start_position_(current_position),
                         command_(command), distance_(distance), step_count_(step_count) {}
    planning::Plan GetPlan() override {
        auto new_plan = lane_follower::GetPlan(lane_uuid_, req_loffset_, command_, distance_, step_count_);
        last_plan_step_ = new_plan[-1];
        return new_plan;
    }
    planning::Plan Replan(float x, float y, float z, float heading  = 0) override {
        auto new_plan = lane_follower::GetPlan(mapobjects::Point3D(x, y, z), command_, distance_, step_count_);
        last_plan_step_ = new_plan[-1];
        return new_plan;
        (void)heading;
    }
    planning::Plan Extend() override {
        planning::Plan extension = lane_follower::GetPlan(last_plan_step_.lane_uuid,
                                                          last_plan_step_.lane_offset,
                                                          command_, distance_, step_count_);
        last_plan_step_ = extension[-1];
        return extension;
    }
    enums::PlannerCommand GetCommand() override {
        return command_;
    }
    std::string GetStrategyString() override {
        std::string direction;
        switch (command_) {
        case freicar::enums::PlannerCommand::LEFT:
            direction = "left";
            break;
        case freicar::enums::PlannerCommand::RIGHT:
            direction = "right";
            break;
        case freicar::enums::PlannerCommand::STRAIGHT:
            direction = "straight";
            break;
        case freicar::enums::PlannerCommand::RANDOM:
            direction = "random";
            break;
        default:
            direction = "invalid";
            break;
        }
        return "lane follower - " + direction;
    }
    bool IsExtendable() override {
        return true;
    }

private:
    mapobjects::Point3D start_position_;
    std::string lane_uuid_;
    float req_loffset_;
    enums::PlannerCommand command_;
    float distance_;
    unsigned int step_count_;
    planning::PlanStep last_plan_step_;
};
/* lane-star strategy, performing A* on lane level, and adding equidistant nodes
   between the two given positions */
class LaneStarStrategy : public PlanningStrategy
{
public:
    LaneStarStrategy(const LaneStarStrategy&) = delete;
    LaneStarStrategy(LaneStarStrategy&&) = delete;
    LaneStarStrategy(mapobjects::Point3D& start, float start_heading, mapobjects::Point3D& goal, float goal_heading,
                     float p2p_distance, unsigned int maximum_steps) : start_(start), goal_(goal), start_heading_(start_heading),
                     goal_heading_(goal_heading), p2p_distance_(p2p_distance), planner_(maximum_steps) {}
    planning::Plan GetPlan() override {
        planner_.ResetContainers();
        return planner_.GetPlan(start_, start_heading_, goal_, goal_heading_, p2p_distance_);
    }
    planning::Plan Replan(float x, float y, float z, float heading) override {
        start_.SetCoords(x, y, z);
        start_heading_ = heading;
        planner_.ResetContainers();
        return planner_.GetPlan(start_, start_heading_, goal_, goal_heading_, p2p_distance_);
    }
    planning::Plan Extend() override {
        // returns empty plan with success = false
        return planning::Plan(false);
    }
    enums::PlannerCommand GetCommand() override {
        return enums::PlannerCommand::POINT;
    }
    std::string GetStrategyString() override {
        return "lane star";
    }
    bool IsExtendable() override {
        return false;
    }
private:
    mapobjects::Point3D start_, goal_;
    float start_heading_, goal_heading_;
    float p2p_distance_;
    planning::lane_star::LaneStar planner_;
};
/* simple strategy that just outputs a plan with 3 steps: start, goal and one
   in the middle */
class DirectPathStrategy : public PlanningStrategy
{
public:
    DirectPathStrategy(const DirectPathStrategy&) = delete;
    DirectPathStrategy(DirectPathStrategy&&) = delete;
    DirectPathStrategy(mapobjects::Point3D& start, mapobjects::Point3D& goal) : start_(start), goal_(goal) {}
    planning::Plan GetPlan() override {
        planning::Plan new_plan(true);
        new_plan.steps.emplace_back(start_, mapobjects::Lane::Connection::STRAIGHT,
                                    "?", 0.0f, 0.0f);
        new_plan.steps.emplace_back((start_ + goal_) / 2, mapobjects::Lane::Connection::STRAIGHT,
                                     "?", 0.0f, 0.0f);
        new_plan.steps.emplace_back(goal_, mapobjects::Lane::Connection::STRAIGHT,
                                     "?", 0.0f, 0.0f);
        return new_plan;
    }
    planning::Plan Replan(float x, float y, float z, float heading = 0) override {
        start_.SetCoords(x, y, z);
        planning::Plan new_plan(true);
        new_plan.steps.emplace_back(start_, mapobjects::Lane::Connection::STRAIGHT,
                                    "?", 0.0f, 0.0f);
        new_plan.steps.emplace_back((start_ + goal_) / 2, mapobjects::Lane::Connection::STRAIGHT,
                                     "?", 0.0f, 0.0f);
        new_plan.steps.emplace_back(goal_, mapobjects::Lane::Connection::STRAIGHT,
                                     "?", 0.0f, 0.0f);
        return new_plan;
        (void)heading;
    }
    planning::Plan Extend() override {
        // returns empty plan with success = false
        return planning::Plan(false);
    }
    enums::PlannerCommand GetCommand() override {
        return enums::PlannerCommand::DIRECT;
    }
    std::string GetStrategyString() override {
        return "direct path";
    }
    bool IsExtendable() override {
        return false;
    }
private:
    mapobjects::Point3D start_, goal_;
    
};
/* abstract strategy that does nothing */
class EmptyStrategy : public PlanningStrategy
{
public:
    EmptyStrategy(const EmptyStrategy&) = delete;
    EmptyStrategy(EmptyStrategy&&) = delete;
    EmptyStrategy() {}
    planning::Plan GetPlan() override {
        // returns empty plan with success = true
        return planning::Plan(true);
    }
    planning::Plan Replan(float x, float y, float z, float heading = 0) override {
        // returns empty plan with success = true
        return planning::Plan(true);
        (void)heading;
        (void)x;
        (void)y;
        (void)z;
    }
    planning::Plan Extend() override {
        // returns empty plan with success = true
        return planning::Plan(true);
    }
    enums::PlannerCommand GetCommand() override {
        return enums::PlannerCommand::EMPTY;
    }
    std::string GetStrategyString() override {
        return "empty";
    }
    bool IsExtendable() override {
        return false;
    }
};
} // namespace strategy

/* a class that can combine many planners based on the set strategy. */
class JointPlanner
{
public:
    JointPlanner() {
        strategy_ = nullptr;
    }
    JointPlanner(strategy::PlanningStrategy* strategy) : strategy_(strategy) {}
    void SetStrategy(strategy::PlanningStrategy* strategy) {
        delete strategy_;
        strategy_ = strategy;
    }
    planning::Plan Plan() {
        // return an empty plan if the strategy is not set
        return strategy_ ? strategy_->GetPlan() : planning::Plan(false);
    }
    planning::Plan Plan(strategy::PlanningStrategy* strategy) {
        SetStrategy(strategy);
        return strategy->GetPlan();
    }
    planning::Plan Replan(float x, float y, float z, float heading) {
        return strategy_->Replan(x, y, z, heading);
    }
    planning::Plan Extend() {
        return strategy_->Extend();
    }
    enums::PlannerCommand GetCommand() {
        return strategy_->GetCommand();
    }
    std::string GetStrategyString() {
        return strategy_->GetStrategyString();
    }
    bool IsExtendable() {
        return strategy_->IsExtendable();
    }
private:
    strategy::PlanningStrategy* strategy_;
};

} // namespace planning
} // namespace freicar

#endif