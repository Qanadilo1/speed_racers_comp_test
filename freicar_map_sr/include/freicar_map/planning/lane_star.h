#ifndef __LANE_STAR__
#define __LANE_STAR__

#include <unordered_map>
#include <functional>
#include <vector>
#include <queue>
#include <limits>

#include "map_core/freicar_map_objects.h"
#include "freicar_map/planning/plan.h"
namespace freicar
{
namespace planning
{
namespace lane_star
{


struct LaneCost {
    float g = 1000.0f;
    float h = 1000.0f;
    float f() { return g + h;}
};

class LaneStar
{
public:
    LaneStar(unsigned int max_steps);
    freicar::planning::Plan GetPlan(const mapobjects::Point3D& start, float start_heading,
                                    const mapobjects::Point3D& goal, float goal_heading,
                                    const float p2p_distance);
    void ResetContainers();
private:
    void ExpandLane(const mapobjects::Lane* lane);
    std::pair<std::vector<const mapobjects::Lane*>, LaneCost> GetLanes();
    std::vector<const mapobjects::Lane*> GetShortestLoop();
    // hash lambda + lane-cost hashmap
	std::function<size_t(const mapobjects::Lane*)> lane_hash_lambda_;
    std::unordered_map<const mapobjects::Lane*, LaneCost, decltype(lane_hash_lambda_)> *lane_cost_hash_;
    // compare lambda for min heap, priority queue <type, container, comparison lambda>
    std::function<bool(const mapobjects::Lane*, const mapobjects::Lane*)> compare_lambda_;
    std::priority_queue<const mapobjects::Lane*, std::vector<const mapobjects::Lane*>, decltype(compare_lambda_)> *queue_;
    // hash map from lane to previous lane
    std::unordered_map<const mapobjects::Lane*, const mapobjects::Lane*, decltype(lane_hash_lambda_)> *previous_hash_;
    // current steps in the planning stage <= maximum_steps
    unsigned int steps_;
	unsigned int maximum_steps_;
	// planning
	bool planning_;
    // start, goal nodes
    const mapobjects::Lane* start_lane_;
    const mapobjects::Lane* goal_lane_;
};

namespace costs
{
static constexpr float kOppositeConstantCost = 100.0f;
static constexpr float kOppositeCostMultiplier = 200.0f;
static constexpr float kAdjacentCostMultiplier = 1.3f;
static constexpr float kJncLeftCostMultiplier = 1.5f;
static constexpr float kJncRightCostMultiplier = 1.5f;
static constexpr float kJncStraightCostMultiplier = 1.0f;
static constexpr float kStraightCostMultiplier = 1.0f;
} // namespace costs

} // namespace lane_star
} // namespace planning
} // namespace freicar
#endif