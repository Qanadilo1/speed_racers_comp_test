#include "freicar_map/planning/lane_star.h"
#include "map_core/freicar_map.h"
#include "map_core/freicar_map_helper.h"
#include <limits>

namespace freicar
{
namespace planning
{
namespace lane_star
{

LaneStar::LaneStar(unsigned int max_steps) {
    lane_hash_lambda_ = [](const mapobjects::Lane* lane) -> size_t {
        return std::hash<std::string>()(lane->GetUuid().GetUuidValue());
    };
    compare_lambda_ = [&](const mapobjects::Lane *lhs,const mapobjects::Lane *rhs) -> bool {
		return lane_cost_hash_->at(lhs).f() > lane_cost_hash_->at(rhs).f();
	};
	// containers
	lane_cost_hash_ = new std::unordered_map<const mapobjects::Lane*, LaneCost, decltype(lane_hash_lambda_)> (10, lane_hash_lambda_);
    previous_hash_ = new std::unordered_map<const mapobjects::Lane*, const mapobjects::Lane*, decltype(lane_hash_lambda_)>(10, lane_hash_lambda_);
    queue_ = new std::priority_queue<const mapobjects::Lane*, std::vector<const mapobjects::Lane*>, decltype(compare_lambda_)>(compare_lambda_);

	// planning flags
    planning_ = false;
    steps_ = 0;
    maximum_steps_ = max_steps;
    start_lane_ = goal_lane_ = nullptr;
}

/* heuristic distance used for A* */
static float GetL1Distance(const mapobjects::Lane* l1, const mapobjects::Lane* l2) {
    auto ct1 = l1->GetHandlePoint();
    auto ct2 = l2->GetHandlePoint();
    return std::abs(ct1.x() - ct2.x()) +
           std::abs(ct1.y() - ct2.y()) +
           std::abs(ct1.z() - ct2.z());
}
/* adjusts the heuritic cost of a lane based on the connection to its predecessor */
static float AdjustCost(float inital_cost, mapobjects::Lane::Connection type) {
	switch (type)
	{
	case mapobjects::Lane::JUNCTION_STRAIGHT:
		return inital_cost * costs::kJncStraightCostMultiplier;
	case mapobjects::Lane::JUNCTION_LEFT:
		return inital_cost * costs::kJncLeftCostMultiplier;
	case mapobjects::Lane::JUNCTION_RIGHT:
		return inital_cost * costs::kJncRightCostMultiplier;
	case mapobjects::Lane::STRAIGHT:
		return inital_cost * costs::kStraightCostMultiplier;
	case mapobjects::Lane::OPPOSITE:
		return inital_cost * costs::kOppositeCostMultiplier;
	case mapobjects::Lane::ADJACENT_LEFT:
		return inital_cost * costs::kAdjacentCostMultiplier;
	case mapobjects::Lane::ADJACENT_RIGHT:
		return inital_cost * costs::kAdjacentCostMultiplier;
	default:
		return inital_cost;
	}
}
/* approximates the true cost of going from a lane to a neighbor */
static float GetApproximateCost(const mapobjects::Lane* lane, const mapobjects::Lane* neighbor) {
    auto type = lane->GetConnectionType(*neighbor);
    switch (type)
	{
	case mapobjects::Lane::JUNCTION_STRAIGHT:
		return (lane->GetLength() + neighbor->GetLength()) / 2;
	case mapobjects::Lane::JUNCTION_LEFT:
		return lane->GetLength();
	case mapobjects::Lane::JUNCTION_RIGHT:
	return (lane->GetLength() + neighbor->GetLength()) / 2;
	case mapobjects::Lane::STRAIGHT:
		return (lane->GetLength() + neighbor->GetLength()) / 2;
	case mapobjects::Lane::OPPOSITE:
		return (lane->GetWidth() + neighbor->GetWidth()) / 2 + costs::kOppositeConstantCost;
	case mapobjects::Lane::ADJACENT_LEFT:
		return (lane->GetWidth() + neighbor->GetWidth()) / 2;
	case mapobjects::Lane::ADJACENT_RIGHT:
		return (lane->GetWidth() + neighbor->GetWidth()) / 2;
	default:
		std::cout << "ERROR: unhandled type between " << lane->GetUuid().GetUuidValue()
				  << " and " << neighbor->GetUuid().GetUuidValue() << "\n"
				  << mapobjects::Lane::GetConnectionString(type) << std::endl;
		return 0.0f;
	}
}
void LaneStar::ExpandLane(const mapobjects::Lane* lane) {
    using Lane = mapobjects::Lane;
    //finding valid neighbors
	std::vector<const Lane*> neighbors;
	for (unsigned char connection = Lane::JUNCTION_STRAIGHT; connection < Lane::BACK; ++connection) {
		auto neighbor = lane->GetConnection(static_cast<Lane::Connection>(connection));
        if (neighbor) {
            neighbors.emplace_back(neighbor);
        }
	}
	LaneCost current_cost = (*lane_cost_hash_)[lane];
	// adding them to <node, cost> hash
	for (auto neighbor : neighbors) {
		// type of connection between neighboring lanes
		auto type = lane->GetConnectionType(*neighbor);
		// if already in the hash map
		auto itr = lane_cost_hash_->find(neighbor);
        if (itr != lane_cost_hash_->end()) {
			LaneCost updated_cost;
			updated_cost.g = current_cost.g + AdjustCost(GetApproximateCost(lane, neighbor), type);
			updated_cost.h = GetL1Distance(neighbor, goal_lane_);
			// if new cost isn't lower, skip the update
			if (updated_cost.f() >= itr->second.f())
				continue;
			// updating previous & cost hash
			(*previous_hash_)[neighbor] = lane;
			(*lane_cost_hash_)[neighbor] = updated_cost;
			// rebuilding heap w/ dirty hack (ty stack overflow)
			std::make_heap(const_cast<const mapobjects::Lane**>(&queue_->top()),
               			   const_cast<const mapobjects::Lane**>(&queue_->top()) + queue_->size(),
               			   compare_lambda_);
		} else {
			LaneCost neighbor_cost;
			// cost from last node to this one : adjusted L2
			neighbor_cost.g = current_cost.g + AdjustCost(GetApproximateCost(lane, neighbor), type);
			// cost estimate to goal : L1
			neighbor_cost.h = GetL1Distance(neighbor, goal_lane_);
			(*lane_cost_hash_)[neighbor] = neighbor_cost;
			queue_->emplace(neighbor);
			// setting previous node for neighbors
			(*previous_hash_)[neighbor] = lane;
		}
    }
}
/* plans on a lane level using A* */
std::pair<std::vector<const mapobjects::Lane*>, LaneCost> LaneStar::GetLanes() {
    planning_ = true;
    steps_ = 0;
	std::vector<const mapobjects::Lane*> path;
	LaneCost final_cost;
	// handling trivial case
	if (start_lane_ == goal_lane_) {
		path.emplace_back(start_lane_);
		return std::make_pair(path, final_cost);
	}
    // hash start node' cost
	LaneCost start_cost;
	start_cost.g = 0;
	start_cost.h = GetL1Distance(start_lane_, goal_lane_);
	(*lane_cost_hash_)[start_lane_] = start_cost;
	// just to be sure
    (*previous_hash_)[start_lane_] = start_lane_;
    queue_->emplace(start_lane_);
    bool success = false;
    while (!queue_->empty()) {   
        auto current = queue_->top();
        queue_->pop();
        if (current == goal_lane_) {
            success = true;
			final_cost = (*lane_cost_hash_)[current];
            break;
        }
        ExpandLane(current);
        if(++steps_ > maximum_steps_)
            break;
    }
	// emplacing in front to get the correct order
    if (success) {
        path.insert(path.begin(), goal_lane_);
        auto previous_lane = goal_lane_;
        do {
            previous_lane = previous_hash_->at(previous_lane);
			path.insert(path.begin(),previous_lane);
        } while (previous_lane != start_lane_);
    }
    planning_ = false;
    return std::make_pair(path, final_cost);
}
/* planning for the edge case where both start and goal positions are on the same node */
std::vector<const mapobjects::Lane*> LaneStar::GetShortestLoop() {
	using Lane = mapobjects::Lane;
	std::pair<std::vector<const mapobjects::Lane*>, LaneCost> best_plan;
	best_plan.second.g = std::numeric_limits<float>::max();
    for (unsigned char connection = Lane::JUNCTION_STRAIGHT; connection < Lane::BACK; ++connection) {
		auto child_lane = goal_lane_->GetConnection(static_cast<Lane::Connection>(connection));
        if (child_lane) {
            ResetContainers();
			start_lane_ = child_lane;
			auto new_plan = GetLanes();		
			if (new_plan.second.g < best_plan.second.g) {
				best_plan = new_plan;
			}
        }
	}
	best_plan.first.insert(best_plan.first.begin(), goal_lane_);
	return best_plan.first;
}
/* plans on a point level using the lane-based A* 
   Edge cases: if the closest point to current_pos is at index i of lane L
		1) current_pos -> L[i] goes backward, L[i+1] goes forward, L goes on for a couple of steps
		2) current_pos -> L[i] goes backward, L ends at index i
   Exception to the edge cases: when we're on offlane, the edge cases don't apply
*/

freicar::planning::Plan
LaneStar::GetPlan(const mapobjects::Point3D& start, float start_heading,
				  const mapobjects::Point3D& goal, float goal_heading, const float p2p_distance) {
	// getting the lane-level plan
    auto& map_instace = map::Map::GetInstance();
    auto start_lp = map_instace.FindClosestLanePointsWithHeading(start.x(), start.y(), start.z(), 3, start_heading)[0].first;
    auto goal_lp = map_instace.FindClosestLanePointsWithHeading(goal.x(), goal.y(), goal.z(), 3, goal_heading)[0].first;
    start_lane_ = map_instace.FindLaneByUuid(start_lp.GetLaneUuid());
    goal_lane_ = map_instace.FindLaneByUuid(goal_lp.GetLaneUuid());
	std::vector<const mapobjects::Lane*> planned_path;
	// edge case : same lane but start is in front of goal
	if (start_lane_ == goal_lane_ && start_lp.GetIndex() > goal_lp.GetIndex()) {
		// std::cout << "edge case, finding shortest loop" << std::endl;
		planned_path = GetShortestLoop();
	} else {
		planned_path = GetLanes().first;
	}
	// std::cout << "planning from (" << start.x() << ", " << start.y() << ", " << start.z() << ") to ("
	// 							   << goal.x() << ", " << goal.y() << ", " << goal.z() << ")" << std::endl;
    using Connection = mapobjects::Lane::Connection;
    
	Plan plan;
	plan.success = true;
	unsigned int lane_index = 0;
    unsigned int path_index = 0;
    float lane_offset = 0;
    float plan_offset = 0;
	bool edge_case = false;
	mapobjects::Point3D p_current;
	const mapobjects::Lane *current_lane;
	std::vector<mapobjects::Point3D> current_points;
	
    p_current.SetCoords(start.x(), start.y(), start.z());
    current_lane = map_instace.FindLaneByUuid(start_lp.GetLaneUuid());
    current_points = current_lane->GetPoints();
    // finding the index and lane_offset of the closest lane point
    lane_offset = start_lp.GetLaneOffset();
    lane_index = start_lp.GetIndex();
    // add this point anyway, no matter which lane it belongs to
    plan.emplace_back(p_current, Connection::STRAIGHT, start_lp.GetLaneUuid(), lane_offset, plan_offset);
    // if the requested point is part of the map, no need to check edge cases
    if (p_current == start_lp) {
        lane_index++;
    } else {
        float angle_cos = 0;
        if (lane_index == current_points.size() - 1) {
			// exception to the edge cases
			auto exception = (p_current.ComputeDistance(current_points[lane_index]) + 
							  p_current.ComputeDistance(current_points[lane_index - 1])) >
							  4 * current_points[lane_index].ComputeDistance(current_points[lane_index - 1]);
            angle_cos = helper::NormalizedDot(p_current, current_points[lane_index], current_points[lane_index - 1]);
            // edge case 2 : skip points from new lane until we get the right direction
            if (!exception && angle_cos > 0) {
                edge_case = true;
                // forcing "else" in the upcoming condition: if (lane_index != current_points.size())
                lane_index++;
            }
        } else {
            angle_cos = helper::NormalizedDot(p_current, current_points[lane_index], current_points[lane_index + 1]);
            // edge case 1: don't consider L[i], requested point is between [lane_index] & [lane_index + 1]
            if (angle_cos < 0) {
                lane_offset += current_points[lane_index].ComputeDistance(p_current);
                // correcting the lane offset post-hoc
                plan[0].lane_offset = lane_offset;
                lane_index++;
            }
        }
    }
	float current_distance = 0;
	Connection lane_connection = Connection::STRAIGHT;
	bool final_node = false;
	// creaing the requested plan
	mapobjects::Point3D p_next;
	while (plan.steps.back().position != goal) {
		if (lane_index != current_points.size()) {
			if (final_node)
				p_next = goal;
			else
				p_next = current_points.at(lane_index);
			auto r_dist = p_current.ComputeDistance(p_next);
			// if the distance between these 2 nodes on lane is not enough
			if (current_distance + r_dist < p2p_distance) {
				// std::cout << current_distance << " + " << r_dist << " < " << p2p_distance
				// 		  << "! final: " << std::to_string(final_node) << " index: " << lane_index << std::endl;
				current_distance += r_dist;
				p_current = p_next;
				// just add the final node
				if (final_node) {
					// std::cout << "finishing up" << std::endl;
					lane_offset += r_dist;
					plan_offset += r_dist;
					plan.emplace_back(goal, Connection::STRAIGHT, 
									  current_lane->GetUuid().GetUuidValue(),
									  lane_offset, plan_offset);
					// std::cout << "breaking" << std::endl;
					break;
				}

				// detect whether we've reached the final node on the last lane
				if (path_index == planned_path.size() - 1) {
					if (lane_index == goal_lp.GetIndex()) {
						final_node = true;
						// std::cout << "set final_node to true" << std::endl;
						continue;
					}
				}
				lane_index++;
			} else {
				// std::cout << current_distance << " + " << r_dist << " > " << p2p_distance
				// 		  << "! final: " << std::to_string(final_node) << " index: " << lane_index << std::endl;
				// amount to interpolate between the two nodes
				auto dist_diff = p2p_distance - current_distance;
				auto v_forward = (p_next - p_current).Normalized();
				p_current = p_current + v_forward * dist_diff;
				// adding the new point to the plan
				lane_offset += p2p_distance;
				plan_offset += p2p_distance;
				plan.emplace_back(p_current, lane_connection,
					current_lane->GetUuid().GetUuidValue(), lane_offset, plan_offset);
				// std::cout << "plan size: " << plan.size() << std::endl;
				current_distance = 0;
			}
		} else {
			// updating indices, lane pointer, points, lane connection
			lane_connection = current_lane->GetConnectionType(*planned_path.at(++path_index));
			current_lane = planned_path.at(path_index);
			// std::cout << "changing to lane " << current_lane->GetUuid().GetUuidValue() << std::endl;
			lane_index = 0;
			lane_offset = -current_distance;
			current_points = current_lane->GetPoints();
			// handling edge case 2
			if (edge_case) {
				float angle_cos = 0;
				// handling exception to the edge cases
				do {
					lane_index++;
					lane_offset += current_points[lane_index - 1].ComputeDistance(current_points[lane_index]);
					angle_cos = helper::NormalizedDot(p_current, current_points[0], current_points[lane_index]);
				} while (angle_cos > 0);
				edge_case = false;
			}
		}
	}
    return plan;
}

void LaneStar::ResetContainers() {
	delete lane_cost_hash_;
	delete previous_hash_;
	delete queue_;
	lane_cost_hash_ = new std::unordered_map<const mapobjects::Lane*, LaneCost, decltype(lane_hash_lambda_)> (10, lane_hash_lambda_);
    previous_hash_ = new std::unordered_map<const mapobjects::Lane*, const mapobjects::Lane*, decltype(lane_hash_lambda_)>(10, lane_hash_lambda_);
    queue_ = new std::priority_queue<const mapobjects::Lane*, std::vector<const mapobjects::Lane*>, decltype(compare_lambda_)>(compare_lambda_);
}

} // namespace lane_star
} // namespace planning
} // namespace freicar