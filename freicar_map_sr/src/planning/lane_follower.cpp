#include "freicar_map/planning/lane_follower.h"
#include "freicar_map/planning/plan.h"
#include "map_core/freicar_map.h"
#include "map_core/freicar_map_helper.h"


namespace freicar
{
namespace planning
{
namespace lane_follower
{

Plan GetPlan(const std::string& lane_uuid, float req_loffset, enums::PlannerCommand command,
             float distance, unsigned int step_count) {
    using Connection = freicar::mapobjects::Lane::Connection;
	if (command > enums::RANDOM) {
		std::cout << "[ERROR] invalid planner command used with lane follower planner" << std::endl;
		return planning::Plan(false);
	}
	// std::cout << ++count << ") " << lane_uuid << ", " << req_loffset << std::endl;
    Plan plan;
	plan.success = true;
	auto &map = freicar::map::Map::GetInstance();
	unsigned int index = 0;
    float lane_offset = 0;
    float plan_offset = 0;
	bool on_junction = false;
	freicar::mapobjects::Point3D p_current;
	const freicar::mapobjects::Lane *current_lane;
	std::vector<freicar::mapobjects::Point3D> current_points;
	current_lane = map.FindLaneByUuid(lane_uuid);
	current_points = current_lane->GetPoints();
	on_junction = current_lane->IsJunctionLane();
	// requsted point is at the end of the lane, simply start from a successor lane
	if (std::abs(req_loffset - current_lane->GetLength()) <= 1e-3) {
		const mapobjects::Lane* next_lane = nullptr;
		do {
			auto lane_connection = static_cast<Connection>(rand() % 4);
			next_lane = current_lane->GetConnection(lane_connection);
		} while (!next_lane);
		current_lane = next_lane;
		current_points = current_lane->GetPoints();
		on_junction = current_lane->IsJunctionLane();
		p_current = current_points[0];
	// checking for validity
	} else if (req_loffset > current_lane->GetLength()) {
		std::cout << "CRITICAL ERROR: requested lane offset " << req_loffset
				  << " is larger than the lane's length " << current_lane->GetLength() << std::endl;
		plan.success = false;
		return plan;
	// finding the point with that lane offset
	} else {
		for (size_t i = 0; i < current_points.size(); ++i) {
			if (i > 0)
				lane_offset += current_points[i - 1].ComputeDistance(current_points[i]);
			if (std::abs(lane_offset - req_loffset) < 1e-6) {
				p_current = current_points[i];
				index = i + 1;
				break;
			} else if (lane_offset > req_loffset) {
				auto v_backrward = (current_points[i - 1] - current_points[i]).Normalized();
				float dist_diff = lane_offset - req_loffset;
				p_current = current_points[i] + v_backrward * dist_diff;
				lane_offset = req_loffset;
				index = i;
				break;
			}
		}
	}
	float t_lane_offset = 0;
	for (size_t i = 1; i < current_points.size(); ++i) {
		auto dist_1 = current_points[i - 1].ComputeDistance(p_current);
		auto dist_2 = p_current.ComputeDistance(current_points[i]);
		auto sum_dist = current_points[i - 1].ComputeDistance(current_points[i]);
		if ((dist_1 + dist_2) - sum_dist < 1e-6) {
			t_lane_offset += dist_1;
			break;
		} else {
			t_lane_offset += sum_dist;
		}
	}
	plan.emplace_back(p_current, Connection::STRAIGHT, current_lane->GetUuid().GetUuidValue(), lane_offset, plan_offset);
	const float p2p_distance = distance / step_count;
	float current_distance = 0;
	Connection lane_connection = Connection::STRAIGHT;
	// creaing the requested plan
	while (plan.steps.size() < step_count || on_junction) {
		if (index != current_points.size()) {
			auto p_next = current_points.at(index);
			auto r_dist = p_current.ComputeDistance(p_next);
			// if the distance between these 2 nodes on lane is not enough
			if (current_distance + r_dist < p2p_distance) {
				current_distance += r_dist;
				p_current = p_next;
				index++;
			} else {
				// amount to interpolate between the two nodes
				auto dist_diff = p2p_distance - current_distance;
				auto v_forward = (p_next - p_current).Normalized();
				p_current = p_current + v_forward * dist_diff;
				// adding the new point to the plan
				lane_offset += p2p_distance;
				plan_offset += p2p_distance;
				plan.emplace_back(p_current, lane_connection,
					current_lane->GetUuid().GetUuidValue(), lane_offset, plan_offset);
				current_distance = 0;
			}
		} else {
			// go to requested lane, if failed, go to next non-junction lane
			// if failed again, then the given command has failed
			const freicar::mapobjects::Lane *next_lane = nullptr;
			switch (command)
			{
			case freicar::enums::PlannerCommand::LEFT:
				next_lane = current_lane->GetConnection(Connection::JUNCTION_LEFT);
				lane_connection = Connection::JUNCTION_LEFT;
				if (!next_lane) {
					next_lane = current_lane->GetConnection(Connection::STRAIGHT);
					lane_connection = Connection::STRAIGHT;
				}
				break;
			case freicar::enums::PlannerCommand::RIGHT:
				next_lane = current_lane->GetConnection(Connection::JUNCTION_RIGHT);
				lane_connection = Connection::JUNCTION_RIGHT;
				if (!next_lane) {
					next_lane = current_lane->GetConnection(Connection::STRAIGHT);
					lane_connection = Connection::STRAIGHT;
				}
				break;
			case freicar::enums::PlannerCommand::STRAIGHT:
				next_lane = current_lane->GetConnection(Connection::JUNCTION_STRAIGHT);
				lane_connection = Connection::JUNCTION_STRAIGHT;
				if (!next_lane) {
					next_lane = current_lane->GetConnection(Connection::STRAIGHT);
					lane_connection = Connection::STRAIGHT;
				}
				break;
			default:
				// randoming between junction straight,left,right or just straight (0 - 3)
				do {
					lane_connection = static_cast<Connection>(rand() % 4);
					next_lane = current_lane->GetConnection(lane_connection);
				} while (!next_lane);
				break;
			}
			if (next_lane) {
				// updating indices, lane pointer, points
				current_lane = next_lane;
				index = 0;
				lane_offset = -current_distance;
				current_points = current_lane->GetPoints();
				on_junction = current_lane->IsJunctionLane();
			} else {
				plan.success = false;
				break;
			}
		}
	}
	return plan;
}

/*
   Edge cases: if the closest point to current_pos is at index i of lane L
		1) current_pos -> L[i] goes backward, L[i+1] goes forward, L goes on for a couple of steps
		2) current_pos -> L[i] goes backward, L ends at index i
*/
Plan GetPlan(const mapobjects::Point3D& current_position, enums::PlannerCommand command,
             float distance, unsigned int step_count) {
    using Connection = freicar::mapobjects::Lane::Connection;
	if (command > enums::RANDOM) {
		std::cout << "[ERROR] invalid planner command used with lane follower planner" << std::endl;
		return planning::Plan(false);
	}
    Plan plan;
	plan.success = true;
	auto &map = freicar::map::Map::GetInstance();  // access map -> objects??
	unsigned int index = 0;
    float lane_offset = 0;
    float plan_offset = 0;
	bool edge_case = false;
	bool on_junction = false;
	freicar::mapobjects::Point3D p_current;
	const freicar::mapobjects::Lane *current_lane;
	std::vector<freicar::mapobjects::Point3D> current_points;
	// if a new agent is spawning, get a random start position, go from there
	p_current.SetCoords(current_position.x(), current_position.y(), current_position.z());
	auto p_closest = map.FindClosestLanePoints(current_position.x(),
											   current_position.y(),
											   current_position.z(),
											   1)[0].first;
	current_lane = map.FindLaneByUuid(p_closest.GetLaneUuid());
	current_points = current_lane->GetPoints();
	on_junction = current_lane->IsJunctionLane(); // ARE WE AT A JUNCTION
	// finding the index and lane_offset of the closest lane point
	lane_offset = p_closest.GetLaneOffset();
	index = p_closest.GetIndex();
	// add this point anyway, no matter which lane it belongs to
	plan.emplace_back(p_current, Connection::STRAIGHT, p_closest.GetLaneUuid(), lane_offset, plan_offset);
	// if the requested point is part of the map, no need to check edge cases
	if (p_current == p_closest) {
		index++;
	} else {
		float angle_cos = 0;
		if (index == current_points.size() - 1) {
			angle_cos = freicar::helper::NormalizedDot(p_current, current_points[index], current_points[index - 1]);
			// edge case 2 : skip points from new lane until we get the right direction
			if (angle_cos > 0) {
				edge_case = true;
				// forcing "else" in the upcoming if-else (line #200)
				index++;
			}
		} else {
			angle_cos = freicar::helper::NormalizedDot(p_current, current_points[index], current_points[index + 1]);
			// edge case 1: don't consider L[i], requested point is between [index] & [index + 1]
			if (angle_cos < 0) {
				lane_offset += current_points[index].ComputeDistance(p_current);
				// correcting the lane offset post-hoc
				plan[0].lane_offset = lane_offset;
				index++;
			}
		}
	}
	const float p2p_distance = distance / step_count;
	float current_distance = 0;
	Connection lane_connection = Connection::STRAIGHT;
	// creaing the requested plan
	while (plan.steps.size() < step_count || on_junction) {
		if (index != current_points.size()) {
			auto p_next = current_points.at(index);
			auto r_dist = p_current.ComputeDistance(p_next);
			// if the distance between these 2 nodes on lane is not enough
			if (current_distance + r_dist < p2p_distance) {
				current_distance += r_dist;
				p_current = p_next;
				index++;
			} else {
				// amount to interpolate between the two nodes
				auto dist_diff = p2p_distance - current_distance;
				auto v_forward = (p_next - p_current).Normalized();
				p_current = p_current + v_forward * dist_diff;
				// adding the new point to the plan
				lane_offset += p2p_distance;
				plan_offset += p2p_distance;
				plan.emplace_back(p_current, lane_connection, 
					current_lane->GetUuid().GetUuidValue(), lane_offset, plan_offset);
				current_distance = 0;
			}
		} else {
			// go to requested lane, if failed, go to next non-junction lane
			// if failed again, then the given command has failed
			const freicar::mapobjects::Lane *next_lane = nullptr;
			switch (command)
			{
			case freicar::enums::PlannerCommand::LEFT:
				next_lane = current_lane->GetConnection(Connection::JUNCTION_LEFT);
				lane_connection = Connection::JUNCTION_LEFT;
				if (!next_lane) {
					next_lane = current_lane->GetConnection(Connection::STRAIGHT);
					lane_connection = Connection::STRAIGHT;
				}
				break;
			case freicar::enums::PlannerCommand::RIGHT:
				next_lane = current_lane->GetConnection(Connection::JUNCTION_RIGHT);
				lane_connection = Connection::JUNCTION_RIGHT;
				if (!next_lane) {
					next_lane = current_lane->GetConnection(Connection::STRAIGHT);
					lane_connection = Connection::STRAIGHT;
				}
				break;
			case freicar::enums::PlannerCommand::STRAIGHT:
				next_lane = current_lane->GetConnection(Connection::JUNCTION_STRAIGHT);
				lane_connection = Connection::JUNCTION_STRAIGHT;
				if (!next_lane) {
					next_lane = current_lane->GetConnection(Connection::STRAIGHT);
					lane_connection = Connection::STRAIGHT;
				}
				break;
			default:
				// randoming between junction straight,left,right or just straight (0 - 3)
				do {
					lane_connection = static_cast<Connection>(rand() % 4);
					next_lane = current_lane->GetConnection(lane_connection);
				} while (!next_lane);
				break;
			}
			if (next_lane) {
				// updating indices, lane pointer, points
				current_lane = next_lane;
				index = 0;
				lane_offset = -current_distance;
				current_points = current_lane->GetPoints();
				on_junction = current_lane->IsJunctionLane();
				// handling edge case 2
				if (edge_case) {
					float angle_cos = 0;
					do {
						index++;
						lane_offset += current_points[index - 1].ComputeDistance(current_points[index]);
						angle_cos = freicar::helper::NormalizedDot(p_current, current_points[0], current_points[index]);
					} while (angle_cos > 0);
					edge_case = false;
				}
			} else {
				plan.success = false;
				break;
			}
		}
	}
	return plan;
}

} // namespace lane_follower

} // namespace planner
} // namespace freicar

