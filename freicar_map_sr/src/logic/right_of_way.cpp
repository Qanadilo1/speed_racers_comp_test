#include "freicar_map/logic/right_of_way.h"
#include "map_core/freicar_map.h"

#include <Eigen/Dense>
#include <bitset>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tuple>
namespace freicar 
{
namespace logic
{
/* some lanes before junctions are very short. sometimes the agents are
   not considered because they're on the preceding lane to the short lane
   which causes "crashes" at junctions since they're not observed. this 
   function returns the correct junction id as well as their distance to
   that junction.
*/
static std::pair<int, float> GetCorrectJunctionID(const JunctionAgent& observed_agent, map::Map& map_instance){
	auto observed_lane = map_instance.FindLaneByUuid(observed_agent.lane_uuid);
	int j_id = map_instance.GetCurrentJunctionID(observed_agent.lane_uuid);
	// on a junction
	if (j_id != -1)
		return std::make_pair(j_id, 0.0f);
	j_id = map_instance.GetUpcomingJunctionID(observed_agent.lane_uuid);
	float distance = observed_lane->GetLength() - observed_agent.lane_offset;
	if (j_id != -1) {
		// if stopping at junction
		return std::make_pair(j_id, distance);
	}
	auto next_lane = observed_lane->GetConnection(mapobjects::Lane::Connection::STRAIGHT);
	if (!next_lane)
		return std::make_pair(-1, 0.0f);
	j_id = map_instance.GetUpcomingJunctionID(next_lane->GetUuid().GetUuidValue());
	if (j_id != -1) {
		distance += next_lane->GetLength();
		return std::make_pair(j_id, distance);
	}
	return std::make_pair(-1, 0.0f);
}
/* returns whether this agent is on the right hand side of an observer */
bool JunctionAgent::IsOnRightHandSideOf(const JunctionAgent& observer_agent) const {
	auto observer_lane = map::Map::GetInstance().FindLaneByUuid(observer_agent.lane_uuid);
	auto observer_pts = observer_lane->GetPointsByReference();
	auto p_l1 = observer_pts.back();
	auto p_l2 = observer_pts[observer_pts.size() - 2];
	float yaw = std::atan2(p_l1.y() - p_l2.y(), p_l1.x() - p_l2.x());
	// getting the pose of this agent in observer_agent's frame. if y > 0, it's on LHS
	Eigen::Matrix3f observer_tf;
	observer_tf << std::cos(yaw), -std::sin(yaw), observer_agent.current_pose.transform.translation.x,
				   std::sin(yaw),  std::cos(yaw), observer_agent.current_pose.transform.translation.y,
				   0			, 0			, 1;
	Eigen::Vector3f old_pos(current_pose.transform.translation.x, current_pose.transform.translation.y, 1);
	Eigen::Vector3f new_pos = observer_tf.inverse() * old_pos;
	if (new_pos.y() == 0)
		std::cout << "[ERROR] 0.000001 chance of hitting this edge case" << std::endl;
	if (new_pos.y() > 0)
		return false;
	return true;
}
/* checks the whether the first agent has priority over observed agent. 
   IMPORTANT: usually observed_agent does not have an intent, so only the intent
   of the current agent is checked, but further functionallity can be added easily */
bool JunctionAgent::HasRightOfWay(const JunctionAgent& observed_agent, bool ignore_intent) const {
	enum JunctionSign : int {
		AGENT_STOP = 0b0001,
		OBSERVED_STOP = 0b0010,
		AGENT_ROW = 0b0100,
		OBSERVED_ROW = 0b1000,
		NONE = 0b0000
	};
	int junction_signs = JunctionSign::NONE;
	auto agent_lane_signs = map::Map::GetInstance().FindLaneByUuid(lane_uuid)->GetRoadSigns();
	auto observed_lane = map::Map::GetInstance().FindLaneByUuid(observed_agent.lane_uuid);
	std::vector<const mapobjects::Roadsign*> observed_lane_signs;
	// if the lane leads to a junction
	if (map::Map::GetInstance().GetUpcomingJunctionID(observed_agent.lane_uuid) != -1)
		observed_lane_signs = observed_lane->GetRoadSigns();
	// if the next lane does
	else {
		auto observed_next_lane = observed_lane->GetConnection(mapobjects::Lane::STRAIGHT);
		if (!observed_next_lane) {
			std::cout << "ERROR: observed agent should've been filtered out in the calling function" << std::endl;
			return true;
		}
		observed_lane_signs = observed_next_lane->GetRoadSigns();
	}
	for (auto agent_sign : agent_lane_signs) {
		if (agent_sign->GetSignType() == "Stop")
			junction_signs |= JunctionSign::AGENT_STOP;
		if (agent_sign->GetSignType() == "RightOfWay")
			junction_signs |= JunctionSign::AGENT_ROW;
	}
	for (auto observed_sign : observed_lane_signs) {
		if (observed_sign->GetSignType() == "Stop")
			junction_signs |= JunctionSign::OBSERVED_STOP;
		if (observed_sign->GetSignType() == "RightOfWay")
			junction_signs |= JunctionSign::OBSERVED_ROW;
	}
	switch (junction_signs) {
	case AGENT_STOP | OBSERVED_STOP:
	case AGENT_ROW | OBSERVED_ROW:
		if (this->IsOnRightHandSideOf(observed_agent) &&        // current on RHS of observed
			!observed_agent.IsOnRightHandSideOf(*this)) {
			return true;
		}
		else if (!this->IsOnRightHandSideOf(observed_agent) &&  // observed on RHS of current
				 observed_agent.IsOnRightHandSideOf(*this)) {
			// if (this->intent == Intent::GOING_RIGHT)	// exception to RHS the rule
			// 	return true;
			return false;
		}
		else if (!this->IsOnRightHandSideOf(observed_agent) &&  // both on LHS of eachother
				 !observed_agent.IsOnRightHandSideOf(*this)) {
			if (!ignore_intent && this->intent == Intent::GOING_LEFT)
				return false;
			else if (this->intent == Intent::GOING_STRAIGHT ||
					 this->intent == Intent::GOING_RIGHT || ignore_intent)
				return true;
			else
				std::cout << "[WARNING] unhandled case: " << name
						  << " and " << observed_agent.name << " on LHS w/ unknown intent" << std::endl;
		}
		else
			std::cout << "[WARNING] unhandled case: " << name
					  << " and " << observed_agent.name << " on RHS" << std::endl;
		break;
	// case AGENT_STOP:
	// case OBSERVED_ROW:
	case AGENT_STOP | OBSERVED_ROW:
		return false;
	// case AGENT_ROW:
	// case OBSERVED_STOP:
	case AGENT_ROW | OBSERVED_STOP:
		return true;
	default:
		std::cout << "[WARNING] unhandled case: " << name
				  << " and " << observed_agent.name << ", "
				  << std::bitset<8>(junction_signs) << std::endl;
		break;
	}

	return false;
}
/* returns whether the given agent has right of way over the vector of observed agents */
// TODO: replace agent' heading with that of the last two lane nodes
std::tuple<bool, bool, std::string>
GetRightOfWay(const JunctionAgent& agent, const std::vector<JunctionAgent>& observed_agents, bool consider_velocities, bool ignore_intent) {
	auto &map_instance = map::Map::GetInstance();
	auto current_junction = map_instance.GetUpcomingJunctionID(agent.lane_uuid);
	// if on the junction, we're the winner anyway. otherwise if not leading to a junction
	// skip the whole thing
	if (current_junction == -1) {
		if (map_instance.GetCurrentJunctionID(agent.lane_uuid) != -1)
			return std::make_tuple(true, true, agent.name);
		return std::make_tuple(false, false, agent.name);
	}
	for (auto& observed_agent : observed_agents) {
		// if on the same or prev lane, observed agent is irrelevant for right of way
		auto observed_lane = map_instance.FindLaneByUuid(observed_agent.lane_uuid);
		auto observed_next = observed_lane->GetConnection(mapobjects::Lane::STRAIGHT);
		bool observed_on_prev_lane = observed_next && observed_next->GetUuid().GetUuidValue() == agent.lane_uuid;
		if (observed_agent.lane_uuid == agent.lane_uuid || observed_on_prev_lane) {
			continue;
		}
		// if the observed agent is off road
		if (observed_agent.lane_uuid == "?") {
			continue;
		}
		auto [observed_junction, dist_to_junction] = GetCorrectJunctionID(observed_agent, map_instance);
		// if not at the same junction
		if (current_junction != observed_junction) {
			continue;
		}
		else {
			// observed agent is on the junction
			if (observed_lane->IsJunctionLane()) {
				return std::make_tuple(false, true, observed_agent.name);
			}
			else {
				// if too far away from junction (2.5 meters)
				if (dist_to_junction > 2.5f) {
					continue;
				}
			}
		}
		bool ignoring_traffic_rules = false;
		if (consider_velocities) {
			auto dx = observed_lane->GetLength() - observed_agent.lane_offset;
			auto velo = std::sqrt(observed_agent.vx() * observed_agent.vx() +
								  observed_agent.vy() * observed_agent.vy() +
								  observed_agent.vz() * observed_agent.vz());
			if (velo > 1e-2 && (dx / velo) < 0.5f) {
				ignoring_traffic_rules = true;
			}
		}
		if (!agent.HasRightOfWay(observed_agent, ignore_intent) || ignoring_traffic_rules) {
			return std::make_tuple(false, false, observed_agent.name);
		}
	}
	return std::make_tuple(true, false, agent.name);
}

} // namespace logic
} // namespace freicar