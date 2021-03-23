#ifndef __RIGHT_OF_WAY_H__
#define __RIGHT_OF_WAY_H__

#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <geometry_msgs/TransformStamped.h>
#include "freicar_common/FreiCarAgentLocalization.h"
#include "freicar_map/logic/map_info.h"
namespace freicar 
{
namespace logic
{
struct JunctionAgent : public AgentMapInfo {
    // enum values intentionally match freicar::mapobjects::Lane::Connection for static_cast
    enum Intent : unsigned char {
        GOING_STRAIGHT = 0,
        GOING_LEFT = 1,
        GOING_RIGHT = 2,
        NONE = 7
    };
    JunctionAgent() {}
    JunctionAgent(const freicar_common::FreiCarAgentLocalization& loc_msg) :
                  AgentMapInfo(loc_msg.name, loc_msg.lane_uuid, loc_msg.lane_offset, loc_msg.velocity) {
        current_pose = loc_msg.current_pose;
        intent = Intent::NONE;
    }
    JunctionAgent(const std::string& a_name, const std::string& l_uuid, float l_offset,
                  const geometry_msgs::Vector3& vel,
                  const geometry_msgs::TransformStamped& pose, Intent intnt = Intent::NONE) :
                  AgentMapInfo(a_name, l_uuid, l_offset, vel), intent(intnt) {
        current_pose = pose;
    }
    JunctionAgent(const AgentMapInfo& a_info, Intent intnt = Intent::NONE) :
                  AgentMapInfo(a_info.name, a_info.lane_uuid, a_info.lane_offset,
                               a_info.GetVelocityVector()), intent(intnt) {
        current_pose = a_info.current_pose;
    }
    Intent intent = Intent::NONE;
    bool IsOnRightHandSideOf(const JunctionAgent& observed_agent) const;
    bool HasRightOfWay(const JunctionAgent& observed_agent, bool ignore_intent) const;
};

/* returns whether the agent has right of way & whether the junction is already occupied.
   if not it also returns the name of which other observed agent it lost to.
   IMPORTANT: the function assumes the calling agent is on a lane that leads to a junction
*/
std::tuple<bool, bool, std::string>
GetRightOfWay(const JunctionAgent& agent, const std::vector<JunctionAgent>& observed_agents,
              bool consider_velocities = false,
              bool ignore_intent = false);

} // namespace logic
} // namespace freicar

#endif