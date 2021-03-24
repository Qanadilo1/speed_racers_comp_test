#ifndef __FREICAR_LANE_FOLLOWER_H__
#define __FREICAR_LANE_FOLLOWER_H__

#include <string>
#include <vector>

#include "freicar_common/shared/planner_cmd.h"
#include "freicar_map/planning/plan.h"
namespace freicar
{
namespace planning
{
namespace lane_follower
{

freicar::planning::Plan GetPlan(const mapobjects::Point3D& current_position, enums::PlannerCommand command,
                                float distance, unsigned int step_count);
freicar::planning::Plan GetPlan(const std::string& lane_uuid, float req_loffset, enums::PlannerCommand command,
                                float distance, unsigned int step_count);
} // namespace lane_follower
} // namespace planner
} // namespace freicar
#endif