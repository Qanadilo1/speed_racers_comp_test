#include "freicar_map/logic/map_info.h"
#include "map_core/freicar_map.h"
#include <cmath>

namespace freicar
{
namespace logic
{

AgentMapInfo::AgentMapInfo() {
    lane_offset = -1;
    velocity_.x = 0;
    velocity_.y = 0;
    velocity_.z = 0;
    velocity_scalar_ = 0;
}
AgentMapInfo::AgentMapInfo(const std::string& name_in, const std::string& lane_uuid_in, float l_offset, const geometry_msgs::Vector3& vel) {
    name = name_in;
    lane_uuid = lane_uuid_in;
    lane_offset = l_offset;
    velocity_ = vel;
}
void AgentMapInfo::UpdatePose(const geometry_msgs::TransformStamped &new_pose, const ros::Time &time) {
    auto dx = new_pose.transform.translation.x - current_pose.transform.translation.x;
    auto dy = new_pose.transform.translation.y - current_pose.transform.translation.y;
    auto dz = new_pose.transform.translation.z - current_pose.transform.translation.z;
    auto dt = (time - last_tick_).toSec();
    velocity_.x = dx / dt;
    velocity_.y = dy / dt;
    velocity_.z = dz / dt;
    velocity_scalar_ = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
    current_pose = new_pose;
    last_tick_ = time;
}
float AgentMapInfo::vx() const {
    return velocity_.x;
}
float AgentMapInfo::vy() const {
    return velocity_.y;
}
float AgentMapInfo::vz() const {
    return velocity_.z;
}
float AgentMapInfo::GetVelocityScalar() const {
    return velocity_scalar_;
}
geometry_msgs::Vector3 AgentMapInfo::GetVelocityVector() const {
    return velocity_;
}

} // namespace agent
} // namespace freicar