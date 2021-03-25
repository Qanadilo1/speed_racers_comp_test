#ifndef __ROS_MAP_INFO__
#define __ROS_MAP_INFO__

#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

namespace freicar
{
namespace helper
{
inline float ToScalar(const geometry_msgs::Vector3& v3) {
    return std::sqrt(v3.x * v3.x + v3.y * v3.y + v3.z * v3.z);
}

}
namespace logic
{

/* this is the struct that is published on /car_localization and is used in
   ros_carla_agent to help agents behave naturally. it's also the base class
   for JunctionAgent which is used to determine right of way at junctions.
*/
struct AgentMapInfo
{

public:
    AgentMapInfo();
    AgentMapInfo(const std::string& name_in, const std::string& lane_uuid_in, float l_offset, const geometry_msgs::Vector3& vel);
    void UpdatePose(const geometry_msgs::TransformStamped &new_pose, const ros::Time &time);
    float vx() const;
    float vy() const;
    float vz() const;
    geometry_msgs::Vector3 GetVelocityVector() const;
    float GetVelocityScalar() const;
    std::string name;
    std::string lane_uuid;
    float lane_offset;
    geometry_msgs::TransformStamped current_pose;
private:
    float velocity_scalar_;
    geometry_msgs::Vector3 velocity_;
    ros::Time last_tick_;
};

} // namespace agent
} // namespace freicar

#endif