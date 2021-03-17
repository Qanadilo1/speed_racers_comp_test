#ifndef __PLAN_H__
#define __PLAN_H__
#include "map_core/freicar_map_objects.h"

namespace freicar
{
namespace planning
{
struct PlanStep {
    std::string lane_uuid;
    float plan_offset;
    float lane_offset;
    mapobjects::Point3D position;
    mapobjects::Lane::Connection path_description;
    PlanStep(const mapobjects::Point3D& pos, mapobjects::Lane::Connection desc, 
             const std::string& l_uuid, float l_offset, float p_offset) : lane_uuid(l_uuid),
                    plan_offset(p_offset), lane_offset(l_offset), position(pos), path_description(desc) {}
    PlanStep() {}
};

struct Plan {
    Plan() {success = false;}
    Plan(bool plan_success) : success(plan_success) {}

    std::vector<PlanStep> steps;
    bool success;
    template <typename... Args>
    void emplace_back(Args... args) {
        steps.emplace_back(args...);
    }
    PlanStep& operator[](long index) {
        return (index >= 0) ? steps[index] : *(steps.end() + index);
    }
    const PlanStep& operator[](long index) const {
        return (index >= 0) ? steps[index] : *(steps.cend() + index);
    }
    void clear() {
        success = false;
        steps.clear();
    }
    size_t size() const {
        return steps.size();
    }
    bool empty() const {
        return steps.empty();
    }
    void append(Plan&& extension) {
        float offset_increase = 0.0f;
        if (!steps.empty())
            offset_increase = steps.back().plan_offset;
        for (auto& plan_step : extension.steps) {
            plan_step.plan_offset += offset_increase;
            steps.emplace_back(std::move(plan_step));
        }
    }
};

} // namespace planning
} // namespace freicar

#endif