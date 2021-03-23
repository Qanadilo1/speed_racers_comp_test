#include "map_core/freicar_map_helper.h"

namespace freicar
{
namespace helper
{
/* returns the dot prodcut between normalized p1-base, p2-base */
float NormalizedDot(const mapobjects::Point3D &base, const mapobjects::Point3D &p1, const mapobjects::Point3D &p2) {
	mapobjects::Point3D v1 = (p1 - base).Normalized();
	mapobjects::Point3D v2 = (p2 - base).Normalized();
	return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z();
}

} // namespace map_helper
} // namespace freicar