#include <iostream>
#include <algorithm>
#include <string>
#include "map_core/freicar_map_objects.h"
#include "map_core/freicar_map_helper.h"

namespace freicar
{
namespace mapobjects
{

// implement the default ctor for MapObject to generate a random uuid
MapObject::MapObject()  {
	id_ = Uuid();
}
// ____________________________________________________________________________
/* implement the virtual ~tors for all MapObjects */
MapObject::~MapObject() {}

// MapObjects
Lane::~Lane() {}
LaneGroup::~LaneGroup() {}
LaneMarking::~LaneMarking() {}
LaneMarkingContainer::~LaneMarkingContainer() {}
LaneObject::~LaneObject() {}
Stopline::~Stopline() {}
Crosswalk::~Crosswalk() {}
Parking::~Parking() {}
Roadsign::~Roadsign() {}
Pivot::~Pivot() {}
Pose::~Pose() {}

// ____________________________________________________________________________
/* implement Uuid class */
Uuid::Uuid() {
	// NOT USED. All uuids are coming from the map
	// boost::uuids::uuid uuid_boost = boost::uuids::random_generator()();
	// uuid_ = boost::lexical_cast<std::string>(uuid_boost);
	// include if needed:
	// <boost/uuid/uuid.hpp>
	// <boost/uuid/uuid_generators.hpp>
	// <boost/uuid/uuid_io.hpp>
	// <boost/lexical_cast.hpp>
}
Uuid::~Uuid() {}

/* implement getter for uuids of map objects */
Uuid MapObject::GetUuid() const {
	return id_;
}
// getter for Uuid
std::string Uuid::GetUuidValue() const {
	return uuid_;
}

// getters for Point3D
std::tuple<float, float, float> Point3D::GetCoords() const {
	return std::make_tuple(x_, y_, z_);
}
float Point3D::x() const {
	return x_;
}
float Point3D::y() const {
	return y_;
}
float Point3D::z() const {
	return z_;
}
/* minus operator overload */
Point3D Point3D::operator-(const Point3D &rhs) const {
	return Point3D(x_ - rhs.x_, y_ - rhs.y_, z_ - rhs.z_);
}
/* plus operator overload */
Point3D Point3D::operator+(const Point3D &rhs) const {
	return Point3D(x_ + rhs.x_, y_ + rhs.y_, z_ + rhs.z_);
}
/* division operator overload */
Point3D Point3D::operator/(const float rhs) const {
	return Point3D(this->x_ / rhs, this->y_ / rhs, this->z_ / rhs);
}
/* multiplication operator overload */
Point3D Point3D::operator*(const float rhs) const {
	return Point3D(this->x_ * rhs, this->y_ * rhs, this->z_ * rhs);
}
/* equality operator overload */
bool Point3D::operator==(const Point3D &rhs) const {
	return x_ == rhs.x_ && y_ == rhs.y_ && z_ == rhs.z_;
}
/* inequality operator overload */
bool Point3D::operator!=(const Point3D &rhs) const {
	return x_ != rhs.x_ || y_ != rhs.y_ || z_ != rhs.z_;
}
/* returns the cross product of the current point with the given one */
Point3D Point3D::Cross(const Point3D& point) const {
	return Point3D(y_ * point.z_ - z_ * point.y_,
				   z_ * point.x_ - x_ * point.z_,
				   x_ * point.y_ - y_ * point.x_);
}
/* returns -1 if the adjacent point is on the left side, +1 if on the right, 0 if dead straight */
int Point3D::GetRelativeDirection(Point3D forward, Point3D adjacent) const {
	forward = forward - *this;
	forward.SetZ(0);
	adjacent = adjacent - *this;
	adjacent.SetZ(0);
	// it made sense mathematically but had to flip L & R
	// maybe because map & unreal are flipped. idk
	if (adjacent.Cross(forward).z() > 0)
		return -1;
	else if (adjacent.Cross(forward).z() < 0)
		return +1;
	return 0;
}
void Point3D::SetCoords(std::tuple<float, float, float> tuple){
	std::tie(x_, y_, z_) = tuple;
}
void  Point3D::SetCoords(float x, float y, float z) {
	x_ = x;
	y_ = y;
	z_ = z;
}
void Point3D::SetX(float x) {
	x_ = x;
}
void Point3D::SetY(float y) {
	y_ = y;
}
void Point3D::SetZ(float z) {
	z_ = z;
}

/* function to compute the euclidean distance between two points */
float Point3D::ComputeDistance(const mapobjects::Point3D& point) const {
	return std::sqrt(std::pow(x_ - point.x_, 2) +
					 std::pow(y_ - point.y_, 2) +
					 std::pow(z_ - point.z_, 2));
}
/* function to compute the euclidean distance between two points */
float Point3D::ComputeDistance(float x, float y, float z) const {
	return std::sqrt(std::pow(x_ - x, 2) +
					 std::pow(y_ - y, 2) +
					 std::pow(z_ - z, 2));
}
/* sets the lane uuid */
float Point3D::GetNorm() const {
	return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
}
float Point3D::GetSquaredNorm() const {
	return x_ * x_ + y_ * y_ + z_ * z_;
}
Point3D Point3D::Normalized() {
	float sq = std::sqrt(x_ * x_ + y_ * y_ + z_ * z_);
	x_ = x_ / sq;
	y_ = y_ / sq;
	z_ = z_ / sq;
	return *this;
}
std::string Point3D::str() {
	return "(" + std::to_string(x_) + ", " +
				 std::to_string(y_) + ", " +
				 std::to_string(z_) + ")";
}
// ____________________________________________________________________________
/* implement Pivot class */
mapobjects::Pose Pivot::GetPose() const {
	return pose_;
}
void Pivot::SetPose(const Pose& pose) {
	pose_ = pose;
}
// ____________________________________________________________________________
// getters for LaneMarkingContainer
std::vector<mapobjects::Uuid> LaneMarkingContainer::GetLeftLaneMarkings() const {
	return left_;
}
std::vector<mapobjects::Uuid> LaneMarkingContainer::GetRightLaneMarkings() const {
	return right_;
}

// ____________________________________________________________________________
// getters for LaneGroup
std::vector<mapobjects::Uuid> LaneGroup::GetLeftLanes() const {
	return lanes_left_;
}
std::vector<mapobjects::Uuid> LaneGroup::GetRightLanes() const {
	return lanes_right_;
}
/* detects whether the current lanegroup is a junction */
void LaneGroup::SetAsJunction(bool is_junction) {
	is_junction_ = is_junction;
}
/* is the lanegroup a junction */
bool LaneGroup::IsJunction() const {
	return is_junction_;
}
// ____________________________________________________________________________
// getters for Lane
std::vector<const Roadsign*> Lane::GetRoadSigns() const {
	return road_signs_;	
}

bool Lane::HasRoadSign(std::string sign_name) const {
	auto result = std::find_if(road_signs_.begin(),
							   road_signs_.end(),
							   [sign_name] (const Roadsign* rs) {
		return rs->GetSignType() == sign_name;
	});
	return result != road_signs_.end();
}
const Stopline* Lane::GetStopLine() const {
	return stopline_;
}
void Lane::AddRoadSign(const Roadsign* sign) {
	road_signs_.emplace_back(sign);
}
void Lane::SetStopLine(Stopline* stopline) {
	float lane_offset = stopline->GetOffset() / 100; // still in cm
	for (size_t i = 0; i < points_.size() - 1; ++i) {
		lane_offset -= points_[i].ComputeDistance(points_[i + 1]);
		if (lane_offset <= 0) {
			stopline->v_forward = (points_[i + 1] - points_[i]).Normalized();
			stopline->position = points_[i + 1] + stopline->v_forward * lane_offset;
			stopline->v_right.SetCoords(stopline->v_forward.y(), -stopline->v_forward.x());
			break;
		}
	}
	stopline_ = stopline;
}
mapobjects::Point3D Lane::GetHandlePoint() const {
	return handle_point_;
}
mapobjects::Point3D& Lane::GetHandlePointByReference() {
	return handle_point_;
}
/* returns the point and its heading, given the lane offset */
std::pair<mapobjects::Point3D, float> Lane::FindPointFromOffset(float offset) {
	Point3D point;
	float heading = 0.0f;
	float p2p_distance = 0.0f;
	for (size_t i = 1; i < points_.size(); ++i) {
		p2p_distance = points_[i - 1].ComputeDistance(points_[i]);
		if (p2p_distance >= offset) {
			auto v_forward = (points_[i] - points_[i - 1]).Normalized();
			point = points_[i - 1] + v_forward * offset;
			heading = std::atan2(v_forward.y(), v_forward.x());
			break;
		} else {
			offset -= p2p_distance;
		}
	}
	return std::make_pair(point, heading);
}
float Lane::GetWidth() const {
	// TODO: remove division by hundred when changed in thrift map
	return width_ / 100;
}
mapobjects::LaneMarkingContainer Lane::GetLaneMarkingContainer() const {
	return lane_markings_;
}
mapobjects::LaneDirection Lane::GetLaneDirection() const {
	return dir_;
}
mapobjects::LaneType Lane::GetLaneType() const {
	return type_;
}
std::vector<mapobjects::Uuid> Lane::GetOutConnections() const {
	return connections_out_;
}
std::vector<mapobjects::Uuid> Lane::GetInConnections() const {
	return connections_in_;
}
std::vector<mapobjects::Point3D> Lane::GetPoints() const {
	return points_;
}
bool Lane::GetVisibility() const {
	return visibility_;
}
int Lane::GetHeight() const {
	return height_;
}
int Lane::GetJunctionID() const {
    return junction_id_;
}
float Lane::GetLength() const {
	return length_;
}
/* returns a reference to the vector lane points. use: making a dense node graph  */
std::vector<Point3D>& Lane::GetPointsByReference() {
	return points_;
}
/* sets the parent Uuid, post-hoc */
void Lane::SetParentUuid(const Uuid& parent_uuid) {
	parent_uuid_ = parent_uuid;
}
/* retunrs the parent lanegroup' uuid */
Uuid Lane::GetParentUuid() const {
	return parent_uuid_;
}
/* returns the closest lanepoint to the given Point3D, starting backwards if reverse = true, from start_index.
   the function tries to return the local minimum to avoid searching the entire vector. start_index can be reused
   from the last found matching point in the list */
[[deprecated("deprecated function")]]
std::pair<Point3D, int> Lane::GetClosestLanePoint(const mapobjects::Point3D &point, bool reverse, int start_index) const {
	float last_dist = 1e6;
	float min_dist = 1e6;
	int index = -1;
	if (reverse) {
		for (int i = (start_index == -1) ? points_.size() - 1 : start_index; i >= 0; --i) {
			float current_dist = points_[i].ComputeDistance(point);
			// hit global minimum
			if (current_dist > last_dist)
				break;
			// found a lower distance
			if (current_dist < min_dist) {
				min_dist = current_dist;
			}
			last_dist = current_dist;
			index = i;
		}
	} else {
		for (unsigned int i = (start_index == -1) ? 0 : start_index; i < points_.size(); ++i) {
			float current_dist = points_[i].ComputeDistance(point);
			// hit global minimum
			if (current_dist > last_dist)
				break;
			// found a lower distance
			if (current_dist < min_dist) {
				min_dist = current_dist;
			}
			last_dist = current_dist;
			index = i;
		}
	}
	return std::make_pair(points_[index], index);
}
/* adds the requested connection to the hash map*/
void Lane::RegisterConnection(Connection connection, Lane* lane) {
	if (connection_hash_->find(connection) != connection_hash_->end()) {
		std::cerr << "lane hash: connection type " << static_cast<int>(connection) << " already exists" << std::endl;
		return;
	}
	(*connection_hash_)[connection] = lane;
}
/* returns the lane that has the requested connection to the current lane, nullptr otherwise */
const Lane* Lane::GetConnection(Connection connection) const {
	auto it = connection_hash_->find(connection);
	if (it != connection_hash_->end())
		return it->second;
	return nullptr;
}
/* returns the type of connection the lane has to the argument, EMPTY otherwise */
Lane::Connection Lane::GetConnectionType(const Lane &lane) const {
	for (auto& it : *connection_hash_) {
		if (it.second == &lane)
			return it.first;
	}
	return Connection::EMPTY;
}
std::string Lane::GetConnectionString(Connection connection) {
	switch (connection)
	{
	case Connection::JUNCTION_STRAIGHT:
		return "junc straight";
		break;
	case Connection::JUNCTION_LEFT:
		return "junc left";
		break;
	case Connection::JUNCTION_RIGHT:
		return "junc right";
		break;
	case Connection::STRAIGHT:
		return "straight";
		break;
	case Connection::OPPOSITE:
		return "opposite";
		break;
	case Connection::ADJACENT_LEFT :
		return "adj left";
		break;
	case Connection::ADJACENT_RIGHT:
		return "adj right";
		break;
	default:
		return "empty";
		break;
	}
}
void Lane::PrintLane() const {
	std::cout << "lane " << GetUuid().GetUuidValue() << " has\n";
	for (auto &it : *connection_hash_) {
		std::cout << "\t" << GetConnectionString(it.first) << " to " << it.second->GetUuid().GetUuidValue() << std::endl;
	}
}
bool Lane::IsJunctionLane() const {
	return junction_id_ != -1;
}
/* returns whether this is the dummy lane that represents offlane */
bool Lane::IsOffroad() const {
	return id_.GetUuidValue() == "?";
}
// ____________________________________________________________________________
// getters for LaneMarking
mapobjects::LaneMarkingType LaneMarking::GetLaneMarkingType() const {
	return type_;
}
bool LaneMarking::GetVisibility() const {
	return visibility_;
}
std::vector<mapobjects::Point3D> LaneMarking::GetPoints() const {
	return points_;
}
std::vector<Point3D>& LaneMarking::GetPointsByReference() {
	return points_;
}

// ____________________________________________________________________________
// constructors for Pose
Pose::Pose(Point3D p) {
	std::tie(x_, y_, z_) = p.GetCoords();
	t_ = 0.0;
}
Pose::Pose(Point3D p, float t_in) {
	std::tie(this->x_, this->y_, this->z_) = p.GetCoords();
	t_ = t_in;
}
// getters for Pose
std::tuple<float, float, float, float> Pose::GetXYZT() {
	return std::make_tuple(this->x_, this->y_, this->z_, this->t_);
}
float Pose::x() const {
	return x_;
}
float Pose::y() const {
	return y_;
}
float Pose::z() const {
	return z_;
}
float Pose::t() const {
	return fmod(t_, 2 * M_PI);
}
mapobjects::Point3D Pose::GetPoint3D() const {
	return mapobjects::Point3D(this->x_, this->y_, this->z_);
}
void Pose::ConvertFromADTF() {
	this->y_ *= -1;
	this->t_ *= -1;
}
// setters for Pose
void Pose::SetPose(float x_in, float y_in, float z_in, float t_in) {
	this->x_ = x_in;
	this->y_ = y_in;
	this->z_ = z_in;
	this->t_ = t_in;
}

// ____________________________________________________________________________
// getters for LaneObject
mapobjects::LaneObjectType LaneObject::GetLaneObjectType() const {
	return type_;
}
mapobjects::Uuid LaneObject::GetLaneGroupId() const {
	return lg_id_;
}
std::vector<mapobjects::Uuid> LaneObject::GetLaneIds() const {
	return lane_ids_;
}
float LaneObject::GetOffset() const {
	return offset_;
}

// ____________________________________________________________________________
// getters for LaneObject::Parking
int Parking::GetNumberOfParkingLots() const {
	return n_parking_lots_;
}
float Parking::GetWidth() const {
	return parking_width_;
}
float Parking::GetHeight() const {
	return parking_height_;
}
float Parking::GetLineWidth() const {
	return linewidth_;
}
std::vector<std::vector<mapobjects::Point3D>> Parking::GetOuterPoints() const {
	return outer_points_;
}
// ____________________________________________________________________________
// getters for LaneObject::Roadsign

std::string Roadsign::GetSignType() const{
	return sign_type_;
}
Point3D Roadsign::GetPosition() const{
	return position_;
}
float Roadsign::GetRotation() const{
	return rotation_;
}
/* sets the roadsign position (used for subtracting the pivot coordinates) */
void Roadsign::SetPosition(float x, float y, float z) {
	position_.SetCoords(x, y, z);
}
// ____________________________________________________________________________
// Junction class
/* adds lane to the lane storage of this junction. assumes each junction lane only has a single
   incoming & outgoing connection.
*/
void Junction::AddLane(const Lane& lane) {
	if (lane.GetJunctionID() == -1) {
		std::cout << "ERROR: lane " << lane.GetUuid().GetUuidValue() 
				  << " is not a junction lane but used as one" << std::endl;
		return;
	}
	else if (lane.GetJunctionID() != id_) {
		std::cout << "ERROR: lane " << lane.GetUuid().GetUuidValue() 
				  << " is not a part of junction #" << id_ << std::endl;
		return;
	}
	// adding lanes
	lanes_.emplace_back(lane.GetUuid());
	auto i_lane = lane.GetInConnections()[0];
	if (std::find(incoming_lanes_.begin(), incoming_lanes_.end(), i_lane) == incoming_lanes_.end())
		incoming_lanes_.emplace_back(i_lane);		
	auto o_lane = lane.GetOutConnections()[0];
	if (std::find(outgoing_lanes_.begin(), outgoing_lanes_.end(), o_lane) == outgoing_lanes_.end())
		outgoing_lanes_.emplace_back(o_lane);

	// adding boundary
	auto lane_points = lane.GetPoints();
	if (!IsBoundaryPoint(lane_points.front()))
		boundary_points_.emplace_back(lane_points.front());
	if (!IsBoundaryPoint(lane_points.back()))
		boundary_points_.emplace_back(lane_points.back());
	// adding lanegroup
	bool new_lanegroup = true;
	for (auto &lanegroup : lanegroups_) {
		if (lanegroup == lane.GetParentUuid()){
			new_lanegroup = false;
			break;
		}
	}
	if (new_lanegroup)
		lanegroups_.emplace_back(lane.GetParentUuid());
}
/* checks whether the given arg is already considered a boundary point */
inline bool Junction::IsBoundaryPoint(const Point3D& point) const {
	for (auto &boundary_point : boundary_points_) {
		if (point == boundary_point)
			return true;
	}
	return false;
}
/* returns the (sorted) boundary points */
std::vector<Point3D> Junction::GetBoundaryPoints() const {
	return boundary_points_;
}
/* returns the handle point for the junction */
Point3D Junction::GetHandlePoint() const {
	return handle_point_;
}
std::vector<Uuid> Junction::GetLaneGroups() const {
	return lanegroups_;
}
/* sorts the boundary points into a proper polygon, sets the handle point & type */
void Junction::PostProcess() {
	if (boundary_points_.size() < 3) {
		std::cout << "ERROR: not enough lanes are added to junction #" << id_ << std::endl;
		return;
	}
	// sort the vector by distance from [0]
	for (size_t i = 0; i < boundary_points_.size(); ++i) {
		size_t idx = i + 1;
		float min_dist = boundary_points_[i].ComputeDistance(boundary_points_[idx]);
		for (size_t j = i + 2; j < boundary_points_.size(); ++j) {
			auto dist = boundary_points_[i].ComputeDistance(boundary_points_[j]);
			if (dist < min_dist) {
				min_dist = dist;
				idx = j;
			}
		}
		// need to swap [idx] with [i + 1]
		if (idx != i + 1) {
			auto temp_point = boundary_points_[i + 1];
			boundary_points_[i + 1] = boundary_points_[idx];
			boundary_points_[idx] = temp_point;
		}
	}
	// setting the handle point (currently used for visualization)
	handle_point_ = Point3D(0, 0, 0);
	for (size_t i = 0; i < boundary_points_.size() ; ++i)
		handle_point_ = handle_point_ + boundary_points_[i];
	handle_point_ = handle_point_ / boundary_points_.size();

	// setting junction type
	// TODO: this will not generalize to all types of junctions
	type_ = (incoming_lanes_.size() == 3) ? Type::T_INTESERSECTION :
			(incoming_lanes_.size() == 4) ? Type::CROSS_INTESERSECTION :
											Type::NONE;
	// ROS_WARN("junction %d has %u lanegroups", id_, (unsigned int)lanegroups_.size());
}
/* returns a list of lanes that are inside the junction */
std::vector<Uuid> Junction::GetLanes() const {
	return  lanes_;
}
/* returns a list of lanes that lead to the junction */
std::vector<Uuid> Junction::GetIncomingLanes() const {
	return  incoming_lanes_;
}
/* returns a list of lanes that exit the junction */
std::vector<Uuid> Junction::GetOutgoingLanes() const {
	return outgoing_lanes_;
}
int Junction::GetID() const {
	return id_;
}
Junction::Type Junction::GetType() {
	return type_;
}

// ____________________________________________________________________________
// LanePoint3D class
std::string LanePoint3D::GetLaneUuid() {
	return lane_uuid_;
}
unsigned int LanePoint3D::GetIndex() {
	return index_;
}
float LanePoint3D::GetHeading() {
	return heading_;
}
float LanePoint3D::GetLaneOffset() {
	return lane_offset_;
}
Point3D LanePoint3D::AsPoint3D() {
	return Point3D(x_, y_, z_);
}

} // namespace mapobjects
} // namespace freicar