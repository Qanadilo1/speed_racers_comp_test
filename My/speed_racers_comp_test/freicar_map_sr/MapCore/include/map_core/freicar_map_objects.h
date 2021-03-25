#ifndef MAP_MAPCORE_MAPOBJECTS_H_
#define MAP_MAPCORE_MAPOBJECTS_H_

#include <math.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <functional>

namespace freicar
{
// defining map class for friendship
namespace map
{
	class Map;
}
namespace mapobjects 
{
/* provide some map object properties */
enum LaneDirection {
	RIGHT = 1,
	LEFT = 2
};

enum LaneType {
	NORMAL = 1,
	CAR_LANE = 2,
	PEDESTRIAN_LANE = 3,
};

enum LaneMarkingType {
	SOLID = 1,
	DASHED = 2,
	CENTER_SOLID = 3,
	CENTER_DASHED = 4,
};

enum LaneObjectType {
	LO_STOPLINE = 1,
	LO_SIGN = 2,
	LO_CROSSWALK = 3,
	LO_PARKING = 4,
};

// ____________________________________________________________________________
/* a class for holding python generated uuid4's */
class Uuid {
 private:
	// std::array<int64_t, 2> uuid;
	std::string uuid_;

 public:
	explicit Uuid(std::string id_in) : uuid_(id_in) {}
	// default ctor should generate random uuid string
	Uuid();
	~Uuid();
	std::string GetUuidValue() const;

	bool operator==(const Uuid rhs) const {
		return uuid_ == rhs.uuid_;
	}
	bool operator!=(const Uuid rhs) const {
		return uuid_ != rhs.uuid_;
	}
};
// ____________________________________________________________________________
/* a class for storing discrete map points in 3d space. Coords are in cm. */
class Point3D {
 protected:
	float x_ = 0.;
	float y_ = 0.;
	float z_ = 0.;
	// meh

 public:
	Point3D() : x_(0.), y_(0.), z_(0.) {}
	Point3D(float x_in, float y_in) : x_(x_in), y_(y_in), z_(0.) {}
	Point3D(float x_in, float y_in, float z_in) : x_(x_in), y_(y_in), z_(z_in) {}
	// defining some operators to avoid needless back and forth conversion to Eigen::Vector3fs
	Point3D operator-(const Point3D &rhs) const;
	Point3D operator+(const Point3D &rhs) const;
	Point3D operator/(const float rhs) const;
	Point3D operator*(const float rhs) const;
	bool operator==(const Point3D &rhs) const;
	bool operator!=(const Point3D &rhs) const;
	Point3D Cross(const Point3D &point) const;
	int GetRelativeDirection(Point3D forward, Point3D adjacent) const;
	void SetCoords(std::tuple<float, float, float> tuple);
	void SetCoords(float x, float y, float z = 0);
	void SetX(float);
	void SetY(float);
	void SetZ(float);

	std::tuple<float, float, float> GetCoords() const;
	float x() const;
	float y() const;
	float z() const;
	float ComputeDistance(const mapobjects::Point3D& point) const;
	float ComputeDistance(float x, float y, float z) const;
	float GetNorm() const;
	float GetSquaredNorm() const;
	Point3D Normalized();
	std::string str();
	// uuid functions
private:
	friend class freicar::map::Map;
};

// ____________________________________________________________________________
/* a class for storing metadata for a specific lane. */
class LaneInfo {
 public:
	// computed from the lane itself
	bool no_overtaking = false;
	bool has_stoplines = false;
	bool has_crosswalks = false;
	bool has_parking = false;
	// these are encoded in Streetdesigner
	bool is_ground = true;
	bool is_ramp_up = false;
	bool is_ramp_down = false;
	bool is_merging = false;
	bool is_tunnel = false;

 public:
	LaneInfo() {}
	LaneInfo(bool overtake_in, bool stoplines_in, bool crosswalks_in, bool parking_in) : no_overtaking(overtake_in),
		has_stoplines(stoplines_in), has_crosswalks(crosswalks_in), has_parking(parking_in) {}
};

// ____________________________________________________________________________
/* a class for holding the lane markings of a Lane MapObject */
class LaneMarkingContainer {
 public:
	LaneMarkingContainer(std::vector<Uuid> left_lm, std::vector<Uuid> right_lm) : \
		left_(left_lm), right_(right_lm) {}

	~LaneMarkingContainer();

 private:
	std::vector<Uuid> left_;
	std::vector<Uuid> right_;

 public:
	std::vector<Uuid> GetLeftLaneMarkings() const;
	std::vector<Uuid> GetRightLaneMarkings() const;
};

// ____________________________________________________________________________
/* a virtual class from which all identifiable map objects should inherit */
class MapObject {
 protected:
	Uuid id_;
	explicit MapObject(Uuid id_in) : id_(id_in) {}

	// default ctor generates Uuid
	MapObject();

 public:
	Uuid GetUuid() const;

	/* make  ~tor pure virtual with virtual ...() = 0 to prevent instanciating.
	 * Must instanciate child objects */
	virtual ~MapObject() = 0;
};

// ____________________________________________________________________________
/* a class to inherit from for mapobject pose functionality */
class Pose {
 protected:
	float x_;
	float y_;
	float z_;
	float t_;

 public:
	Pose() : x_(0), y_(0), z_(0), t_(0) {}
	Pose(float x_in, float y_in, float z_in) :
		x_(x_in), y_(y_in), z_(z_in), t_(0) {}
	Pose(float x_in, float y_in, float z_in, float t_in) :
		x_(x_in), y_(y_in), z_(z_in), t_(t_in) {}
	explicit Pose(Point3D p);
	Pose(Point3D p, float t_in);
	~Pose();

	std::tuple<float, float, float, float> GetXYZT();
	float x() const;
	float y() const;
	float z() const;
	float t() const;
	mapobjects::Point3D GetPoint3D() const;
	void ConvertFromADTF();
	void SetPose(float x_in, float y_in, float z_in, float t_in);
};

// ____________________________________________________________________________
/* a class for storing the map pivot point */
class Pivot : public MapObject{
 public:
	Pivot() : MapObject() {}
	Pivot(Uuid id_in, Pose pose_in) : id_(id_in), pose_(pose_in) {}
	~Pivot();
	mapobjects::Pose GetPose() const;

 private:
	Uuid id_;
	Pose pose_;
	void SetPose(const Pose &pose);
	friend class freicar::map::Map;
};

// ____________________________________________________________________________
/* a class for holding lane group data */
class LaneGroup : public MapObject {
 public:
	LaneGroup(Uuid id_in, std::vector<Uuid> l_l, std::vector<Uuid> l_r) :
		MapObject(id_in), lanes_left_(l_l), lanes_right_(l_r), is_junction_(false) {}
	~LaneGroup();
	// new
	bool IsJunction() const;
	std::vector<Uuid> GetLeftLanes() const;
	std::vector<Uuid> GetRightLanes() const;
private:
	void SetAsJunction(bool is_junction);
	std::vector<Uuid> lanes_left_;
	std::vector<Uuid> lanes_right_;
	// new
	bool is_junction_;
	friend class freicar::map::Map;
};

// ____________________________________________________________________________
/* a class to store lane marking points */
class LaneMarking : public MapObject {
 public:
	LaneMarking(Uuid id_in, LaneMarkingType t, std::vector<Point3D> p, bool v) :
		MapObject(id_in), type_(t), points_(p), visibility_(v) {}

	~LaneMarking();
	LaneMarkingType GetLaneMarkingType() const;
	bool GetVisibility() const;
	std::vector<Point3D> GetPoints() const;
 private:
	LaneMarkingType type_;
	std::vector<Point3D> points_;
	bool visibility_;

	std::vector<Point3D>& GetPointsByReference();
	friend class freicar::map::Map;	
};

// ____________________________________________________________________________
/* a base class for lane objects */
class LaneObject : public MapObject {
 public:
	LaneObject(Uuid id_in, LaneObjectType type_in, Uuid lg_id_in, std::vector<Uuid> lane_ids_in, float offset_in)
		: MapObject(id_in), type_(type_in), lg_id_(lg_id_in), lane_ids_(lane_ids_in), offset_(offset_in) {}

	~LaneObject();

 protected:
	LaneObjectType type_;
	Uuid lg_id_;
	std::vector<Uuid> lane_ids_;
	float offset_;

 public:
	LaneObjectType GetLaneObjectType() const;
	Uuid GetLaneGroupId() const;
	std::vector<Uuid> GetLaneIds() const;
	float GetOffset() const;
};

class Lane;
// ____________________________________________________________________________
/* a class for stoplines */
class Stopline : public LaneObject {
 public:
	Stopline(Uuid id_in, Uuid lg_id_in, std::vector<Uuid> lane_ids_in, float offset_in)
		: LaneObject(id_in, LaneObjectType::LO_STOPLINE, lg_id_in, lane_ids_in, offset_in) {}
	~Stopline();
	Point3D GetPosition() {return position;}
private:
	// set in Lane::SetStopline, used for visulization in Map
	Point3D position, v_forward, v_right;
	friend class freicar::mapobjects::Lane;
	friend class freicar::map::Map;
};

// ____________________________________________________________________________
/* a class for crosswalks */
class Crosswalk : public LaneObject {
 public:
	Crosswalk(Uuid id_in, Uuid lg_id_in, std::vector<Uuid> lane_ids_in, float offset_in) :
		LaneObject(id_in, LaneObjectType::LO_CROSSWALK, lg_id_in, lane_ids_in, offset_in) {}
	~Crosswalk();
};

// ____________________________________________________________________________
/* a class for parking spaces */
class Parking : public LaneObject {
 public:
	Parking(Uuid id_in, Uuid lg_id_in, std::vector<Uuid> lane_ids_in, float offset_in, int n_parking_lots_in,
			float parking_height_in, float parking_width_in,float linewidth_in, std::vector<std::vector<Point3D>> outer_points_in) :
			LaneObject(id_in, LaneObjectType::LO_PARKING, lg_id_in, lane_ids_in, offset_in), n_parking_lots_(n_parking_lots_in),
			parking_height_(parking_height_in), parking_width_(parking_width_in), linewidth_(linewidth_in), outer_points_(outer_points_in) {}
	~Parking();

 private:
	int n_parking_lots_;
	float parking_height_;
	float parking_width_;
	float linewidth_;
	std::vector<std::vector<Point3D>> outer_points_;

 public:
	int GetNumberOfParkingLots() const;
	float GetWidth() const;
	float GetHeight() const;
	float GetLineWidth() const;
	std::vector<std::vector<Point3D>> GetOuterPoints() const;
};

// ____________________________________________________________________________
/* a class for roadsigns */
class Roadsign : public LaneObject {
 public:
	Roadsign(Uuid id_in, Uuid lg_id_in, std::vector<Uuid> lane_ids_in, float offset_in, std::string sign_type_in,
		mapobjects::Point3D position_in, float rotation_in) : LaneObject(id_in, LaneObjectType::LO_SIGN, lg_id_in, lane_ids_in, offset_in),
		sign_type_(sign_type_in), position_(position_in), rotation_(rotation_in) {}
	~Roadsign();

	std::string GetSignType() const;
	Point3D GetPosition() const;
	float GetRotation() const;

private:
	void SetPosition(float x, float y, float z);
	std::string sign_type_;
	Point3D position_;
	float rotation_;
	friend class freicar::map::Map;
};

// ____________________________________________________________________________
/* a class for storing lane information */
class Lane : public MapObject {
public:
	// enum for storing the lane-lane relationship
	enum Connection : unsigned char {
		JUNCTION_STRAIGHT = 0,
		JUNCTION_LEFT = 1,
		JUNCTION_RIGHT = 2,
		STRAIGHT = 3,
		OPPOSITE = 4,
		ADJACENT_LEFT = 5,
		ADJACENT_RIGHT = 6,
		BACK = 7,
		EMPTY = 8,
	};
	Lane(Uuid id_in, LaneMarkingContainer lm, LaneDirection d, std::vector<Uuid> c_in,
		std::vector<Uuid> c_out, LaneType t, std::vector<Point3D> p, float w, Point3D hp, bool v, int h, int j_id) :
		MapObject(id_in), lane_markings_(lm), dir_(d), connections_out_(c_out), connections_in_(c_in),
		type_(t), points_(p), width_(w), handle_point_(hp), visibility_(v), height_(h), junction_id_(j_id) {
			// setting up hash table
			hash_lambda_ = [](const Connection &type) {
				return std::hash<int>()(static_cast<int>(type));
			};
			connection_hash_ = new std::unordered_map<Connection, Lane*, decltype(hash_lambda_)>(8, hash_lambda_);
			stopline_ = nullptr;
			// calculating length
			length_ = 0;
			// dummy lane has no lane points => if condition needed
			if (points_.size() > 1) {
				for (size_t i = 0; i < points_.size() - 1; ++i)
					length_ += points_[i].ComputeDistance(points_[i + 1]);
			}

		}
	~Lane();
private:
	LaneMarkingContainer lane_markings_;
	LaneDirection dir_;
	std::vector<Uuid> connections_out_;
	std::vector<Uuid> connections_in_;
	LaneType type_;
	std::vector<Point3D> points_;
	float width_;
	float length_;
	Point3D handle_point_;
	bool visibility_;
	int height_ = 0;
	int junction_id_;
	// new
	Uuid parent_uuid_;
	std::function<size_t(const Connection &type)> hash_lambda_;
	std::unordered_map<Connection, Lane*, decltype(hash_lambda_)> *connection_hash_;
	// pointer to lane objects
	std::vector<const Roadsign*> road_signs_;
	const Stopline* stopline_;
public:
	bool IsOffroad() const;
	std::vector<const Roadsign*> GetRoadSigns() const;
	bool HasRoadSign(std::string sign_name) const;
	const Stopline* GetStopLine() const;
	Uuid GetParentUuid() const;
	std::pair<mapobjects::Point3D, float> FindPointFromOffset(float offset);
	std::vector<Point3D>& GetPointsByReference();
	// connection hash
	const Lane* GetConnection(Connection connection) const;
	Connection GetConnectionType(const Lane &lane) const;
	static std::string GetConnectionString(Connection connection);
	void PrintLane() const;
	std::pair<Point3D, int> GetClosestLanePoint(const mapobjects::Point3D &point, bool reverse = false, int start_index = -1) const;
	LaneType GetLaneType() const;
	LaneDirection GetLaneDirection() const;
	Point3D GetHandlePoint() const;
	float GetWidth() const;
	LaneMarkingContainer GetLaneMarkingContainer() const;
	std::vector<Uuid> GetOutConnections() const;
	std::vector<Uuid> GetInConnections() const;
	std::vector<Point3D> GetPoints() const;
	bool GetVisibility() const;
	int GetHeight() const;
    int GetJunctionID() const;
	bool IsJunctionLane() const;
	float GetLength() const;
private:
	// these functions will only be used by the Map class for post processing
	void SetParentUuid(const Uuid& lane_group);
	void RegisterConnection(Connection connection, Lane* lane);
	void AddRoadSign(const Roadsign* sign);
	void SetStopLine(Stopline* stopline);
	Point3D& GetHandlePointByReference();
	friend class freicar::map::Map;
};

// ____________________________________________________________________________
/* a class for junctions, used for determining right of way */
class Junction 
{
public:
	enum Type {
		T_INTESERSECTION,
		CROSS_INTESERSECTION,
		ROUNDABOUT,
		NONE
	};
	Junction(int id) : id_(id) {}
	std::vector<Uuid> GetLanes() const;
	std::vector<Uuid> GetLaneGroups() const;
	std::vector<Uuid> GetIncomingLanes() const;
	std::vector<Uuid> GetOutgoingLanes() const;
	std::vector<Point3D> GetBoundaryPoints() const;
	int GetID() const;
	Point3D GetHandlePoint() const;
	Type GetType();

private:
	bool IsBoundaryPoint(const Point3D& point) const;
	void AddLane(const Lane& lane);
	void PostProcess();

	
	std::vector<Uuid> lanes_, outgoing_lanes_, incoming_lanes_;
	std::vector<Uuid> lanegroups_;
	int id_;
	// junction' boundary polygon
	std::vector<Point3D> boundary_points_;
	Point3D handle_point_;
	Type type_;
	friend class freicar::map::Map;
};

// ____________________________________________________________________________
/* a class for lane points that is used to build a kd-tree. It inherits from Point3D
   and adds metadata to each point in a lane.
 */
class LanePoint3D : public Point3D
{
public:
	LanePoint3D() {};
	std::string GetLaneUuid();
	unsigned int GetIndex();
	float GetHeading();
	float GetLaneOffset();
	Point3D AsPoint3D();

private:
	std::string lane_uuid_;
	unsigned int index_;
	float heading_;
	float lane_offset_;
	friend class freicar::map::Map;
};

} // namespace mapobjects
} // namespace freicar

// extending std to confuse the next dev
namespace std {
	inline std::string to_string(const freicar::mapobjects::Point3D& point) {
		return "(" + std::to_string(point.x()) + ", " +
				 	 std::to_string(point.y()) + ", " +
				 	 std::to_string(point.z()) + ")";
	}
} // namespace std

#endif
