
/******************************************************************************
  
  Audi Autonomous Driving Cup 2018
  Team frAIsers
  AUTHOR: Fabien Jenne
  
  Thrift protocol for communicating the global map over Thrift
  
******************************************************************************/

namespace py map_thrift
namespace cpp map_thrift

typedef i64 int64

/* message op codes */
enum MessageOp {
    ADD = 1,
    UPDATE_WHOLE = 2,
    UPDATE_PART = 3,
    DELETE = 4,
    POSE_UPDATE = 5,
    POSE_DELETE = 6,
}

enum LaneMarkingType {
    SOLID = 1,
    DASHED = 2,
    CENTER_SOLID = 3,
    CENTER_DASHED = 4,
}

enum LaneType {
    NORMAL = 1,
    CAR_LANE = 2,
    PEDESTRIAN_LANE = 3,
}

enum LaneDirection {
    RIGHT = 1,
    LEFT = 2,
}

enum LaneObjectType {
    STOPLINE = 1,
    CROSSWALK = 2,
    PARKING = 3,
    SIGN = 4,
}

/* struct Uuid {
    1: required int64 lower,
    2: required int64 upper,
} */

struct Uuid {
    1: required string uuid,
}

struct Point3D {
    1: required double x,
    2: required double y,
    3: required double z = 0,  // set for multilevel maps
}

struct Pose {
    1: required Point3D position,
    2: required double orientation,
}

struct Pivot {
    1: Uuid id,
    2: Pose pose,
    3: bool is_valid = false,
}

/* holds parts of a map */
struct MapContainer {
    1: list<MapPart> map_parts,
    2: Pivot pivot,
}

struct LaneObjectList {
    1: list<Stopline> stoplines,
    2: list<Crosswalk> crosswalks,
    3: list<Parking> parking_lots,
    4: list<Roadsign> roadsigns,
}

/* a single part of the map containing different types */
struct MapPart {
    1: list<Lane> lanes,
    2: list<LaneGroup> lane_groups,
    3: list<LaneMarking> lane_markings,
    4: LaneObjectList lane_objects,  // LaneObjectList has to come before MapPart, as it is a custom datatype
    // 4: list<RoadSegment> RoadSegments,
}

struct LaneMarking {
    1: LaneMarkingType type,
    2: list<Point3D> points,
    3: required Uuid id,                     // uuid
    4: bool visibility,
}

struct LaneMarkingContainer {
    1: list<Uuid> left,
    2: list<Uuid> right,
}

struct Lane {
    1: LaneMarkingContainer lane_markings,     // uuids
    2: LaneDirection dir,
    3: list<Uuid> outgoing_connections,
    4: list<Uuid> incoming_connections,
    5: required LaneType type,
    6: list<Point3D> points,
    7: double width,
    8: required Uuid id,                     // uuid
    9: Point3D handle_point,
    10: bool visibility,
    11: int64 height,
    12: int64 junction_id,
}

struct LaneGroup {
    1: required Uuid id,
    2: list<Uuid> lanes_right,
    3: list<Uuid> lanes_left,       // holds uuids of lanes
}

struct LaneObjectBase {
    1: required Uuid id,
    2: LaneObjectType type,
    3: Uuid lg_id,
    4: list<Uuid> l_ids,
    5: double offset,
}

struct Stopline {
    1: required LaneObjectBase lane_object,
}

struct Crosswalk {
    1: required LaneObjectBase lane_object,  // offset is from the middle of the crosswalk lines
}

struct Parking {
    1: required LaneObjectBase lane_object,
    2: int64 number_of_lots,
    3: double parking_height,
    4: double parking_width,
    5: double line_width,
    6: list<list<Point3D>> outer_points,
}

struct Roadsign {
    1: required LaneObjectBase lane_object,
    2: string sign_type,
    3: Point3D position,
    4: double rotation,
}

/* struct RoadSegment {

} */

struct PointViz {
    1: required Point3D point,
    2: required double radius,
}

struct MapMessage {
    1: required MessageOp op,
    2: required MapContainer container,
}

struct CarMessage {
    1: required MessageOp op,
    2: required Pose pose,  // optional doesn't work here!
    3: required int64 n_points_viz,
    4: required list<PointViz> points_viz,
}

// read from left top row-wise
struct ThriftImage {
    1: required i16 width,
    2: required i16 height,
    3: required i8 channels,
    4: required binary bytes,
    5: required Pose pose,
}

service MapComm {
    void ping(),
    // get the internal map from the server
    // MapMessage getMap(),
    // MapMessage getMapPart(1:list<Uuid> parts),
    // send a map to the server
    bool sendMapMessage(1:MapMessage map_complete),
    bool sendCarPose(1:CarMessage car_pose),
    bool sendThriftImage(1:ThriftImage image),
    // bool sendMapPart(1:MapMessage map_part);
}
