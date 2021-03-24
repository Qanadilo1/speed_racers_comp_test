#include "map_core/freicar_type_conversion.h"
/*
  this is where i stashed a lot of the ugliness
  the purpose of this header file is to provide type conversions 
  between objects from thrift_map & mapobjects. they really don't
  need to be part of any class. 
*/

namespace freicar
{
namespace convert
{
// some prototypes to fix some errors
std::vector<mapobjects::Uuid> ToMapObjectsUuids(const std::vector<map_thrift::Uuid> &uuids);

mapobjects::Pivot ToMapObjectsPivot(const map_thrift::Pivot& thrift_pivot) {
	return mapobjects::Pivot(mapobjects::Uuid(thrift_pivot.id.uuid), 
                             mapobjects::Pose(mapobjects::Point3D(thrift_pivot.pose.position.x, 
                                                                  thrift_pivot.pose.position.y,
                                                                  thrift_pivot.pose.position.z), 
                                   								  thrift_pivot.pose.orientation));
}

/* converts a map_thrift::LaneMarkingType to maobjects::LaneMarkingType */
mapobjects::LaneMarkingType ToMapObjectLaneMarkingType(const map_thrift::LaneMarkingType::type& type) {
    return static_cast<mapobjects::LaneMarkingType>(type);
}
/* converts map_thrift::LaneMarkingType to mapobjects::LaneMarkingType */
mapobjects::LaneType ToMapObjectLaneType(const map_thrift::LaneType::type& type) {
    return static_cast<mapobjects::LaneType>(type);
}
/* converts map_thrift::LaneDirection to mapobjects::LaneDirection */
mapobjects::LaneDirection ToMapObjectLaneDirection(const map_thrift::LaneDirection::type& type) {
    return static_cast<mapobjects::LaneDirection>(type);
}
/* converts map_thrift::LaneMarkingContainer to mapobjects::LaneMarkingContainer */
mapobjects::LaneMarkingContainer ToMapObjectLaneMarkingContainer(const map_thrift::LaneMarkingContainer& lmc) {
    mapobjects::LaneMarkingContainer mo_lmc = mapobjects::LaneMarkingContainer(ToMapObjectsUuids(lmc.left),
                                                                               ToMapObjectsUuids(lmc.right));
    return mo_lmc;
}

/* converts mapobjects::Point3D to map_thrift::Point3D.
   here the GLOBAL_OFFSET_X/Y for Streetdesigner should be added again if it is used */
map_thrift::Point3D ToThriftPoint(const mapobjects::Point3D& point) {
    map_thrift::Point3D r;
    std::tie(r.x, r.y, r.z) = point.GetCoords();
    return r;
}

/* converts a vector of mapobjects::Point3D to map_thrift::Point3D */
std::vector<map_thrift::Point3D> ToThriftPoints(const std::vector<mapobjects::Point3D>& points) {
    std::vector<map_thrift::Point3D> r;
    int n = points.size();
    r.reserve(n);
    for (int i = 0; i < n; ++i) {
        map_thrift::Point3D p = ToThriftPoint(points[i]);
        r.push_back(p);
    }
    return r;
}
/* converts a vector of mapobjects::Uuid to map_thrift::Uuid */
std::vector<map_thrift::Uuid> ToThriftUuids(const std::vector<mapobjects::Uuid>& uuids) {
    std::vector<map_thrift::Uuid> r;
    int n = uuids.size();
    r.reserve(n);
    map_thrift::Uuid id;
    for (int i = 0; i < n; ++i) {
        id.uuid = uuids[i].GetUuidValue();
        r.push_back(id);
    }
    return r;
}
/* converts mapobjects::Uuid to map_thrift::Uuid */
map_thrift::Uuid ToThriftUuid(const mapobjects::Uuid& uuid) {
    map_thrift::Uuid id;
    id.uuid = uuid.GetUuidValue();
    return id;
}

/* converts mapobjects::LaneMarkingType to map_thrift::LaneMarkingType */
map_thrift::LaneMarkingType::type ToThriftLaneMarkingType(const mapobjects::LaneMarkingType& type) {
    return static_cast<map_thrift::LaneMarkingType::type>(type);
}
/* converts mapobjects::LaneMarkingType to map_thrift::LaneMarkingType */
map_thrift::LaneType::type ToThriftLaneType(const mapobjects::LaneType& type) {
    return static_cast<map_thrift::LaneType::type>(type);
}
/* converts mapobjects::LaneDirection to map_thrift::LaneDirection */
map_thrift::LaneDirection::type ToThriftLaneDirection(const mapobjects::LaneDirection& dir) {
    return static_cast<map_thrift::LaneDirection::type>(dir);
}
/* converts a vector of mapobjects::LaneMarkingContainer to a vector of map_thrift::LaneMarkingContainer */
map_thrift::LaneMarkingContainer ToThriftLaneMarkingContainer(const mapobjects::LaneMarkingContainer& lmc) {
    map_thrift::LaneMarkingContainer to = map_thrift::LaneMarkingContainer();
    to.left = ToThriftUuids(lmc.GetLeftLaneMarkings());
    to.right = ToThriftUuids(lmc.GetRightLaneMarkings());
    return to;
}

/* converts a map_thrift::Point3D to mapobjects::Point3D */
std::vector<mapobjects::Point3D> ToMapObjectPoints(const std::vector<map_thrift::Point3D>& points) {
    std::vector<mapobjects::Point3D> r;
    int n = points.size();
    r.reserve(n);
    for (int i = 0; i < n; ++i) {
        r.push_back(mapobjects::Point3D(points[i].x, points[i].y, points[i].z));
    }
    return r;
}
/* converts a map_thrift::Uuid to mapobjects::Uuid */
std::vector<mapobjects::Uuid> ToMapObjectsUuids(const std::vector<map_thrift::Uuid>& uuids) {
    std::vector<mapobjects::Uuid> r;
    int n = uuids.size();
    r.reserve(n);
    for (int i = 0; i < n; ++i) {
        r.push_back(mapobjects::Uuid(uuids[i].uuid));
    }
    return r;
}
/* converts a map_thrift::LaneMarking to maobjects::LaneMarking implicitly via the indices (must match) */
mapobjects::LaneMarking ToMapObjectsLanemarking(const map_thrift::LaneMarking& lm) {
    return mapobjects::LaneMarking(mapobjects::Uuid(lm.id.uuid),
        						   // original author: static_cast might be faster, but explicit enum matching is safer
        						   // comment: your code is the opposite of safe & fast. congrats. i changed them to static_cast
        						   convert::ToMapObjectLaneMarkingType(lm.type),
        						   convert::ToMapObjectPoints(lm.points),
        						   lm.visibility);
}

/* converts a map_thrift::Lane to maobjects::Lane */
mapobjects::Lane ToMapObjectsLane(const map_thrift::Lane& lane) {
    return mapobjects::Lane(mapobjects::Uuid(lane.id.uuid),
                            convert::ToMapObjectLaneMarkingContainer(lane.lane_markings),
                            convert::ToMapObjectLaneDirection(lane.dir),
                            convert::ToMapObjectsUuids(lane.incoming_connections),
                            convert::ToMapObjectsUuids(lane.outgoing_connections),
                            convert::ToMapObjectLaneType(lane.type),
                            convert::ToMapObjectPoints(lane.points),
                            lane.width,
                            mapobjects::Point3D(lane.handle_point.x, lane.handle_point.y, lane.handle_point.z),
                            lane.visibility,
                            lane.height,
                            lane.junction_id);
}

/* converts a map_thrift::LaneGroup to maobjects::LaneGroup */
mapobjects::LaneGroup ToMapObjectsLaneGroup(const map_thrift::LaneGroup& lg) {
    return mapobjects::LaneGroup(mapobjects::Uuid(lg.id.uuid),
                                 convert::ToMapObjectsUuids(lg.lanes_left),
                                 convert::ToMapObjectsUuids(lg.lanes_right));
}

/* converts a map_thrift::Stopline to maobjects::Stopline* */
mapobjects::Stopline ToMapObjectsStopLine(const map_thrift::Stopline& stop_line) {
	return mapobjects::Stopline(mapobjects::Uuid(stop_line.lane_object.id.uuid),
								mapobjects::Uuid(stop_line.lane_object.lg_id.uuid),
								convert::ToMapObjectsUuids(stop_line.lane_object.l_ids),
								stop_line.lane_object.offset);
}

/* converts a map_thrift::Crosswalk to maobjects::Crosswalk* */
mapobjects::Crosswalk ToMapObjectsCrossWalk(const map_thrift::Crosswalk& crosswalk) {
	return mapobjects::Crosswalk(mapobjects::Uuid(crosswalk.lane_object.id.uuid),
            					 mapobjects::Uuid(crosswalk.lane_object.lg_id.uuid),
            					 convert::ToMapObjectsUuids(crosswalk.lane_object.l_ids),
            					 crosswalk.lane_object.offset);
}

/* converts a map_thrift::Parking to maobjects::Parking* */
mapobjects::Parking ToMapObjectsParking(const map_thrift::Parking& parking_lot) {
	std::vector<std::vector<mapobjects::Point3D>> outline_list;
	for(auto& outline : parking_lot.outer_points){
		outline_list.emplace_back(ToMapObjectPoints(outline));
	}
	return mapobjects::Parking(mapobjects::Uuid(parking_lot.lane_object.id.uuid),
							   mapobjects::Uuid(parking_lot.lane_object.lg_id.uuid),
							   convert::ToMapObjectsUuids(parking_lot.lane_object.l_ids),
							   parking_lot.lane_object.offset,
							   parking_lot.number_of_lots,
							   parking_lot.parking_height,
							   parking_lot.parking_width,
							   parking_lot.line_width,
							   outline_list);
}

/* converts a map_thrift::Roadsign to maobjects::Roadsign* */
mapobjects::Roadsign ToMapObjectsRoadsign(const map_thrift::Roadsign& roadsign) {
	return mapobjects::Roadsign(mapobjects::Uuid(roadsign.lane_object.id.uuid),
								mapobjects::Uuid(roadsign.lane_object.lg_id.uuid),
								convert::ToMapObjectsUuids(roadsign.lane_object.l_ids),
								0,
								roadsign.sign_type,
								mapobjects::Point3D(roadsign.position.x, 
            					                	roadsign.position.y, 
            					                	roadsign.position.z),
                                roadsign.rotation);
}

map_thrift::LaneMarking ToThriftLaneMarking(const mapobjects::LaneMarking& mo) {
    map_thrift::LaneMarking to;
    to.type = convert::ToThriftLaneMarkingType(mo.GetLaneMarkingType());
    to.points = convert::ToThriftPoints(mo.GetPoints());
    to.id = convert::ToThriftUuid(mo.GetUuid());
    to.visibility = mo.GetVisibility();
    return to;
}
// Lanes
map_thrift::Lane ToThriftLane(const mapobjects::Lane& lane) {
    map_thrift::Lane to;
    to.lane_markings = convert::ToThriftLaneMarkingContainer(lane.GetLaneMarkingContainer());
    to.dir = convert::ToThriftLaneDirection(lane.GetLaneDirection());
    to.outgoing_connections = convert::ToThriftUuids(lane.GetOutConnections());
    to.incoming_connections = convert::ToThriftUuids(lane.GetInConnections());
    to.type = convert::ToThriftLaneType(lane.GetLaneType());
    to.points = convert::ToThriftPoints(lane.GetPoints());
    to.width = lane.GetWidth();
    to.id = convert::ToThriftUuid(lane.GetUuid());
    to.handle_point = convert::ToThriftPoint(lane.GetHandlePoint());
    to.visibility = lane.GetVisibility();
    to.height = lane.GetHeight();
    to.junction_id = lane.GetJunctionID();
    return to;
}
map_thrift::LaneGroup ToThriftLaneGroup(const mapobjects::LaneGroup& lg) {
    map_thrift::LaneGroup to;
    to.id = convert::ToThriftUuid(lg.GetUuid());
    to.lanes_left = convert::ToThriftUuids(lg.GetLeftLanes());
    to.lanes_right = convert::ToThriftUuids(lg.GetRightLanes());
    return to;
}
// batch
std::vector<map_thrift::LaneMarking> ToThriftLaneMarkingBatch(const std::vector<mapobjects::LaneMarking>& lm_list) {
    int n = lm_list.size();
    std::vector<map_thrift::LaneMarking> to_list(n);
    for (int i = 0; i < n; ++i) {
        to_list[i] = convert::ToThriftLaneMarking(lm_list[i]);
    }
    return to_list;
}
std::vector<map_thrift::Lane> ToThriftLaneBatch(const std::vector<mapobjects::Lane>& l_list) {
    int n = l_list.size();
    std::vector<map_thrift::Lane> to_list(n);
    for (int i = 0; i < n; ++i) {
        to_list[i] = convert::ToThriftLane(l_list[i]);
    }
    return to_list;
}
std::vector<map_thrift::LaneGroup> ToThriftLaneGroupBatch(const std::vector<mapobjects::LaneGroup>& lg_list) {
    int n = lg_list.size();
    std::vector<map_thrift::LaneGroup> to_list(n);
    for (int i = 0; i < n; ++i) {
        to_list[i] = convert::ToThriftLaneGroup(lg_list[i]);
    }
    return to_list;
}
map_thrift::Pivot ToThriftPivot(const mapobjects::Pivot& map_pivot) {
    // look at this function before this commit .......
    map_thrift::Pivot t_pivot;
    t_pivot.id = convert::ToThriftUuid(map_pivot.GetUuid());
    t_pivot.pose.position = convert::ToThriftPoint(mapobjects::Point3D(map_pivot.GetPose().x(),
        										    				   map_pivot.GetPose().y(),
        										    				   map_pivot.GetPose().z()));
    t_pivot.pose.orientation = map_pivot.GetPose().t();
    t_pivot.is_valid = true;
    return t_pivot;
}
std::vector<map_thrift::Stopline> ToThriftStoplineBatch(const std::vector<mapobjects::Stopline> &sl_list) {
    std::vector<map_thrift::Stopline> thrift_sl_list;
    thrift_sl_list.reserve(sl_list.size());
    map_thrift::Stopline thrift_sl;
    for (auto &sl : sl_list) {
        thrift_sl.lane_object.id.uuid = sl.GetUuid().GetUuidValue();
        thrift_sl.lane_object.lg_id.uuid = sl.GetLaneGroupId().GetUuidValue();
        thrift_sl.lane_object.l_ids = convert::ToThriftUuids(sl.GetLaneIds());
        thrift_sl.lane_object.offset = sl.GetOffset();
        thrift_sl_list.emplace_back(thrift_sl);
    }
    return thrift_sl_list;
}
std::vector<map_thrift::Roadsign> ToThriftRoadsignBatch(const std::vector<mapobjects::Roadsign> &rs_list) {
    std::vector<map_thrift::Roadsign> thrift_rs_list;
    thrift_rs_list.reserve(rs_list.size());
    map_thrift::Roadsign thrift_rs;
    for (auto &rs : rs_list) {
        thrift_rs.lane_object.id.uuid = rs.GetUuid().GetUuidValue();
        thrift_rs.lane_object.lg_id.uuid = rs.GetLaneGroupId().GetUuidValue();
        thrift_rs.lane_object.l_ids = convert::ToThriftUuids(rs.GetLaneIds());
        thrift_rs.lane_object.offset = 0;
        thrift_rs.sign_type = rs.GetSignType();
        auto rs_pos = rs.GetPosition();
        thrift_rs.position.x = rs_pos.x();
        thrift_rs.position.y = rs_pos.y();
        thrift_rs.position.z = rs_pos.z();
        thrift_rs.rotation = rs.GetRotation();
        thrift_rs_list.emplace_back(thrift_rs);
    }
    return thrift_rs_list;
}
std::vector<map_thrift::Crosswalk> ToThriftCrosswalkBatch(const std::vector<mapobjects::Crosswalk> &cw_list) {
    std::vector<map_thrift::Crosswalk> thrift_cw_list;
    thrift_cw_list.reserve(cw_list.size());
    map_thrift::Crosswalk thrift_cw;
    for (auto &cw : cw_list) {
        thrift_cw.lane_object.id.uuid = cw.GetUuid().GetUuidValue();
        thrift_cw.lane_object.lg_id.uuid = cw.GetLaneGroupId().GetUuidValue();
        thrift_cw.lane_object.l_ids = convert::ToThriftUuids(cw.GetLaneIds());
        thrift_cw.lane_object.offset = cw.GetOffset();
        thrift_cw_list.emplace_back(thrift_cw);
    }
    return thrift_cw_list;
}
std::vector<map_thrift::Parking> ToThriftParkingBatch(const std::vector<mapobjects::Parking> &pk_list) {
    std::vector<map_thrift::Parking> thrift_pk_list;
    thrift_pk_list.reserve(pk_list.size());
    map_thrift::Parking thrift_pk;
    for (auto &parking : pk_list) {
        for (auto outline : parking.GetOuterPoints()) {
            thrift_pk.outer_points.emplace_back(convert::ToThriftPoints(outline));
        }
        thrift_pk.lane_object.id.uuid = parking.GetUuid().GetUuidValue();
        thrift_pk.lane_object.lg_id.uuid = parking.GetLaneGroupId().GetUuidValue();
        thrift_pk.lane_object.l_ids = convert::ToThriftUuids(parking.GetLaneIds());
        thrift_pk.lane_object.offset = parking.GetOffset();
        thrift_pk.number_of_lots = parking.GetNumberOfParkingLots();
        thrift_pk.parking_height = parking.GetHeight();
        thrift_pk.parking_width = parking.GetWidth();
        thrift_pk.line_width = parking.GetLineWidth();
        thrift_pk_list.emplace_back(thrift_pk);
    }
    return thrift_pk_list;
}

} // namespace convert

namespace create
{
/* function to create map_thrift::MapPart from Thrift map objects */
map_thrift::MapPart MapPartFromMapObjects(const std::vector<map_thrift::LaneMarking>& v_lm,
                                            const std::vector<map_thrift::Lane>& v_l,
                                            const std::vector<map_thrift::LaneGroup>& v_lg,
                                            const map_thrift::LaneObjectList& v_lo) {
    // here the lane objects could be added to lane_object_list
    // std::cout << "Map: currently no lane_objects are added to the MapMessage" << std::endl;
    map_thrift::MapPart m_part;
    m_part.lanes = v_l;
    m_part.lane_groups = v_lg;
    m_part.lane_markings = v_lm;
    m_part.lane_objects = v_lo;
    return m_part;
}
map_thrift::MapContainer MapContainerFromMapParts(const std::vector<map_thrift::MapPart>& m_parts, 
												  const mapobjects::Pivot& map_pivot) {
    map_thrift::MapContainer m_container;
    m_container.map_parts = m_parts;
    m_container.pivot = convert::ToThriftPivot(map_pivot);
    return m_container;
}
/* function to create map_thrift::MapContainer from map_thrift::MapPart */
map_thrift::MapContainer MapContainerFromMapParts(const map_thrift::MapPart& m_part, 
												  const mapobjects::Pivot& map_pivot) {
    map_thrift::MapContainer m_container;
    m_container.map_parts = std::vector<map_thrift::MapPart>{m_part};
    m_container.pivot = convert::ToThriftPivot(map_pivot);
    return m_container;
}
/* function to create map_thrift::MapMessage ADD */
map_thrift::MapMessage MapAddMessage(const map_thrift::MapContainer& m_container) {
    map_thrift::MapMessage mm;
    mm.op = map_thrift::MessageOp::type::ADD;
    mm.container = m_container;
    return mm;
}
/* function to create map_thrift::MapMessage UPDATE_WHOLE */
map_thrift::MapMessage MapUpdateWholeMessage(const map_thrift::MapContainer& m_container) {
    map_thrift::MapMessage mm;
    mm.op = map_thrift::MessageOp::type::UPDATE_WHOLE;
    mm.container = m_container;
    return mm;
}
/* function to create map_thrift::MapMessage UPDATE_PART */
map_thrift::MapMessage MapUpdatePartMessage(const map_thrift::MapContainer& m_container) {
    map_thrift::MapMessage mm;
    mm.op = map_thrift::MessageOp::type::UPDATE_PART;
    mm.container = m_container;
    return mm;
}
/* function to create map_thrift::MapMessage DELETE */
map_thrift::MapMessage MapDeleteMessage(const map_thrift::MapContainer& m_container) {
    map_thrift::MapMessage mm;
    mm.op = map_thrift::MessageOp::type::DELETE;
    mm.container = m_container;
    return mm;
}

} // namespace create
} // namespace freicar

