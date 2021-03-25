#include "map_core/freicar_type_conversion.h"
#include "map_core/freicar_map_config.h"
#include "map_core/freicar_map.h"
#include "map_core/freicar_map_helper.h"

#include <functional>
#include <algorithm>
#include <numeric>
#include <cstdio>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
namespace freicar
{
namespace map
{

/* constructor for the map class */
Map::Map() {
    status_ = MapStatus::UNINITIALIZED;
    IntializeMapContainers();
    pivot_point_ = mapobjects::Pivot();
}

/* returns the singleton instance */
Map& Map::GetInstance(){
    static Map instance;
    return instance;
}

/* returns the map status */
MapStatus Map::status() const {
    return status_;
}

/* returns the pivot point*/
mapobjects::Pivot Map::pivot() const {
    return pivot_point_;
}
/* initializes all the containers used for the map */
void Map::IntializeMapContainers(){
	lane_storage_ = std::vector<mapobjects::Lane>();
    lanegroup_storage_ = std::vector<mapobjects::LaneGroup>();
    lanemarking_storage_ = std::vector<mapobjects::LaneMarking>();
    lanemarking_uuids_ = std::unordered_map<std::string, int>();
    lane_uuids_ = std::unordered_map<std::string, int>();
    lanegroup_uuids_ = std::unordered_map<std::string, int>();
    lane_stoplines_lookup_ = std::unordered_map<std::string, std::vector<mapobjects::Uuid>>();
    lane_crosswalks_lookup_ = std::unordered_map<std::string, std::vector<mapobjects::Uuid>>();
    lane_parking_lookup_ = std::unordered_map<std::string, std::vector<mapobjects::Uuid>>();
    lane_roadsign_lookup_ = std::unordered_map<std::string, std::vector<mapobjects::Uuid>>();
}
/* empties all the map containers */
bool Map::ClearMap() {
    // enters CRITICAL section
    std::lock_guard<std::mutex> guard(access_mutex_);
	// clear struct storage
    lanemarking_storage_.clear();
    lane_storage_.clear();
    lanegroup_storage_.clear();
    // clear the uuid storage
    lane_uuids_.clear();
    lanegroup_uuids_.clear();
    lanemarking_uuids_.clear();
    laneobject_uuids_.clear();
    // clear the lookup storages
    lane_stoplines_lookup_.clear();
    lane_crosswalks_lookup_.clear();
    lane_parking_lookup_.clear();
    lane_roadsign_lookup_.clear();
    map_changed_ = true;
    pivot_point_ =  mapobjects::Pivot();
	status_ = MapStatus::UNINITIALIZED;
	return true;
}
/* returns sizes of map storages */
std::tuple<int, int, int> Map::GetMapStats() const {
    return std::make_tuple(lane_storage_.size(),
						   lanegroup_storage_.size(),
						   lanemarking_storage_.size());
}

// Thrift Functions
// ------------------------------------------------------

/* returns the entire map as a thrift message */
map_thrift::MapMessage Map::AsThriftMessagae() const {
    // enters CRITICAL section
    std::lock_guard<std::mutex> guard(access_mutex_);
	map_thrift::LaneObjectList laneobject_list;
	laneobject_list.crosswalks = convert::ToThriftCrosswalkBatch(crosswalk_storage_);
	laneobject_list.parking_lots = convert::ToThriftParkingBatch(parking_storage_);
	laneobject_list.roadsigns = convert::ToThriftRoadsignBatch(roadsign_storage_);
	laneobject_list.stoplines = convert::ToThriftStoplineBatch(stopline_storage_);
    map_thrift::MapPart m_part = create::MapPartFromMapObjects(
        						 convert::ToThriftLaneMarkingBatch(lanemarking_storage_),
        						 convert::ToThriftLaneBatch(lane_storage_),
        						 convert::ToThriftLaneGroupBatch(lanegroup_storage_),
								 laneobject_list);
    map_thrift::MapContainer m_container;
    m_container.map_parts = std::vector<map_thrift::MapPart>{m_part};
    m_container.pivot = convert::ToThriftPivot(pivot_point_);
	map_thrift::MapMessage mm = map_thrift::MapMessage();
    mm.op = map_thrift::MessageOp::type::ADD;
    mm.container = m_container;
	return mm;
}

/* function to process incoming map messages on the car */
void Map::ProcessMapMessage(const map_thrift::MapMessage& message, MapStatus status) {
    map_thrift::MessageOp::type op = message.op;
    switch (op) {
        case map_thrift::MessageOp::type::ADD:
            AppendMessageToMap(message);
			status_ = status;
            break;
        case map_thrift::MessageOp::type::UPDATE_WHOLE:
            AppendMessageToMap(message);
			status_ = status;
            break;
        case map_thrift::MessageOp::type::UPDATE_PART:
            AppendMessageToMap(message);
			status_ = status;
			break;
		case map_thrift::MessageOp::type::DELETE:
            ClearMap();
            break;
        case map_thrift::MessageOp::type::POSE_UPDATE:
            std::cout << "map: thrift MapMessage POSE_UPDATE received. invalid use." << std::endl;
            break;
        case map_thrift::MessageOp::type::POSE_DELETE:
            std::cout << "map: thrift MapMessage POSE_DELETE received. invalid use." << std::endl;
            break;
    }
}

// MapAdd Functions
// --------------------------------------------------------------------

/*  adds all the objects contained in a map message */
void Map::AppendMessageToMap(const map_thrift::MapMessage& message) {
    // loop over all MapParts in the message
    for (auto& map_part : message.container.map_parts) {
        if (map_part.lanes.size() != 0) {
            for(auto &lane : map_part.lanes){
				AddLane(convert::ToMapObjectsLane(lane));
			}
        }
        if (map_part.lane_groups.size() != 0) {
            for(auto &lane_group : map_part.lane_groups){
                AddLaneGroup(convert::ToMapObjectsLaneGroup(lane_group));
            }
        }
        if (map_part.lane_markings.size() != 0) {
            for(auto &lane_marking : map_part.lane_markings){
                AddLaneMarking(convert::ToMapObjectsLanemarking(lane_marking));
            }
        }
        AddThriftLaneObjectList(map_part.lane_objects);
    }
    // adding the pivot
    SetPivotFromThrift(message.container.pivot);

    map_changed_ = true;
    status_ =  MapStatus::UPDATED_FROM_REMOTE;
}

/* converts map_thrift::Pivot to mapobjects::Pivot */
void Map::SetPivotFromThrift(const map_thrift::Pivot& thrift_pivot){
    if (thrift_pivot.is_valid) {
		mapobjects::Pivot pivot = convert::ToMapObjectsPivot(thrift_pivot);
		pivot_point_ = pivot;
    } else {
        std::cout << "map: detected invalid pivot. ignoring" << std::endl;
    }
}

/* adds a mapobjects::LaneMarking to the map, or updates an existing instance */
void Map::AddLaneMarking(const mapobjects::LaneMarking& lanemarking) {
    // enters CRITICAL section
    std::lock_guard<std::mutex> guard(access_mutex_);

    int index = FindDuplicate(lanemarking);
    if (index == -1) {
        try {
            lanemarking_storage_.push_back(lanemarking);
            lanemarking_uuids_.insert(
            {(lanemarking.GetUuid()).GetUuidValue(), (static_cast<int>(lanemarking_storage_.size()) - 1)});
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!" << std::endl;
        }
    } else if (index > -1) {
        try {
            lanemarking_storage_[index] = lanemarking;
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!" << std::endl;
        }
    }
	map_changed_ = true;
}

/* adds a mapobjects::Lane to the map, or updates an existing instance */
void Map::AddLane(const mapobjects::Lane& lane) {
    // enters CRITICAL section
    std::lock_guard<std::mutex> guard(access_mutex_);
    int index = FindDuplicate(lane);
    if (index == -1) {
        try {
            lane_storage_.push_back(lane);
            lane_uuids_.insert({(lane.GetUuid()).GetUuidValue(), (static_cast<int>(lane_storage_.size()) - 1)});
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!" << std::endl;
        }
    } else if (index > -1) {
        try {
            lane_storage_[index] = lane;
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!" << std::endl;
        }
    }
	map_changed_ = true;
}
/* adds a mapobjects::LaneGroup to the map, or updates an existing instance */
void Map::AddLaneGroup(const mapobjects::LaneGroup& lane_group) {
    // enters CRITICAL section
    std::lock_guard<std::mutex> guard(access_mutex_);

    int index = FindDuplicate(lane_group);
    if (index == -1) {
        try {
            lanegroup_storage_.push_back(lane_group);
			// map uuid to index
            lanegroup_uuids_.insert({(lane_group.GetUuid()).GetUuidValue(), (static_cast<int>(lanegroup_storage_.size()) - 1)});
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!" << std::endl;
        }
    } else if (index > -1) {
        try {
            lanegroup_storage_[index] = lane_group;
        } catch (const std::invalid_argument& ia) {
            std::cout << "[ERROR] mapobject is of invalid type!" << std::endl;
        }
    }
	map_changed_ = true;
}
/* converts and adds all the lane objects in a map_thrift::LaneObjectList to the map*/
void Map::AddThriftLaneObjectList(const map_thrift::LaneObjectList& lo_list) {
	std::lock_guard<std::mutex> guard(access_mutex_);
	for (auto& stopline : lo_list.stoplines) {
		stopline_storage_.emplace_back(convert::ToMapObjectsStopLine(stopline));
		map_changed_ = true;
	}
	for (auto& crosswalk : lo_list.crosswalks) {
		crosswalk_storage_.emplace_back(convert::ToMapObjectsCrossWalk(crosswalk));
		map_changed_ = true;
	}
	for (auto& parking : lo_list.parking_lots) {
		parking_storage_.emplace_back(convert::ToMapObjectsParking(parking));
		map_changed_ = true;
	}
	for (auto& roadsign : lo_list.roadsigns) {
		roadsign_storage_.emplace_back(convert::ToMapObjectsRoadsign(roadsign));
		map_changed_ = true;
	}
}

// Lookup Functions
// --------------------------------------------------------------------

std::vector<mapobjects::Lane> Map::getLanes(){
    return lane_storage_;
}
std::vector<mapobjects::Roadsign> Map::getSigns(){
    return roadsign_storage_;
}

/* returns a lane by reference based on its uuid */
mapobjects::Lane* Map::FindLaneByUuid(const mapobjects::Uuid& lane_uuid) {
	auto it = lane_uuids_.find(lane_uuid.GetUuidValue());
	if (it == lane_uuids_.end()) {
		return nullptr;
	}
    return &lane_storage_.at(it->second);
}
/* returns a lane by reference based on its uuid */
mapobjects::Lane* Map::FindLaneByUuid(const std::string& lane_uuid) {
	auto it = lane_uuids_.find(lane_uuid);
	if (it != lane_uuids_.end()) {
		return &lane_storage_.at(it->second);
	}
    return nullptr;
}
/* returns the lanegroup by reference based on the Uuid */
mapobjects::LaneGroup* Map::FindLaneGroupByUuid(const mapobjects::Uuid &lanegroup_uuid) {
	// return lanegroup_storage_.at(lanegroup_uuids_.at(lanegroup_uuid.GetUuidValue()));
	auto lookup = lanegroup_uuids_.find(lanegroup_uuid.GetUuidValue());
	if (lookup != lanegroup_uuids_.end()) {
		return &lanegroup_storage_.at(lookup->second);
	}
	return nullptr;
}
/*  check for duplicates in map. returns -1 if the not found, < -1 if found multiple times, index otherwise*/
int Map::FindDuplicate(const mapobjects::MapObject& mo) {
    std::unordered_map<std::string, int>::iterator it;
    std::string uuid = (mo.GetUuid()).GetUuidValue();
    // search all the uuid storages
    int index = -1;
    // lanes
    it = lane_uuids_.find(uuid);
    if (it != lane_uuids_.end()) {
        if (index == -1) {
            index = it->second;
        } else {
            index = -2;
        }
    }
    // lanemarkings
    it = lanemarking_uuids_.find(uuid);
    if (it != lanemarking_uuids_.end()) {
        if (index == -1) {
            index = it->second;
        } else {
            index = -2;
        }
    }
    // lanegroups
    it = lanegroup_uuids_.find(uuid);
    if (it != lanegroup_uuids_.end()) {
        if (index == -1) {
            index = it->second;
        } else {
            index = -2;
        }
    }
    // laneobjects
    it = laneobject_uuids_.find(uuid);
    if (it != laneobject_uuids_.end()) {
        if (index == -1) {
            index = it->second;
        } else {
            index = -2;
        }
    }
    return index;
}
/* adds a stopline to lookup storage */
void Map::AddToStoplinesLookup(mapobjects::Stopline stopline) {
    std::vector<mapobjects::Uuid> stopline_lanes = stopline.GetLaneIds();
    int n_lanes_to_process = stopline_lanes.size();
    for (int i = 0; i < n_lanes_to_process; ++i) {
        auto it = lane_stoplines_lookup_.find(stopline_lanes[i].GetUuidValue());
        // if lane not in lookup
        if (it == lane_stoplines_lookup_.end()) {
            lane_stoplines_lookup_.insert({stopline_lanes[i].GetUuidValue(),
										   std::vector<mapobjects::Uuid>(1, stopline.GetUuid())});
		} else {
            it->second.push_back(stopline.GetUuid());
		}
    }
}
/* adds a crosswalk to lookup storage */
void Map::AddToCrosswalksLookup(mapobjects::Crosswalk crosswalk) {
    // bypass lane_ids at it is not assigned in the Streetdesigner
    // std::vector<mapobjects::Uuid> crosswalk_lanes = _mo->getLaneIds();
	// getting left and right lanes from this lanegroup
	std::vector<mapobjects::Uuid> crosswalk_lanes;
	auto* lanegroup = FindLaneGroupByUuid(crosswalk.GetLaneGroupId());
	auto left_lanes = lanegroup->GetLeftLanes();
	auto right_lanes = lanegroup->GetRightLanes();
	crosswalk_lanes.insert(crosswalk_lanes.end(), left_lanes.begin(), left_lanes.end());
	crosswalk_lanes.insert(crosswalk_lanes.end(), right_lanes.begin(), right_lanes.end());

	int n_lanes_to_process = crosswalk_lanes.size();
	for (int i = 0; i < n_lanes_to_process; ++i) {
        auto it = lane_crosswalks_lookup_.find(crosswalk_lanes[i].GetUuidValue());
        // if lane not in lookup
        if (it == lane_stoplines_lookup_.end())
            lane_crosswalks_lookup_.insert({crosswalk_lanes[i].GetUuidValue(), 
											std::vector<mapobjects::Uuid>(1, crosswalk.GetUuid())});
        else
            it->second.push_back(crosswalk.GetUuid());
    }
}
/* adds a parking lot to its lookup storage */
void Map::AddToParkingLookup(mapobjects::Parking parking) {
    std::vector<mapobjects::Uuid> parking_lanes = parking.GetLaneIds();
    int n_lanes_to_process = parking_lanes.size();
    for (int i = 0; i < n_lanes_to_process; ++i) {
        auto it = lane_parking_lookup_.find(parking_lanes[i].GetUuidValue());
        // if lane not in lookup
        if (it == lane_parking_lookup_.end())
            lane_parking_lookup_.insert({parking_lanes[i].GetUuidValue(), 
										 std::vector<mapobjects::Uuid>(1, parking.GetUuid())});
        else
            it->second.push_back(parking.GetUuid());
    }
}
/* adds a roadsign to lookup storage */
void Map::AddToRoadsignLookup(mapobjects::Roadsign roadsign) {
    std::vector<mapobjects::Uuid> stopline_lanes = roadsign.GetLaneIds();
    int n_lanes_to_process = stopline_lanes.size();
    for (int i = 0; i < n_lanes_to_process; ++i) {
        auto it = lane_stoplines_lookup_.find(stopline_lanes[i].GetUuidValue());
        // if lane not in lookup
        if (it == lane_stoplines_lookup_.end())
            lane_stoplines_lookup_.insert({stopline_lanes[i].GetUuidValue(), 
										   std::vector<mapobjects::Uuid>(1, roadsign.GetUuid())});
        else
            it->second.push_back(roadsign.GetUuid());
    }
}
/* returns a vector of outgoing lanes that go to other lanegroups. To be used after postprocessing */
std::vector<mapobjects::Uuid> Map::GetPureOutgoingConnections(const mapobjects::Lane &lane) {
	auto* lane_group = FindLaneGroupByUuid(lane.GetParentUuid());
	std::vector<mapobjects::Uuid> pure_out;
	for (auto& out_connection : lane.GetOutConnections()) {
		auto out_lane = FindLaneByUuid(out_connection);
		if (out_lane->GetParentUuid() != lane_group->GetUuid())
			pure_out.emplace_back(out_lane->GetUuid());
	}
	return pure_out;
}
/* returns a vector of incoming lanes that come from other lanegroups. To be used after postprocessing */
std::vector<mapobjects::Uuid> Map::GetPureIncomingConnections(const mapobjects::Lane &lane) {
	auto* lane_group = FindLaneGroupByUuid(lane.GetParentUuid());
	std::vector<mapobjects::Uuid> pure_in;
	for (auto& in_connection : lane.GetInConnections()) {
		auto in_lane = FindLaneByUuid(in_connection);
		if (in_lane->GetParentUuid() != lane_group->GetUuid())
			pure_in.emplace_back(in_lane->GetUuid());
	}
	return pure_in;
}
/* returns the closest lane point and the squared distance to a given position using kd-tree search */
std::vector<std::pair<mapobjects::LanePoint3D, float>>
Map::FindClosestLanePoints(float local_x, float local_y, float local_z, unsigned int count) const {
	float kd_point[3] = {local_x ,  local_y , local_z };
	std::vector<size_t> ret_index(count);
	std::vector<float> out_dist_sqr(count);
	count = kd_tree_->knnSearch(&kd_point[0], count, &ret_index[0], &out_dist_sqr[0]);
	std::vector<std::pair<mapobjects::LanePoint3D, float>> ret_nodes;
	// In case of less points in the tree than requested:
	ret_index.resize(count);
	out_dist_sqr.resize(count);
	for (size_t i = 0; i < count; ++i)
		ret_nodes.emplace_back(kd_points_[ret_index[i]], out_dist_sqr[i]);
	return ret_nodes;
}

/* returns the closest lane point and the squared distance to a given position using kd-tree search, sorted by heading (-PI, PI)*/
std::vector<std::pair<mapobjects::LanePoint3D, float>>
Map::FindClosestLanePointsWithHeading(float local_x, float local_y, float local_z, unsigned int count, float heading) const {
	float kd_point[3] = {local_x ,  local_y , local_z };
	std::vector<size_t> ret_index(count);
	std::vector<float> out_dist_sqr(count);
	count = kd_tree_->knnSearch(&kd_point[0], count, &ret_index[0], &out_dist_sqr[0]);
	std::vector<std::pair<mapobjects::LanePoint3D, float>> ret_nodes;
	ret_index.resize(count);
	out_dist_sqr.resize(count);
	// normalizing heading just in case
	if (heading > M_PI)
		heading -= 2 * M_PI;
	else if (heading < -M_PI)
		heading += 2 * M_PI;
	// no sorting by heading if just asking for 1
	if (count == 1) {
		ret_nodes.emplace_back(kd_points_[ret_index[0]], out_dist_sqr[0]);
		return ret_nodes;
	}
	std::vector<unsigned int> permutation(count);
	std::iota(permutation.begin(), permutation.end(), 0);
	// sort by heading
	std::sort(permutation.begin(), permutation.end(), [ret_index, heading, this](unsigned int a, unsigned int b) {
		return std::abs(kd_points_[ret_index[a]].heading_ - heading) < std::abs(kd_points_[ret_index[b]].heading_ - heading);
	});
	for (size_t i = 0; i < count; ++i)
		ret_nodes.emplace_back(kd_points_[ret_index[permutation[i]]], out_dist_sqr[permutation[i]]);
	return ret_nodes;
}
/* returns a random node from the map */
mapobjects::LanePoint3D Map::GetRandomLanePoint() const {
	return kd_points_.at(rand() % kd_points_.size());
}
/* returns a random lane from the map */
mapobjects::Lane& Map::GetRandomLane() {
	// -1 so the off-road lane doesn't get selected
	return lane_storage_.at(rand() % (lane_storage_.size() - 1));
}
/* return random lane */
std::vector<mapobjects::Junction> Map::GetJunctions() {
	return junction_storage_;
}
mapobjects::Junction Map::GetJunctionByID(int junction_id) {
	if (junction_id < 0 || junction_id >= static_cast<int>(junction_storage_.size())) {
		// invalid junction
		return mapobjects::Junction(-1);
	}
	return junction_storage_[junction_id];
}
// Post Processing ------------------------------------------------------------------------
/* sets lane parent uuids, detects junctions and adjusts the coordinate system */
void Map::PostProcess(float density_cm) {
	AdjustCoordinates();
	SetLaneParents();
	InitializeJunctions();
	UpdateNodes(density_cm);
	FillLaneHashMaps();
	CreateKDIndex();
	AddLaneObjectPointers();
	// adding a dummy lane to indicate offlane
	using namespace mapobjects;
	Uuid id("?");
	std::vector<Uuid> empty;
	std::vector<Point3D> empty_p;
	LaneMarkingContainer container(empty, empty);
	Lane dummy(id, container, LaneDirection::LEFT, empty, empty,
			   LaneType::PEDESTRIAN_LANE, empty_p, 0, Point3D(), false,
			   0, -1);
	AddLane(dummy);
}
/* adds pointers of roadsigns & stoplines to their respective lanes */
void Map::AddLaneObjectPointers() {
	for (auto& roadsign : roadsign_storage_) {
		for (auto &lane_uuid : roadsign.GetLaneIds()) {
			FindLaneByUuid(lane_uuid)->AddRoadSign(&roadsign);
		}
	}
	for (auto& stopline : stopline_storage_) {
		for (auto &lane_uuid : stopline.GetLaneIds()) {
			FindLaneByUuid(lane_uuid)->SetStopLine(&stopline);
		}
	}
}
/* sets the lanegroup uuid of each lane, to be used after the map is loaded */
void Map::SetLaneParents() {
	for (auto &lane_group : lanegroup_storage_) {
		// for all right lanes
		for (auto& right_lane_uuid : lane_group.GetRightLanes()) {
			FindLaneByUuid(right_lane_uuid)->SetParentUuid(lane_group.GetUuid());
		}
		// for all left lanes
		for (auto& left_lane_uuid : lane_group.GetLeftLanes()) {
			FindLaneByUuid(left_lane_uuid)->SetParentUuid(lane_group.GetUuid());
		}
	}	
}
/* mark lanegroups as junctions if any of their lanes are a junction lane. before ffd01c6e
   there was no indication of junctions in the map structure. lanegroups being marked as junctions
   is still required in GetNeighboringLanes().
*/
void Map::InitializeJunctions() {
	for (auto &lane : lane_storage_) {
		FindLaneGroupByUuid(lane.GetParentUuid())->SetAsJunction(lane.IsJunctionLane());
		if (lane.IsJunctionLane()) {
			int junc_idx = lane.GetJunctionID();
			// instantiate if the junction object doesn't exist yet
			for (int i = junction_storage_.size(); i - 1 < junc_idx; ++i)
				junction_storage_.emplace_back(i);
			junction_storage_[junc_idx].AddLane(lane);
			lane_junction_lookup_[lane.GetUuid().GetUuidValue()] = junc_idx;
		}
	}
	for (unsigned int i = 0; i < junction_storage_.size(); ++i) {
		junction_storage_[i].PostProcess();
		for (auto& incoming_lane_uuid : junction_storage_[i].GetIncomingLanes()) {
			incoming_lane_junction_lookup_[incoming_lane_uuid.GetUuidValue()] = junction_storage_[i].GetID();
		}
		for (auto& outgoing_lane_uuid : junction_storage_[i].GetOutgoingLanes()) {
			outgoing_lane_junction_lookup_[outgoing_lane_uuid.GetUuidValue()] = junction_storage_[i].GetID();
		}
	}
	// std::cout << "map-post: created " << junction_storage_.size() << " junctions" << std::endl;
}
/* deducts pivot coordinates from all lane points and flips y. converts (cm) to (m) */
void Map::AdjustCoordinates() {
	for(auto& lane : lane_storage_) {
		for(auto &point : lane.GetPointsByReference()) {
			point.SetCoords(std::make_tuple(point.x() - pivot_point_.GetPose().x(),
										    point.y() - pivot_point_.GetPose().y(),
											point.z() - pivot_point_.GetPose().z()));
		}
		auto &hpoint = lane.GetHandlePointByReference();
		hpoint.SetCoords(std::make_tuple(hpoint.x() - pivot_point_.GetPose().x(),
										 hpoint.y() - pivot_point_.GetPose().y(),
										 hpoint.z() - pivot_point_.GetPose().z()));
	}
	for(auto& lane_marking : lanemarking_storage_) {
		for(auto& point : lane_marking.GetPointsByReference()) {
			point.SetCoords(std::make_tuple(point.x() - pivot_point_.GetPose().x(),
										    point.y() - pivot_point_.GetPose().y(),
											point.z() - pivot_point_.GetPose().z()));
		}
	}
	for (auto& roadsign : roadsign_storage_) {
		auto pos = roadsign.GetPosition();
		roadsign.SetPosition(pos.x() - pivot_point_.GetPose().x(),
							 pos.y() - pivot_point_.GetPose().y(),
							 pos.z() - pivot_point_.GetPose().z());
	}
	pivot_point_.SetPose(mapobjects::Pose(0, 0, 0, 0));
}
/* makes the lanepoints more dense, initializes their lane_uuid member */
void Map::UpdateNodes(float density_m) {
	// std::cout << "map-post: densifying ...\n";
	unsigned int dense_nodes = 0;
	// for all right lanes
	for (auto &lane : lane_storage_) {
		auto &points = lane.GetPointsByReference();
		// for all points in the lane
		for (unsigned int i = 0; i < points.size(); ++i) {
			// if graph should be dense && not the last point in lane
			if (density_m > 0 && i != points.size() - 1) {
				float dist = points[i].ComputeDistance(points[i+1]);
				if (dist > density_m) {
					dense_nodes++;
					auto first = points[i], second = points[i + 1];
					auto v_forward = (second - first).Normalized();
					mapobjects::Point3D interpolated = first + v_forward * density_m;
					points.insert(points.begin() + i + 1, interpolated);
				}
			}
		}
		// kd_points_.insert(kd_points_.end(), points.begin(), points.end());
	}
	// std::cout << "map-post: created " << dense_nodes << " new nodes" << std::endl;
}
/* detects all possible connections between lanes and filles the hash maps with the data */
void Map::FillLaneHashMaps() {
	for (auto &current_lane : lane_storage_) {
		// managing connections inside the current lane_group
		int adj_left_lane_idx, adj_right_lane_idx, opposite_lane_idx;
		std::tie(adj_left_lane_idx, adj_right_lane_idx, opposite_lane_idx) = 
			GetNeighboringLanes(current_lane, *FindLaneGroupByUuid(current_lane.GetParentUuid()));
		if (adj_left_lane_idx != -1) {
			current_lane.RegisterConnection(mapobjects::Lane::ADJACENT_LEFT,
											FindLaneByUuid(current_lane.GetOutConnections()[adj_left_lane_idx]));
		}
		if (adj_right_lane_idx != -1) {
			current_lane.RegisterConnection(mapobjects::Lane::ADJACENT_RIGHT,
											FindLaneByUuid(current_lane.GetOutConnections()[adj_right_lane_idx]));
		}
		if (opposite_lane_idx != -1) {
			current_lane.RegisterConnection(mapobjects::Lane::OPPOSITE,
											FindLaneByUuid(current_lane.GetOutConnections()[opposite_lane_idx]));
		}
		// managing connections between different lanegroups
		auto interconnections = GetPureOutgoingConnections(current_lane);
		// normal case with only one pure outgoing connections
		if (interconnections.size() == 1) {
			auto next_lane = FindLaneByUuid(interconnections[0]);
			current_lane.RegisterConnection(mapobjects::Lane::STRAIGHT, next_lane);
			// lanes right after junctions cannot have a BACK connection
			if (!current_lane.IsJunctionLane())
				next_lane->RegisterConnection(mapobjects::Lane::BACK, &current_lane);
		} else {
			// junctions
			auto &current_lanepoints = current_lane.GetPointsByReference();
			mapobjects::Point3D p_current = current_lanepoints.back();
			// at the time i wrote this it made sense but mathematically it doesn't
			// technically v_forward should be:
			// p_current - current_lanepoints[current_lanepoints.size() - 2];
			// but this value breaks every connection. TODO for next dev: rewrite?
			mapobjects::Point3D v_forward = p_current + (p_current - current_lanepoints[current_lanepoints.size() - 2]);
			for (auto &junction_uuid : interconnections) {
				auto junction_lane = FindLaneByUuid(junction_uuid);
				// only process lanes that are in junctions
				if (!junction_lane->IsJunctionLane())
					continue;

				auto dest_lane = FindLaneByUuid(junction_lane->GetOutConnections()[0]);
				// last point in current lane
				// a Point3D in forward direction
				// first & second Point3D from the destination_lane to determine its direction
				// w.r.t. current lane. getting by reference to improve performance. the math
				// checked out at the time but i don't know wtf is going on now.
				mapobjects::Point3D p_dest_base = dest_lane->GetPointsByReference()[0];
				mapobjects::Point3D p_dest = dest_lane->GetPointsByReference()[1];
				p_dest = p_dest + (p_current - p_dest_base);
				int relative_direction = p_current.GetRelativeDirection(v_forward, p_dest);
				// definitely straight
				if (relative_direction == 0) {
					current_lane.RegisterConnection(mapobjects::Lane::JUNCTION_STRAIGHT, junction_lane);
				} // left or straight
				else if (relative_direction == +1) {
					float angle_cos = helper::NormalizedDot(p_current, v_forward, p_dest);
					if (angle_cos >= config::kJungtionStraightMargin)
						current_lane.RegisterConnection(mapobjects::Lane::JUNCTION_STRAIGHT, junction_lane);
					else
						current_lane.RegisterConnection(mapobjects::Lane::JUNCTION_LEFT, junction_lane);
				} // right or straight
				else if (relative_direction == -1) {
					float angle_cos = helper::NormalizedDot(p_current, v_forward, p_dest);
					if (angle_cos >= config::kJungtionStraightMargin)
						current_lane.RegisterConnection(mapobjects::Lane::JUNCTION_STRAIGHT, junction_lane);
					else
						current_lane.RegisterConnection(mapobjects::Lane::JUNCTION_RIGHT, junction_lane);
				}
			}
		}
	}
}

/* initializes the kd-tree */
void Map::CreateKDIndex() {
	mapobjects::LanePoint3D lane_node;
	mapobjects::Point3D v_forward;
	// updating lane uuids and adding to kd_tree point vector
	for (auto &lane : lane_storage_) {
		float lane_offset = 0.0f;
		auto &points = lane.GetPointsByReference();
		for (size_t i = 0; i < points.size(); ++i) {
			if (i == 0) {
				v_forward = (points[1] - points[0]).Normalized();
			} else {
				v_forward = points[i] - points[i - 1];
				float distance = v_forward.GetNorm();
				lane_offset += distance;
				v_forward = v_forward / distance;
			}
			lane_node.heading_ = std::atan2(v_forward.y(), v_forward.x());
			lane_node.lane_offset_ = lane_offset;
			lane_node.lane_uuid_ = lane.GetUuid().GetUuidValue();
			lane_node.index_ = i;
			lane_node.SetCoords(points[i].GetCoords());
			kd_points_.emplace_back(lane_node);
		}
	}
	kd_tree_ = new nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, Map>,
			   										   Map, 3> (3, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	kd_tree_->buildIndex();
	// std::cout << "map-post: kd-tree index built" << std::endl;
}
/* test function for printing all the lanes */
void Map::PrintMap() {
	for (unsigned int i = 0; i < lane_storage_.size(); ++i) {
		lane_storage_[i].PrintLane();
	}
}
/* visualizes the map on RVIZ */
void Map::SendAsRVIZMessage(float point_size, float lanemarking_size, std::shared_ptr<ros::NodeHandle> n) {
	// lazy intialization
	if (!marker_pub_)
		marker_pub_ = std::make_unique<ros::Publisher> (n->advertise<visualization_msgs::MarkerArray>("map_vis_marker", 10, true));

	visualization_msgs::MarkerArray marker_array;
	unsigned int id_offset = 0;
	// publishing the lane markings
	visualization_msgs::Marker lane_markings;
	lane_markings.pose.orientation.w = 1;
	lane_markings.pose.orientation.x = 0;
	lane_markings.pose.orientation.y = 0;
	lane_markings.pose.orientation.z = 0;
	lane_markings.id = id_offset++;
	lane_markings.ns = "lane_markings";
	lane_markings.header.stamp = ros::Time();
	lane_markings.header.frame_id = "map";
	lane_markings.action = visualization_msgs::Marker::ADD;
	lane_markings.type = visualization_msgs::Marker::LINE_LIST;
	lane_markings.scale.x = lanemarking_size;
	// white
	lane_markings.color.r = 1.0;
	lane_markings.color.g = 1.0;
	lane_markings.color.b = 1.0;
	lane_markings.color.a = 1.0;

	for (auto &lanemarking : lanemarking_storage_) {
		auto points = lanemarking.GetPoints();
		// probably for junctions, i hope
		if (!lanemarking.GetVisibility())
			continue;

		geometry_msgs::Point line_start, line_end;
		for (unsigned int i = 0; i < points.size(); ++i) {
			auto type = lanemarking.GetLaneMarkingType();
			// for dashed lanes
			if (type == mapobjects::LaneMarkingType::CENTER_DASHED || type == mapobjects::LaneMarkingType::DASHED) {
				if (i % 2 == 0) {
					line_start.x = points[i].x();
					line_start.y = points[i].y();
					line_start.z = points[i].z();
				} else {
					line_end.x = points[i].x();
					line_end.y = points[i].y();
					line_end.z = points[i].z();
					lane_markings.points.push_back(line_start);
					lane_markings.points.push_back(line_end);
				}
			} else if (i != 0) {
				line_start.x = points[i - 1].x();
				line_start.y = points[i - 1].y();
				line_start.z = points[i - 1].z();
				line_end.x = points[i].x();
				line_end.y = points[i].y();
				line_end.z = points[i].z();
				lane_markings.points.push_back(line_start);
				lane_markings.points.push_back(line_end);
			}
		}
	}
	marker_array.markers.emplace_back(lane_markings);
	// publishing lane points
	visualization_msgs::Marker points;
	points.id = id_offset++;
	points.ns = "lane_points";
	points.header.stamp = ros::Time();
	points.header.frame_id = "map";
	points.action = visualization_msgs::Marker::ADD;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = point_size;
	points.scale.y = point_size;
	points.pose.orientation = geometry_msgs::Quaternion();
	points.color.b = 1.0;
	points.color.a = 1.0;
	points.color.g = 0.8;
	geometry_msgs::Point p;
	for(auto &lane : lane_storage_) {
		for (auto &point3d : lane.GetPointsByReference()) {
			p.x = point3d.x();
			p.y = point3d.y();
			p.z = point3d.z();
			points.points.emplace_back(p);
		}
	}
	marker_array.markers.emplace_back(points);
	// publishing lane uuids
	for(auto &lane : lane_storage_) {
		visualization_msgs::Marker text;
		text.id = id_offset++;
		text.ns = "lane_uuids";
		text.header.stamp = ros::Time();
		text.header.frame_id = "map";
		text.action = visualization_msgs::Marker::ADD;
		text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		text.pose.position.x = lane.GetHandlePoint().x();
		text.pose.position.y = lane.GetHandlePoint().y();
		text.pose.position.z = lane.GetHandlePoint().z();
		text.pose.orientation.x = 0.0;
		text.pose.orientation.y = 0.0;
		text.pose.orientation.z = 0.0;
		text.pose.orientation.w = 1.0;
		text.text = lane.GetUuid().GetUuidValue();
		text.scale.z = point_size * 2;
		if (lane.IsJunctionLane()) {
			text.color.r = 0.9f;
			text.color.g = 0.4f;
			text.color.b = 0.4f;
		} else {
			text.color.r = 1.0f;
			text.color.g = 1.0f;
			text.color.b = 1.0f;
		}
		text.color.a = 1.0f;
		marker_array.markers.emplace_back(text);
	}
	// publishing junction polygons
	for(auto &junction : junction_storage_) {
		visualization_msgs::Marker jpolygon;
		jpolygon.id = id_offset++;
		jpolygon.ns = "junctions";
		jpolygon.header.stamp = ros::Time();
		jpolygon.header.frame_id = "map";
		jpolygon.action = visualization_msgs::Marker::ADD;
		jpolygon.type = visualization_msgs::Marker::LINE_STRIP;
		jpolygon.pose.orientation.w = 1;
		jpolygon.pose.orientation.x = 0;
		jpolygon.pose.orientation.y = 0;
		jpolygon.pose.orientation.z = 0;
		jpolygon.scale.x = lanemarking_size;
		jpolygon.color.r = 0.3f;
		jpolygon.color.g = 0.5f;
		jpolygon.color.b = 0.8f;
		jpolygon.color.a = 1.0f;
		auto boundary_points = junction.GetBoundaryPoints();
		geometry_msgs::Point pt;
		for (auto &bpoint : boundary_points) {
			pt.x = bpoint.x();
			pt.y = bpoint.y();
			pt.z = bpoint.z();
			jpolygon.points.emplace_back(pt);
		}
		pt.x = boundary_points[0].x();
		pt.y = boundary_points[0].y();
		pt.z = boundary_points[0].z();
		jpolygon.points.emplace_back(pt);
		marker_array.markers.emplace_back(jpolygon);
	}
	// publishing junction labels
	for(auto &junction : junction_storage_) {
		visualization_msgs::Marker jtext;
		jtext.id = id_offset++;
		jtext.ns = "junctions";
		jtext.header.stamp = ros::Time();
		jtext.header.frame_id = "map";
		jtext.action = visualization_msgs::Marker::ADD;
		jtext.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		jtext.pose.position.x = junction.GetHandlePoint().x();
		jtext.pose.position.y = junction.GetHandlePoint().y();
		jtext.pose.position.z = junction.GetHandlePoint().z();
		jtext.pose.orientation.x = 0.0;
		jtext.pose.orientation.y = 0.0;
		jtext.pose.orientation.z = 0.0;
		jtext.pose.orientation.w = 1.0;
		jtext.text = "junction #" + std::to_string(junction.GetID());
		jtext.scale.z = point_size * 5;
		jtext.color.r = 0.2f;
		jtext.color.g = 0.4f;
		jtext.color.b = 0.8f;
		jtext.color.a = 1.0f;
		marker_array.markers.emplace_back(jtext);
	}
	// publishing stop lines
	for(auto &lane : lane_storage_) {
		auto *stopline_ptr = lane.GetStopLine();
		if (!stopline_ptr)
			continue;
		visualization_msgs::Marker stopline;
		stopline.id = id_offset++;
		stopline.ns = "lane_objects";
		stopline.header.stamp = ros::Time();
		stopline.header.frame_id = "map";
		stopline.action = visualization_msgs::Marker::ADD;
		stopline.type = visualization_msgs::Marker::LINE_STRIP;
		stopline.pose.orientation.w = 1;
		stopline.pose.orientation.x = 0;
		stopline.pose.orientation.y = 0;
		stopline.pose.orientation.z = 0;
		stopline.scale.x = lanemarking_size;
		stopline.color.r = 0.1f;
		stopline.color.g = 0.1f;
		stopline.color.b = 0.6f;
		stopline.color.a = 1.0f;
		geometry_msgs::Point pt;
		auto point3d = stopline_ptr->position + stopline_ptr->v_forward * 0.03
					 + stopline_ptr->v_right * lane.GetWidth() / 2;
		pt.x = point3d.x(); pt.y = point3d.y(); pt.z = point3d.z();
		stopline.points.emplace_back(pt);
		point3d = point3d - stopline_ptr->v_forward * 0.06;
		pt.x = point3d.x(); pt.y = point3d.y(); pt.z = point3d.z();
		stopline.points.emplace_back(pt);
		point3d = point3d - stopline_ptr->v_right * lane.GetWidth();
		pt.x = point3d.x(); pt.y = point3d.y(); pt.z = point3d.z();
		stopline.points.emplace_back(pt);
		point3d = point3d + stopline_ptr->v_forward * 0.06;
		pt.x = point3d.x(); pt.y = point3d.y(); pt.z = point3d.z();
		stopline.points.emplace_back(pt);
		point3d = point3d + stopline_ptr->v_right * lane.GetWidth();
		pt.x = point3d.x(); pt.y = point3d.y(); pt.z = point3d.z();
		stopline.points.emplace_back(pt);
		marker_array.markers.emplace_back(stopline);
	}
	// publishing junction labels
	for(auto &sign : roadsign_storage_) {
		visualization_msgs::Marker sign_text;
		sign_text.id = id_offset++;
		sign_text.ns = "roadsigns";
		sign_text.header.stamp = ros::Time();
		sign_text.header.frame_id = "map";
		sign_text.action = visualization_msgs::Marker::ADD;
		sign_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		sign_text.text = sign.GetSignType();
		sign_text.pose.position.x = sign.GetPosition().x();
		sign_text.pose.position.y = sign.GetPosition().y();
		sign_text.pose.position.z = sign.GetPosition().z();
		sign_text.pose.orientation.x = 0.0;
		sign_text.pose.orientation.y = 0.0;
		sign_text.pose.orientation.z = 0.0;
		sign_text.pose.orientation.w = 1.0;
		sign_text.scale.z = point_size * 5;
		sign_text.color.r = 1.0f;
		sign_text.color.g = 0.0f;
		sign_text.color.b = 0.0f;
		sign_text.color.a = 1.0f;
		marker_array.markers.emplace_back(sign_text);
		
	}
	marker_pub_->publish(marker_array);
}

/* returns neighboarding lanes in the same lanegroup */
std::tuple<int, int, int> Map::GetNeighboringLanes(const mapobjects::Lane& lane, const mapobjects::LaneGroup& lane_group) {
	auto out_connections = lane.GetOutConnections();
	int adjacent_left_idx = -1, adjacent_right_idx = -1, opposite_idx = -1;
	for (size_t i = 0 ; i < out_connections.size() ; ++i) {
		auto adj_lane = FindLaneByUuid(out_connections[i]);
		// ruling out connections to other lane groups
		if (adj_lane->GetParentUuid() == lane_group.GetUuid()) {
			if (!lane_group.IsJunction()) {
				// if lane is in the same direction
				if (adj_lane->GetLaneDirection() == lane.GetLaneDirection()) {
					mapobjects::Point3D current = lane.GetPoints()[0];
					mapobjects::Point3D forward = lane.GetPoints()[1];
					mapobjects::Point3D adjacent = adj_lane->GetPoints()[0];
					int relative_direction = current.GetRelativeDirection(forward, adjacent);
					if (relative_direction == 1)
						adjacent_right_idx = i;
					if (relative_direction == -1)
						adjacent_left_idx = i;
				} 
				// if it's in the opposite direction (!junction is just due to paranoia)
				else
					opposite_idx = i;
			} else {
				// if it's a junction, no need to check for lane direction as they may be wrong
				mapobjects::Point3D current = lane.GetPoints()[0];
				mapobjects::Point3D forward = lane.GetPoints()[1];
				mapobjects::Point3D adjacent = adj_lane->GetPoints()[0];
				int relative_direction = current.GetRelativeDirection(forward, adjacent);
				if (relative_direction == 1)
					adjacent_right_idx = i;
				if (relative_direction == -1)
					adjacent_left_idx = i;
			}
			
		}
	}
	return std::make_tuple(adjacent_left_idx, adjacent_right_idx, opposite_idx);
}
/* returns the ID of the junction that the current lane leads to, 0 otherwise */
int Map::GetUpcomingJunctionID(const std::string& lane_uuid_value) const{
	auto query = incoming_lane_junction_lookup_.find(lane_uuid_value);
	if (query == incoming_lane_junction_lookup_.end())
		return -1;
	return query->second;
}
/* returns the ID of the junction that the current lane leads to, 0 otherwise */
int Map::GetPastJunctionID(const std::string& lane_uuid_value) const {
	auto query = outgoing_lane_junction_lookup_.find(lane_uuid_value);
	if (query == outgoing_lane_junction_lookup_.end())
		return -1;
	return query->second;
}
int Map::GetCurrentJunctionID(const std::string& lane_uuid_value) const {
	auto query = lane_junction_lookup_.find(lane_uuid_value);
	if (query == lane_junction_lookup_.end())
		return -1;
	return query->second;
}

} // namespace map
} // namespace freicar