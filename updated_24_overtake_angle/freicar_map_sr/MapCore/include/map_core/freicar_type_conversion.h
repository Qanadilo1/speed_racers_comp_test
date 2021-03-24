#ifndef __AISCAR_TYPE_CONVERSION_H__
#define __AISCAR_TYPE_CONVERSION_H__

#include "map_core/freicar_map_objects.h"
#include "gen-cpp/map_data_structure_types.h"
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
	// map_thrift to mapobjects conversion
	mapobjects::LaneMarkingContainer ToMapObjectLaneMarkingContainer(const map_thrift::LaneMarkingContainer &lmc);
	mapobjects::LaneMarkingType 	 ToMapObjectLaneMarkingType(const map_thrift::LaneMarkingType::type &type);
	mapobjects::LaneDirection 		 ToMapObjectLaneDirection(const map_thrift::LaneDirection::type &type);
	std::vector<mapobjects::Point3D> ToMapObjectPoints(const std::vector<map_thrift::Point3D> &points);
	mapobjects::Crosswalk			 ToMapObjectsCrossWalk(const map_thrift::Crosswalk &crosswalk);
	std::vector<mapobjects::Uuid> 	 ToMapObjectsUuids(const std::vector<map_thrift::Uuid> &uuids);
	mapobjects::Stopline			 ToMapObjectsStopLine(const map_thrift::Stopline &stop_line);
	mapobjects::Parking			 	 ToMapObjectsParking(const map_thrift::Parking &parking_lot);
	mapobjects::LaneMarking 		 ToMapObjectsLanemarking(const map_thrift::LaneMarking &lm);
	mapobjects::Roadsign			 ToMapObjectsRoadsign(const map_thrift::Roadsign &roadsign);
	mapobjects::Pivot 				 ToMapObjectsPivot(const map_thrift::Pivot &thrift_pivot);
	mapobjects::LaneType 			 ToMapObjectLaneType(const map_thrift::LaneType::type &type);
	mapobjects::LaneGroup 			 ToMapObjectsLaneGroup(const map_thrift::LaneGroup &lg);
	mapobjects::Lane 				 ToMapObjectsLane(const map_thrift::Lane &lane);
	
	// mapobjects to map_thrift conversion
	map_thrift::Lane 					 ToThriftLane(const mapobjects::Lane &lane);
	map_thrift::Uuid 					 ToThriftUuid(const mapobjects::Uuid &uuid);
	map_thrift::LaneType::type 			 ToThriftLaneType(const mapobjects::LaneType &type);
	map_thrift::Point3D 				 ToThriftPoint(const mapobjects::Point3D &point);
	map_thrift::Pivot 					 ToThriftPivot(const mapobjects::Pivot &map_pivot);
	map_thrift::LaneGroup 				 ToThriftLaneGroup(const mapobjects::LaneGroup &lg);
	map_thrift::LaneMarking 			 ToThriftLaneMarking(const mapobjects::LaneMarking &lm);
	std::vector<map_thrift::Uuid> 		 ToThriftUuids(const std::vector<mapobjects::Uuid> &uuids);
	map_thrift::LaneDirection::type 	 ToThriftLaneDirection(const mapobjects::LaneDirection &dir);
	map_thrift::LaneMarkingType::type 	 ToThriftLaneMarkingType(const mapobjects::LaneMarkingType &type);
	std::vector<map_thrift::Point3D> 	 ToThriftPoints(const std::vector<mapobjects::Point3D> &points);
	std::vector<map_thrift::Lane> 		 ToThriftLaneBatch(const std::vector<mapobjects::Lane> &l_list);
	map_thrift::LaneMarkingContainer 	 ToThriftLaneMarkingContainer(const mapobjects::LaneMarkingContainer &lmc);
	std::vector<map_thrift::LaneGroup> 	 ToThriftLaneGroupBatch(const std::vector<mapobjects::LaneGroup> &lg_list);
	std::vector<map_thrift::LaneMarking> ToThriftLaneMarkingBatch(const std::vector<mapobjects::LaneMarking> &lm_list);
	std::vector<map_thrift::Stopline>    ToThriftStoplineBatch(const std::vector<mapobjects::Stopline> &sl_list);
	std::vector<map_thrift::Roadsign>    ToThriftRoadsignBatch(const std::vector<mapobjects::Roadsign> &rs_list);
	std::vector<map_thrift::Crosswalk>   ToThriftCrosswalkBatch(const std::vector<mapobjects::Crosswalk> &cw_list);
	std::vector<map_thrift::Parking>	 ToThriftParkingBatch(const std::vector<mapobjects::Parking> &pk_list);
} // namespace convert

namespace create
{
	map_thrift::MapPart 		MapPartFromMapObjects(const std::vector<map_thrift::LaneMarking> &v_lm,
											  		  const std::vector<map_thrift::Lane> &v_l,
											  		  const std::vector<map_thrift::LaneGroup> &v_lg,
													  const map_thrift::LaneObjectList& v_lo);
	map_thrift::MapContainer 	MapContainerFromMapParts(const std::vector<map_thrift::MapPart> &m_parts,
													  	 const mapobjects::Pivot &map_pivot);
	map_thrift::MapContainer 	MapContainerFromMapParts(const map_thrift::MapPart &m_part,
													  	 const mapobjects::Pivot &map_pivot);
	map_thrift::MapMessage 		MapAddMessage(const map_thrift::MapContainer &m_container);
	map_thrift::MapMessage 		MapUpdateWholeMessage(const map_thrift::MapContainer &m_container);
	map_thrift::MapMessage 		MapUpdatePartMessage(const map_thrift::MapContainer &m_container);
	map_thrift::MapMessage 		MapDeleteMessage(const map_thrift::MapContainer &m_container);

} // namespace create
} // namespace freicar


#endif