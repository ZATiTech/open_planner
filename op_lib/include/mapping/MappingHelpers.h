
/// \file MappingHelpers.h
/// \brief Helper functions for mapping operation such as (load and initialize vector maps , convert map from one format to another, .. )
/// \author Hatem Darweesh
/// \date Jul 2, 2016



#ifndef MAPPINGHELPERS_H_
#define MAPPINGHELPERS_H_

#include "planning/RoadNetwork.h"
#include "utilities/data_rw.h"
#include <math.h>
#include <tinyxml.h>

namespace op {
	namespace mapping {


static EnumString<BOUNDARY_TYPE> BOUNDARY_TYPE_STR(NORMAL_ROAD_BOUNDARY,
		{
				{NORMAL_ROAD_BOUNDARY, "Road"},
				{INTERSECTION_BOUNDARY, "Intersection"},
				{CROSSING_BOUNDARY, "Crossing"},
				{UTURN__BOUNDARY, "U turn"},
				{EXIT_ROAD_BOUNDARY, "Road exit"},
				{MERGE_ROAD_BOUNDARY, "Merge"},
				{HIGHWAY_BOUNDARY, "Highway"},
				{PARKING_BOUNDARY, "Parking"},
				{FREE_SPACE_BOUNDARY, "Free space"},
				{VEGETATION_BOUNDARY, "Vegetation"},
				{KEEP_OUT_BOUNDARY, "Keep out"},
				{BUILDING_BOUNDARY, "Building"},
				{TRAFFIC_ISLAN_BOUNDARY, "Island"},
				{WALK_WAY_BOUNDARY, "Walkway"},
				{SHARED_WALK_WAY_BOUNDARY, "Sharedway"},
				{EXIT_BOUNDARY, "Exit"},
		});

static EnumString<CustomBehaviorType> CustomBehaviorTypeStr(CUSTOM_AVOIDANCE_DISABLED,
		{
				{CUSTOM_AVOIDANCE_DISABLED, "Disabled"},
				{CUSTOM_AVOIDANCE_ENABLED, "Enabled"}
		});

class MappingHelpers {
public:
	MappingHelpers();
	virtual ~MappingHelpers();

	static void UpdateMapWithOccupancyGrid(OccupancyToGridMap& map_info, const std::vector<int>& data, RoadNetwork& map, std::vector<WayPoint*>& updated_list);

	static void UpdateMapWithSignalPose(const std::vector<WayPoint>& points, RoadNetwork& map, std::vector<WayPoint*>& updated_list, const double& min_affect_radius = 0.75, const double& hit_cost = 1);

	static void LinkTrafficLightsIntoGroups(RoadNetwork& map);

	static Lane* GetClosestLaneFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance = 5.0, const bool& bDirectionBased = true);
	static WayPoint* GetClosestWaypointFromMap(const WayPoint& pos, RoadNetwork& map, const bool& bDirectionBased = true);
	static std::vector<Lane*> GetClosestLanesFast(const WayPoint& pos, RoadNetwork& map, const double& distance = 10.0);
	static WayPoint* GetClosestWaypointFromMapUsingDistanceOnly(const WayPoint& pos, RoadNetwork& map, const double& distance = 1.0);
	static std::vector<WayPoint*> GetClosestWaypointsFromMapUsingDistanceOnly(const WayPoint& pos, RoadNetwork& map, const double& distance = 1.0);
	static std::vector<WayPoint*> GetClosestWaypointsListFromMap(const WayPoint& center, RoadNetwork& map, const double& distance = 2.0, const bool& bDirectionBased = true);

	static WayPoint* GetClosestBackWaypointFromMap(const WayPoint& pos, RoadNetwork& map);
	static WayPoint GetFirstWaypoint(RoadNetwork& map);
	static WayPoint* GetLastWaypoint(RoadNetwork& map);
	static void FindAdjacentLanes(RoadNetwork& map);
	static void FindAdjacentLanesV2(RoadNetwork& map, const double& min_d = 1.2, const double& max_d = 3.5);
	/**
	 *
	 *
	 * @param map
	 * @param lane_id
	 * @param dir = 0 for both left and right, = 1 for left lane, = 2 for right lane
	 * @param min_d
	 * @param max_d
	 */
	static void FindAdjacentSingleLane(RoadNetwork& map, const int& lane_id, const int& dir,  const double& min_d = 1.2, const double& max_d = 3.5);

	static void ConnectBoundariesToWayPoints(RoadNetwork& map);

	static void LinkBoundariesToWayPoints(RoadNetwork& map);

	static void LinkMissingBranchingWayPoints(RoadNetwork& map); // pointers link
	static void LinkMissingBranchingWayPointsV2(RoadNetwork& map);  // pointers link
	static void LinkTrafficLightsAndStopLines(RoadNetwork& map);
	static void LinkTrafficLightsAndStopLinesV2(RoadNetwork& map);

	static void ConvertVelocityToMeterPerSecond(RoadNetwork& map);

	static void GetUniqueNextLanes(const Lane* l,  const std::vector<Lane*>& traversed_lanes, std::vector<Lane*>& lanes_list);

	static void RemoveShortTwoPointsLanesFromMap(RoadNetwork& map, double l_length);

	static Lane* GetLaneFromPath(const WayPoint& currPos, const std::vector<WayPoint>& currPath);
	static Lane* GetLaneById(const int& id,RoadNetwork& map);
	static int GetLaneIdByWaypointId(const int& id,std::vector<Lane>& lanes);

	static WayPoint* FindWaypoint(const int& id, RoadNetwork& map);
	static WayPoint* FindWaypointV2(const int& id, const int& l_id, RoadNetwork& map);
	static std::vector<std::string> SplitString(const std::string& str, const std::string& token);

	static void AssignActionCostToLane(Lane* pL, ACTION_TYPE action, double cost);

	static void ConnectLanes(RoadNetwork& map);

	static void ConnectWayPoints(RoadNetwork& map);

	static void InsertUniqueId(std::vector<int>& id_list, int id);

	static void ConnectTrafficLightsAndStopLines(RoadNetwork& map);
	static void ConnectTrafficSignsAndStopLines(RoadNetwork& map);

	static void ConnectMissingStopLinesAndLanes(RoadNetwork& map);

	static void FixRedundantPointsLanes(std::vector<Lane>& lanes);
	static void FixTwoPointsLanes(std::vector<Lane>& lanes);
	static void FixTwoPointsLane(Lane& lanes);
	static void FixUnconnectedLanes(std::vector<Lane>& lanes, const int& max_angle_diff = 90);
	/**
	 * Any distance between consecutive lanes is bigger than 0.25 will be filled by the last point
	 * @param lanes
	 * @param stitching_distance
	 */
	static void StitchLanes(std::vector<Lane>& lanes, const double& min_stitching_distance = 0.25, const double& max_stitching_distance = 4.0);

	/**
	 * Any distance between consecutive lanes is bigger than 0.25 will be filled by the last point
	 * @param lanes
	 * @param stitching_distance
	 */
	static void StitchLanes(RoadNetwork& map, const double& min_stitching_distance = 0.25, const double& max_stitching_distance = 4.0);

	static void TrimPath(std::vector<WayPoint>& points, double trim_angle);

	static void InsertWayPointToBackOfLane(const WayPoint& wp, Lane& lane);
	static void InsertWayPointToFrontOfLane(const WayPoint& wp, Lane& lane);

	static void LinkLanesPointers(RoadNetwork& map);
	static void LinkLaneChangeWaypointsPointers(RoadNetwork& map);

	static void GetMapMaxIds(RoadNetwork& map);

	static void GetClosestStopLines(const RoadNetwork& map, const WayPoint& p, const double& search_radius, std::vector<StopLine>& stop_lines);

	static bool IsPointExist(const WayPoint& p, const std::vector<WayPoint>& points);

	static void InsertUniqueStopLine(std::vector<StopLine>& stop_lines, const StopLine& sl);
	static void InsertUniqueTrafficLight(std::vector<TrafficLight>& traffic_lights, const TrafficLight& tl);
	static void InsertUniqueTrafficSign(std::vector<TrafficSign>& traffic_signs, const TrafficSign& ts);

	static void llaToxyz_proj(const std::string& proj_str, const WayPoint& origin, const double& lat,
			const double& lon, const double& alt, double& x_out, double& y_out, double& z_out);

	static void xyzTolla_proj(const std::string& proj_str, const WayPoint& origin, const double& x_in,
			const double& y_in, const double& z_in, double& lat, double& lon, double& alt);

	static void correct_gps_coor(double& lat,double& lon);
	static void correct_nmea_coor(double& lat,double& lon);

	static std::string FromMarkColorToText(MARKING_COLOR mark_color);
	static std::string FromLineTypeToText(LINE_TYPE type);
	static std::string FromLightTypeToText(TRAFFIC_LIGHT_TYPE type);
	static std::string FromSignTypeToText(TRAFFIC_SIGN_TYPE type);

	static LINE_TYPE FromTextToLineType(std::string type);
	static MARKING_COLOR FromTextToMarkColor(std::string mark_color);
	static TRAFFIC_LIGHT_TYPE FromTextToLightType(std::string light_type);
	static TRAFFIC_SIGN_TYPE FromTextToSignType(std::string sign_type);

	static LINE_TYPE FromNumberToLineType(int type);
	static MARKING_COLOR FromNumberToMarkColor(int color);
	static TRAFFIC_LIGHT_TYPE FromNumberToLightType(int type);
	static TRAFFIC_SIGN_TYPE FromNumberToSignType(int type);

	static std::vector<Lane*> GetClosestMultipleLanesFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance = 5.0);

	static void ShiftMapUsingInternalOrigin(RoadNetwork& map);
	static void ShiftMapUsingIncr(RoadNetwork& map, const double& x_inc, const double& y_inc, const double& z_inc = 0);
	static void ShiftMapItemsUsingIncr(std::vector<Lane>& lanes, std::vector<StopLine>& stop_lines, const double& x_inc, const double& y_inc, const double& z_inc = 0);
	static void RotateMapUsingIncr(RoadNetwork& map, const double&  angle, const double& x_center, const double& y_center);
	/**
	 * This function is added to handle CARLA map issue
	 */
	static void ShiftStopLinesToMatchTrafficLights(RoadNetwork& map);

	static void InsertPointToEndOfPathWithAngleThreshold(const WayPoint& p, std::vector<WayPoint>& path, double max_angle = M_PI_2);

	/**
	 * @brief Reading supporting file which contains projection string and map origin. it should have the same name as the fileName + ".proj.dat"
	 * @param fileName map file name
	 * @param map Map object that will be filled with projection data
	 */
	static void LoadProjectionData(const std::string& fileName, RoadNetwork& map);

	static void UpdatePointWithProjection(const RoadNetwork& map, WayPoint& p);

	static void SplitLane(int lane_id, int point_index, RoadNetwork& map, Lane& modified_lane, Lane& added_lane);

};

} /* namespace mapping */
} /* namespace op */

#endif /* MAPPINGHELPERS_H_ */
