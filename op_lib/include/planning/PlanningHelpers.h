
/// \file PlanningHelpers.h
/// \brief Helper functions for planning algorithms
/// \author Hatem Darweesh
/// \date Jun 16, 2016


#ifndef PLANNINGHELPERS_H_
#define PLANNINGHELPERS_H_

#include "planning/PlannerCommonDef.h"
#include "planning/RoadNetwork.h"
#include "utilities/data_rw.h"
#include <tinyxml.h>

#define DISABLE_CARLA_SPECIAL_CODE

namespace op {
	namespace planning {

#define distance2points(from , to) sqrt(pow(to.x - from.x, 2) + pow(to.y - from.y, 2))
#define distance2pointsSqr(from , to) pow(to.x - from.x, 2) + pow(to.y - from.y, 2)
#define pointNorm(v) sqrt(v.x*v.x + v.y*v.y)
#define angle2points(from , to) atan2(to.y - from.y, to.x - from.x )
#define LANE_CHANGE_SPEED_FACTOR 0.5
#define LANE_CHANGE_COST 50.0 // meters
//#define BACKUP_STRAIGHT_PLAN_DISTANCE 10 //meters
#define LANE_CHANGE_MIN_DISTANCE 8
#define MAX_STEERING_ALLOWED_DELAY 3.0 //seconds

class PlanningHelpers
{

public:
	static std::vector<std::pair<GPSPoint, GPSPoint> > m_TestingClosestPoint;

public:
	PlanningHelpers();
	virtual ~PlanningHelpers();

	/**
	 * @brief For multiple paths input, this function calculate distance from currPose to the end of each path, if the distance to the end of the path is less than end_ragne_distance , the function returns the index of the path, if not it returns -1.
	 * @param paths
	 * @param currPose
	 * @param end_range_distance
	 * @return -1 if distance from currPose to the end of all paths is less than end_range_distance.
	 */
	static int CheckForEndOfPaths(const std::vector<std::vector<WayPoint> >& paths, const WayPoint& currPose, const double& end_range_distance);

	static bool GetRelativeInfo(const std::vector<WayPoint>& trajectory, const WayPoint& p, RelativeInfo& info, const int& prevIndex = 0);

	static bool GetRelativeInfoDirectionLimited(const std::vector<WayPoint>& trajectory, const WayPoint& p, RelativeInfo& info, const int& prevIndex = 0);

	static bool GetRelativeInfoLimited(const std::vector<WayPoint>& trajectory, const WayPoint& p, RelativeInfo& info, const int& prevIndex = 0);

	static bool GetRelativeInfoDirection(const std::vector<WayPoint>& trajectory, const WayPoint& p, RelativeInfo& info, const int& prevIndex =0);

	static bool GetRelativeInfoRange(const std::vector<std::vector<WayPoint> >& trajectories, const WayPoint& p, const double& searchDistance, RelativeInfo& info);

	static WayPoint GetFollowPointOnTrajectory(const std::vector<WayPoint>& trajectory, const RelativeInfo& init_p, const double& distance, unsigned int& point_index);

	static double GetExactDistanceOnTrajectory(const std::vector<WayPoint>& trajectory, const RelativeInfo& p1,const RelativeInfo& p2);

	static int GetClosestNextPointIndex_obsolete(const std::vector<WayPoint>& trajectory, const WayPoint& p, const int& prevIndex = 0);

	static int GetClosestNextPointIndexFast(const std::vector<WayPoint>& trajectory, const WayPoint& p, const int& prevIndex = 0);

	static int GetClosestNextPointIndexFastV2(const std::vector<WayPoint>& trajectory, const WayPoint& p, const int& prevIndex = 0);

	static int GetClosestNextPointIndexDirectionFast(const std::vector<WayPoint>& trajectory, const WayPoint& p, const int& prevIndex = 0, const bool& debug = false);

	static int GetClosestPointIndex(const std::vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex = 0 );

	static WayPoint GetPerpendicularOnTrajectory_obsolete(const std::vector<WayPoint>& trajectory, const WayPoint& p, double& distance, const int& prevIndex = 0);	static double GetPerpDistanceToTrajectorySimple_obsolete(const std::vector<WayPoint>& trajectory, const WayPoint& p, const int& prevIndex = 0);

	static double GetPerpDistanceToVectorSimple_obsolete(const WayPoint& p1, const WayPoint& p2, const WayPoint& pose);

	static WayPoint GetNextPointOnTrajectory_obsolete(const std::vector<WayPoint>& trajectory, const double& distance, const int& currIndex = 0);

	static double GetDistanceOnTrajectory_obsolete(const std::vector<WayPoint>& path, const int& start_index, const WayPoint& p);

	static void CreateManualBranch(std::vector<WayPoint>& path, const DIRECTION_TYPE& direction);

	static void CreateManualBranchFromTwoPoints(WayPoint& p1,WayPoint& p2 , const double& distance, const DIRECTION_TYPE& direction, std::vector<WayPoint>& path);

	//Change points position along the path so the distance between all points are the same and equal to density distance
	static void FixPathDensity(std::vector<WayPoint>& path, const double& distanceDensity);

	// Just add new point if distance between any two points is bigger than the res distance
	static void FixPathResolution(std::vector<WayPoint>& path, const double& res);

	static void FixPathDensity(std::vector<GPSPoint>& path, const double& distanceDensity);

	static void SmoothPath(std::vector<WayPoint>& path, double weight_data =0.25,double weight_smooth = 0.25,double tolerance = 0.01);

	static void SmoothPath(std::vector<GPSPoint>& path, double weight_data =0.25,double weight_smooth = 0.25,double tolerance = 0.01);

	static double CalcCircle(const GPSPoint& pt1, const GPSPoint& pt2, const GPSPoint& pt3, GPSPoint& center);

	// static double CalcCircleV2(const WayPoint& pt1, const WayPoint& pt2, const WayPoint& pt3, WayPoint& center);

	static void FixAngleOnly(std::vector<WayPoint>& path);

	static double CalcAngleAndCost(std::vector<WayPoint>& path, const double& lastCost = 0);

	//static double CalcAngleAndCostSimple(std::vector<WayPoint>& path, const double& lastCost = 0);

	static void CalcAngleAndCurvatureCost(std::vector<WayPoint>& path);

	static void CalcDtLaneInfo(std::vector<WayPoint>& path);

	static void PredictConstantTimeCostForTrajectory(std::vector<WayPoint>& path, const WayPoint& currPose, const double& minVelocity);

	static double GetAccurateDistanceOnTrajectory(std::vector<WayPoint>& path, const int& start_index, const WayPoint& p);

	static void ExtractPartFromPointToDistanceFast(const std::vector<WayPoint>& originalPath, const WayPoint& pos, const double& minDistance,
				const double& pathDensity, std::vector<WayPoint>& extractedPath);

	static int ExtractPartFromPointToDistanceDirectionFast(const std::vector<WayPoint>& originalPath, const WayPoint& pos, const double& minDistance,
			const double& pathDensity, std::vector<WayPoint>& extractedPath, int prev_index = 0);

	static void CalculateRollInTrajectories(const WayPoint& carPos, const double& speed, const std::vector<WayPoint>& originalCenter, int& start_index,
			int& end_index, std::vector<double>& end_laterals ,
			std::vector<std::vector<WayPoint> >& rollInPaths, const double& max_roll_distance,
			const double&  carTipMargin, const double& rollInMargin,
			const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
			const int& rollOutsNumber, const double& SmoothDataWeight, const double& SmoothWeight,
			const double& SmoothTolerance, const bool& bHeadingSmooth,
			std::vector<WayPoint>& sampledPoints);

	static void SmoothSpeedProfiles(std::vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance	= 0.1);

	static void SmoothCurvatureProfiles(std::vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance = 0.1);

	static void SmoothZ(std::vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance = 0.1);

	static void SmoothWayPointsDirections(std::vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance	= 0.1);

	static void SmoothGlobalPathSpeed(std::vector<WayPoint>& path);

	static void GenerateRecommendedSpeed(std::vector<WayPoint>& path, const double& max_speed, const double& speedProfileFactor);

	/**
	 *
	 * @param dt
	 * @param CurrSpeed
	 * @param vehicleInfo uses (max_acceleration, max deceleration, max_speed)
	 * @param ctrlParams uses (accel_push, brake_push, safe_follow_distance)
	 * @param CurrBehavior uses (state, max_velocity, stop_distance, follow_distance)
	 * @return
	 */
	static double GetACCVelocityModelBased(const double& dt, const double& CurrSpeed, const CAR_BASIC_INFO& vehicleInfo,
			const ControllerParams& ctrlParams, const BehaviorState& CurrBehavior);

	static void ShiftRecommendedSpeed(std::vector<WayPoint>& path, const double& max_speed, const double& curr_speed, const double& inc_ratio, const double& path_density);

	static WayPoint* BuildPlanningSearchTreeV2(WayPoint* pStart,
			const WayPoint& goalPos,
			const std::vector<int>& globalPath, const double& DistanceLimit,
			const bool& bEnableLaneChange,
			std::vector<WayPoint*>& all_cells_to_delete );

	static WayPoint* BuildPlanningSearchTreeStraight(WayPoint* pStart,
			const double& DistanceLimit,
			std::vector<WayPoint*>& all_cells_to_delete );

	static int PredictiveDP(WayPoint* pStart, const double& DistanceLimit,
			std::vector<WayPoint*>& all_cells_to_delete, std::vector<WayPoint*>& end_waypoints);

	static int PredictiveIgnorIdsDP(WayPoint* pStart, const double& DistanceLimit,
				std::vector<WayPoint*>& all_cells_to_delete, std::vector<WayPoint*>& end_waypoints, std::vector<int>& lanes_ids);

	static bool CheckLaneIdExits(const std::vector<int>& lanes, const Lane* pL);

	static WayPoint* CheckLaneExits(const std::vector<WayPoint*>& nodes, const Lane* pL);

	static WayPoint* CheckNodeExits(const std::vector<WayPoint*>& nodes, const WayPoint* pL);

	static WayPoint* CreateLaneHeadCell(Lane* pLane, WayPoint* pLeft, WayPoint* pRight,
			WayPoint* pBack);

	static double GetLanePoints(Lane* l, 
			const double& minDistance , const double& prevCost, std::vector<WayPoint>& points);

	static WayPoint* GetMinCostCell(const std::vector<WayPoint*>& cells, const std::vector<int>& globalPathIds);

	static void TraversePathTreeBackwards(WayPoint* pHead, WayPoint* pStartWP, const std::vector<int>& globalPathIds,
			std::vector<WayPoint>& localPath, std::vector<std::vector<WayPoint> >& localPaths);

	static double CalculateLookAheadDistance(const double& steering_delay, const double& curr_velocity, const double& min_distance, double speed_factor = 0.15, double delay_factor = 1.0);

	static void PredictMotionTimeBased(double& x, double &y, double& heading, double steering, double velocity, double wheelbase, double time_elapsed);

	static void PredictMotionDistanceBased(double& x, double &y, double& heading, double steering, double distance, double wheelbase);

	static void EstimateFuturePosition(const WayPoint& currPose, const double& currSteering, const double& est_distance,
			const double& est_resolution, const double& wheel_base, WayPoint& estimatedPose);

	static void ExtractPlanAlernatives(const std::vector<WayPoint>& singlePath, const double& plan_distance, std::vector<std::vector<WayPoint> >& allPaths, double lane_change_distance = 10);

	static void ExtractPlanAlernativesV2(const std::vector<WayPoint>& singlePath, const double& plan_distance, const double& lane_change_distane, std::vector<std::vector<WayPoint> >& allPaths);

	static void ExtractPlanAlernativesSection(const WayPoint& startPose, const double& plan_distance, std::vector<WayPoint>& path);

	static void RemoveFromPathUntil(std::vector<WayPoint>& path, const double& distance);

	static std::vector<int> GetUniqueLeftRightIds(const std::vector<WayPoint>& path);

	static bool FindInList(const std::vector<int>& list,const int& x);

	// static ACTION_TYPE GetBranchingDirection(WayPoint& currWP, WayPoint& nextWP);

	static void CalcContourPointsForDetectedObjects(const WayPoint& currPose, std::vector<DetectedObject>& obj_list, const double& filterDistance = 100);

	static double GetVelocityAhead(const std::vector<WayPoint>& path, const RelativeInfo& info,int& prev_index, const double& reasonable_brake_distance);

	static double GetCurvatureCostAhead(const std::vector<WayPoint>& path, const RelativeInfo& info,int& prev_index, const double& search_distance);

	static bool CompareTrajectories(const std::vector<WayPoint>& path1, const std::vector<WayPoint>& path2);

	static double GetDistanceToClosestStopLineAndCheck(const std::vector<WayPoint>& path, const WayPoint& p, const double& giveUpDistance, int& stopLineID,int& stopSignID, int& trafficLightID, const int& prevIndex = 0);

	static double GetDistanceToClosestStopLineAndCheckV2(const std::vector<WayPoint>& path, const WayPoint& p, const std::vector<StopLine>& slines, int& stopLineID,int& stopSignID, std::vector<int>& trafficLightIDs);

	static bool GetThreePointsInfo(const WayPoint& p0, const WayPoint& p1, const WayPoint& p2, WayPoint& perp_p, double& long_d, double lat_d);

	static void WritePathToFile(const std::string& fileName, const std::vector<WayPoint>& path);

	static LIGHT_INDICATOR GetIndicatorsFromPath(const std::vector<WayPoint>& path, const WayPoint& pose, const double& seachDistance);

	static WayPoint GetRealCenter(const WayPoint& currState, const double& wheel_base);

	static void GetCubeAndCenterofTwoPoints(const WayPoint& p1, const WayPoint& p2, double width, double depth,
			WayPoint& center_p, WayPoint& min_p, WayPoint& max_p);

	static std::string MakePathID(const std::vector<WayPoint>& _path);
	static std::string MakePathDirectionID(const std::vector<WayPoint>& _path);

	static double GetDistanceFromPoseToEnd(const WayPoint& pose, const std::vector<WayPoint>& path);

	static void InitializeSafetyPolygon(const WayPoint& curr_state, const CAR_BASIC_INFO& car_info,
	                                                  const VehicleState& vehicle_state, const double& lateral_safe_d,
	                                                  const double& long_safe_d, const bool& use_turning_angle, PolygonShape& car_border);

	static int PointInsidePolygon(const std::vector<GPSPoint>& points,const GPSPoint& p);

	static int PointInsidePolygon(const std::vector<WayPoint>& points,const WayPoint& p);

	static void TestQuadraticSpline(const std::vector<WayPoint>& center_line, std::vector<WayPoint>& path);

	static bool CheckFrontLane(WayPoint* pWP1, WayPoint* pWP2, int search_level);
	static bool CheckBackLane(WayPoint* pWP1, WayPoint* pWP2, int search_level);
	static bool CheckRightLane(WayPoint* pWP1, WayPoint* pWP2);
	static bool CheckLeftLane(WayPoint* pWP1, WayPoint* pWP2);
	static void FilterWaypoints(std::vector<WayPoint*>& wp_list, WayPoint* pPrevWP);

	static WayPoint CalcCenterPoint(const std::vector<WayPoint>& points);

	static double frunge ( double x );

	static double fprunge ( double x );

	static double fpprunge ( double x );

};

static EnumString<TRAFFIC_LIGHT_TYPE> TRAFFIC_LIGHT_TYPE_STR(UNKNOWN_LIGHT,
		{
				{UNKNOWN_LIGHT, "Unknown"},
				{RED_LIGHT, "Red"},
				{GREEN_LIGHT, "Green"},
				{YELLOW_LIGHT, "Yellow"},
				{CROSS_GREEN, "Cross Green"},
				{CROSS_RED, "Cross Red"},
				{LEFT_GREEN, "Left Green"},
				{FORWARD_GREEN, "Forward Green"},
				{RIGHT_GREEN, "Right Green"},
				{FLASH_YELLOW, "Flash Yellow"},
				{FLASH_RED, "Flash Red"},
		});

} /* namespace planning */
} /* namespace op */

#endif /* PLANNINGHELPERS_H_ */
