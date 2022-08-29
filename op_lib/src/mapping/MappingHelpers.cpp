
/// \file MappingHelpers.cpp
/// \brief Helper functions for mapping operation such as (load and initialize vector maps , convert map from one format to another, .. )
/// \author Hatem Darweesh
/// \date Jul 2, 2016



#include "mapping/MappingHelpers.h"
#include "utilities/matrix_operations.h"
#include "planning/PlanningHelpers.h"
#include <float.h>
#include <fstream>
#include <algorithm>

#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H
#include <proj_api.h>


#define RIGHT_INITIAL_TURNS_COST 0
#define LEFT_INITIAL_TURNS_COST 0
#define DEBUG_MAP_PARSING 0
#define DEFAULT_REF_VELOCITY 60 //km/h

namespace op {
	namespace mapping {

using namespace std;
using namespace planning;
using namespace utilities;

constexpr int ANGLE_MAX_FOR_DIRECTION_CHECK = 15;
constexpr double MAX_DISTANCE_TO_START_LANE_DETECTION = 100;

MappingHelpers::MappingHelpers() {
}

MappingHelpers::~MappingHelpers() {
}

void MappingHelpers::ConvertVelocityToMeterPerSecond(RoadNetwork& map)
{
	if(map.roadSegments.size() == 0) return;

	for(auto& l: map.roadSegments.at(0).Lanes)
	{
		l.speed = l.speed/3.6;
		for(auto& p: l.points)
		{
			p.v = p.v/3.6;
		}
	}
}

void MappingHelpers::RemoveShortTwoPointsLanesFromMap(RoadNetwork& map, double l_length)
{
	for(auto& rseg : map.roadSegments)
	{
		for(int i = 0 ; i < (int)rseg.Lanes.size(); i++)
		{
			Lane* pL = &rseg.Lanes.at(i);
			if((int)pL->points.size() < 4)
			{
				double d = PlanningHelpers::CalcAngleAndCost(pL->points);

				if(d < l_length)
				{
					for(auto& f_id : pL->fromIds)
					{
						Lane* pPrev = GetLaneById(f_id, map);
						if(pPrev != nullptr)
						{
							for(unsigned int k = 0 ; k < pPrev->toIds.size(); k++)
							{
								if(pPrev->toIds.at(k) == pL->id)
								{
									pPrev->toIds.erase(pPrev->toIds.begin()+k);
									break;
								}
							}
							pPrev->toIds.insert(pPrev->toIds.begin(), pL->toIds.begin(), pL->toIds.end());
						}
					}

					for(auto& t_id : pL->toIds)
					{
						Lane* pNext = GetLaneById(t_id, map);
						if(pNext != nullptr)
						{
							for(unsigned int k = 0 ; k < pNext->fromIds.size(); k++)
							{
								if(pNext->fromIds.at(k) == pL->id)
								{
									pNext->fromIds.erase(pNext->fromIds.begin()+k);
									break;
								}
							}
							pNext->fromIds.insert(pNext->fromIds.begin(), pL->fromIds.begin(), pL->fromIds.end());
						}
					}

					//std::cout << "Short Lane: " << pL->id << ", size: " << pL->points.size() << ", Length: " << d << std::endl;
					rseg.Lanes.erase(rseg.Lanes.begin()+i);
					i--;
				}
			}
		}
	}
}

Lane* MappingHelpers::GetLaneById(const int& id,RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			if(map.roadSegments.at(rs).Lanes.at(i).id == id)
				return &map.roadSegments.at(rs).Lanes.at(i);
		}
	}

	return nullptr;
}

int MappingHelpers::GetLaneIdByWaypointId(const int& id,std::vector<Lane>& lanes)
{
	for(unsigned int in_l= 0; in_l < lanes.size(); in_l++)
	{
		for(unsigned int in_p = 0; in_p<lanes.at(in_l).points.size(); in_p++)
		{
			if(id == lanes.at(in_l).points.at(in_p).id)
			{
				return lanes.at(in_l).points.at(in_p).laneId;
			}
		}
	}

	return 0;
}

void MappingHelpers::AssignActionCostToLane(Lane* pL, ACTION_TYPE action, double cost)
{
  for(unsigned int j = 0 ; j < pL->points.size(); j++)
  {
      pL->points.at(j).actionCost.clear();
      pL->points.at(j).actionCost.push_back(make_pair(action, cost));
  }
}

WayPoint* MappingHelpers::FindWaypoint(const int& id, RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				if(map.roadSegments.at(rs).Lanes.at(i).points.at(p).id == id)
					return &map.roadSegments.at(rs).Lanes.at(i).points.at(p);
			}
		}
	}

	return nullptr;
}

WayPoint* MappingHelpers::FindWaypointV2(const int& id, const int& l_id, RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pLane = &map.roadSegments.at(rs).Lanes.at(i);
			if(pLane ->id != l_id)
			{
				for(unsigned int p= 0; p < pLane->points.size(); p++)
				{
					if(pLane->points.at(p).id == id)
						return &pLane->points.at(p);
				}
			}
		}
	}

	return nullptr;
}

void MappingHelpers::LinkTrafficLightsIntoGroups(RoadNetwork& map)
{
	//First get max group ID;
	int max_group_id = 0;
	for(auto& x : map.trafficLights)
	{
		if(x.groupID > max_group_id)
		{
			max_group_id = x.groupID;
		}
	}

	for(auto& x : map.trafficLights)
	{
		if(x.groupID == 0)
		{
			max_group_id++;
			//Find the closest single traffic light bulbs
			for(auto& c : map.trafficLights)
			{
				if(c.groupID == 0)
				{
					double d = hypot(x.pose.pos.y - c.pose.pos.y, x.pose.pos.x - c.pose.pos.x);
					if(d < 1.0 && fabs(x.vertical_angle - c.vertical_angle) < 10 && fabs(x.horizontal_angle - c.horizontal_angle) < 10)
					{
						c.groupID = max_group_id;
					}
				}
			}
		}
	}
}

WayPoint* MappingHelpers::GetClosestWaypointFromMap(const WayPoint& pos, RoadNetwork& map, const bool& bDirectionBased)
{
	WayPoint* pWaypoint = nullptr;
	double min_d = DBL_MAX;

	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			Lane* pL = &map.roadSegments.at(j).Lanes.at(k);
			RelativeInfo info;
			if(PlanningHelpers::GetRelativeInfoLimited(pL->points, pos, info))
			{
				if(bDirectionBased && fabs(info.angle_diff) > ANGLE_MAX_FOR_DIRECTION_CHECK)
					continue;

				double d = fabs(info.perp_distance);
				if(info.bAfter)
				{
					d = hypot(pL->points.at(pL->points.size()-1).pos.y - pos.pos.y, pL->points.at(pL->points.size()-1).pos.x - pos.pos.x);
				}
				else if(info.bBefore)
				{
					d = hypot(pL->points.at(0).pos.y - pos.pos.y, pL->points.at(0).pos.x - pos.pos.x);
				}

				if(d < MAX_DISTANCE_TO_START_LANE_DETECTION && d < min_d)
				{
					pWaypoint = &pL->points.at(info.iFront);
					min_d = d;
				}
			}
		}
	}

	return pWaypoint;
}

vector<WayPoint*> MappingHelpers::GetClosestWaypointsListFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance, const bool& bDirectionBased)
{
	vector<WayPoint*> waypoints_list;

	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			Lane* pL = &map.roadSegments.at(j).Lanes.at(k);
			RelativeInfo info;
			if(PlanningHelpers::GetRelativeInfoLimited(pL->points, pos, info))
			{
				if(bDirectionBased && fabs(info.angle_diff) > ANGLE_MAX_FOR_DIRECTION_CHECK)
					continue;

				double d = fabs(info.perp_distance);
				if(info.bAfter)
				{
					d = hypot(pL->points.at(pL->points.size()-1).pos.y - pos.pos.y, pL->points.at(pL->points.size()-1).pos.x - pos.pos.x);
				}
				else if(info.bBefore)
				{
					d = hypot(pL->points.at(0).pos.y - pos.pos.y, pL->points.at(0).pos.x - pos.pos.x);
				}

				if(d < distance)
				{
					waypoints_list.push_back(&pL->points.at(info.iBack));
				}
			}
		}
	}

	return waypoints_list;
}

WayPoint* MappingHelpers::GetClosestBackWaypointFromMap(const WayPoint& pos, RoadNetwork& map)
{
	double distance_to_nearest_lane = 1;
	Lane* pLane = 0;
	while(distance_to_nearest_lane < 100 && pLane == 0)
	{
		pLane = GetClosestLaneFromMap(pos, map, distance_to_nearest_lane);
		distance_to_nearest_lane += 1;
	}

	if(!pLane) return nullptr;

	int closest_index = PlanningHelpers::GetClosestNextPointIndexFast(pLane->points, pos);

	if(closest_index>2)
		return &pLane->points.at(closest_index-3);
	else if(closest_index>1)
		return &pLane->points.at(closest_index-2);
	else if(closest_index>0)
		return &pLane->points.at(closest_index-1);
	else
		return &pLane->points.at(closest_index);
}

WayPoint* MappingHelpers::GetClosestWaypointFromMapUsingDistanceOnly(const WayPoint& pos, RoadNetwork& map, const double& distance)
{
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			Lane* pL = &map.roadSegments.at(j).Lanes.at(k);
			int index = PlanningHelpers::GetClosestNextPointIndexFast(pL->points, pos);

			if(index < 0 || index >= (int)pL->points.size()) continue;

			double d = hypot(pL->points.at(index).pos.y - pos.pos.y, pL->points.at(index).pos.x - pos.pos.x);
			if(d <= distance)
			{
				return &pL->points.at(index);
			}			
		}
	}

	return nullptr;
}

std::vector<WayPoint*> MappingHelpers::GetClosestWaypointsFromMapUsingDistanceOnly(const WayPoint& pos, RoadNetwork& map, const double& distance)
{
	std::vector<WayPoint*> waypoint_list;
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			Lane* pL = &map.roadSegments.at(j).Lanes.at(k);
			int index = PlanningHelpers::GetClosestNextPointIndexFast(pL->points, pos);

			if(index < 0 || index >= (int)pL->points.size()) continue;

			double d = hypot(pL->points.at(index).pos.y - pos.pos.y, pL->points.at(index).pos.x - pos.pos.x);
			if(d <= distance)
			{
				waypoint_list.push_back(&pL->points.at(index));
			}
		}
	}

	return waypoint_list;
}

std::vector<Lane*> MappingHelpers::GetClosestLanesFast(const WayPoint& center, RoadNetwork& map, const double& distance)
{
	vector<Lane*> lanesList;
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			Lane* pL = &map.roadSegments.at(j).Lanes.at(k);
			int index = PlanningHelpers::GetClosestNextPointIndexFast(pL->points, center);

			if(index < 0 || index >= (int)pL->points.size()) continue;

			double d = hypot(pL->points.at(index).pos.y - center.pos.y, pL->points.at(index).pos.x - center.pos.x);
			if(d <= distance)
			{
				lanesList.push_back(pL);
			}
		}
	}

	return lanesList;
}

Lane* MappingHelpers::GetClosestLaneFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance, const bool& bDirectionBased)
{
	Lane* pCloseLane = nullptr;
	double min_d = DBL_MAX;

	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			Lane* pL = &map.roadSegments.at(j).Lanes.at(k);
			RelativeInfo info;
			if(PlanningHelpers::GetRelativeInfoLimited(pL->points, pos, info))
			{
				if(bDirectionBased && fabs(info.angle_diff) > ANGLE_MAX_FOR_DIRECTION_CHECK)
					continue;

				double d = fabs(info.perp_distance);
				if(info.bAfter)
				{
					d = hypot(pL->points.at(pL->points.size()-1).pos.y - pos.pos.y, pL->points.at(pL->points.size()-1).pos.x - pos.pos.x);
				}
				else if(info.bBefore)
				{
					d = hypot(pL->points.at(0).pos.y - pos.pos.y, pL->points.at(0).pos.x - pos.pos.x);
				}

				if(d < distance && d < min_d)
				{
					pCloseLane = pL;
					min_d = d;
				}
			}
		}
	}

	return pCloseLane;
}

WayPoint MappingHelpers::GetFirstWaypoint(RoadNetwork& map)
{
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			for(unsigned int pindex=0; pindex< map.roadSegments.at(j).Lanes.at(k).points.size(); pindex ++)
			{
				WayPoint fp =  map.roadSegments.at(j).Lanes.at(k).points.at(pindex);
				return fp;
			}
		}
	}

	return WayPoint();
}

WayPoint* MappingHelpers::GetLastWaypoint(RoadNetwork& map)
{
	if(map.roadSegments.size() > 0 && map.roadSegments.at(map.roadSegments.size()-1).Lanes.size() > 0)
	{
		std::vector<Lane>* lanes = &map.roadSegments.at(map.roadSegments.size()-1).Lanes;
		if(lanes->at(lanes->size()-1).points.size() > 0)
			return &lanes->at(lanes->size()-1).points.at(lanes->at(lanes->size()-1).points.size()-1);
	}

	return nullptr;
}

void MappingHelpers::GetUniqueNextLanes(const Lane* l,  const vector<Lane*>& traversed_lanes, vector<Lane*>& lanes_list)
{
	if(!l) return;

	for(unsigned int i=0; i< l->toLanes.size(); i++)
	{
		bool bFound = false;
		for(unsigned int j = 0; j < traversed_lanes.size(); j++)
		if(l->toLanes.at(i)->id == traversed_lanes.at(j)->id)
		{
			bFound = true;
			break;
		}

		if(!bFound)
			lanes_list.push_back(l->toLanes.at(i));
	}
}

Lane* MappingHelpers::GetLaneFromPath(const WayPoint& currPos, const std::vector<WayPoint>& currPath)
{
	if(currPath.size() < 1) return nullptr;

	int closest_index = PlanningHelpers::GetClosestNextPointIndexFast(currPath, currPos);

	return currPath.at(closest_index).pLane;
}

vector<string> MappingHelpers::SplitString(const string& str, const string& token)
{
	vector<string> str_parts;
	int iFirstPart = str.find(token);

	while(iFirstPart >= 0)
	{
		iFirstPart++;
		int iSecondPart = str.find(token, iFirstPart);
		if(iSecondPart>0)
		{
			str_parts.push_back(str.substr(iFirstPart,iSecondPart - iFirstPart));
		}
		else
		{
			str_parts.push_back(str.substr(iFirstPart,str.size() - iFirstPart));
		}

		iFirstPart = iSecondPart;
	}

	return str_parts;
}

void MappingHelpers::FindAdjacentLanes(RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		//Link Lanes
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			//Link left and right lanes
			for(unsigned int rs_2 = 0; rs_2 < map.roadSegments.size(); rs_2++)
			{
				for(unsigned int i2 =0; i2 < map.roadSegments.at(rs_2).Lanes.size(); i2++)
				{
					int iCenter1 = (int)pL->points.size()/2;
					WayPoint wp_1 = pL->points.at(iCenter1);
					int iCenter2 = PlanningHelpers::GetClosestNextPointIndexFast(map.roadSegments.at(rs_2).Lanes.at(i2).points, wp_1 );
					WayPoint closest_p = map.roadSegments.at(rs_2).Lanes.at(i2).points.at(iCenter2);
					double mid_a1 = wp_1.pos.a;
					double mid_a2 = closest_p.pos.a;
					double angle_diff = Angle::AngleBetweenTwoAnglesPositive(mid_a1, mid_a2);
					double distance = distance2points(wp_1.pos, closest_p.pos);

					if(pL->id != map.roadSegments.at(rs_2).Lanes.at(i2).id && angle_diff < 0.05 && distance < 3.5 && distance > 2.5)
					{
						double perp_distance = DBL_MAX;
						if((int)pL->points.size() > 2 && map.roadSegments.at(rs_2).Lanes.at(i2).points.size()>2)
						{
							RelativeInfo info;
							PlanningHelpers::GetRelativeInfo(pL->points, closest_p, info);
							perp_distance = info.perp_distance;
							//perp_distance = PlanningHelpers::GetPerpDistanceToVectorSimple(pL->points.at(iCenter1-1), pL->points.at(iCenter1+1), closest_p);
						}

						if(perp_distance > 1.0 && perp_distance < 5.0)
						{
							pL->pRightLane = &map.roadSegments.at(rs_2).Lanes.at(i2);
							for(unsigned int i_internal = 0; i_internal< pL->points.size(); i_internal++)
							{
								if(i_internal<map.roadSegments.at(rs_2).Lanes.at(i2).points.size())
								{
									pL->points.at(i_internal).RightPointId = map.roadSegments.at(rs_2).Lanes.at(i2).id;
									pL->points.at(i_internal).pRight = &map.roadSegments.at(rs_2).Lanes.at(i2).points.at(i_internal);
//									map.roadSegments.at(rs_2).Lanes.at(i2).points.at(i_internal).pLeft = &pL->points.at(i_internal);
								}
							}
						}
						else if(perp_distance < -1.0 && perp_distance > -5.0)
						{
							pL->pLeftLane = &map.roadSegments.at(rs_2).Lanes.at(i2);
							for(unsigned int i_internal = 0; i_internal< pL->points.size(); i_internal++)
							{
								if(i_internal<map.roadSegments.at(rs_2).Lanes.at(i2).points.size())
								{
									pL->points.at(i_internal).LeftPointId = map.roadSegments.at(rs_2).Lanes.at(i2).id;
									pL->points.at(i_internal).pLeft = &map.roadSegments.at(rs_2).Lanes.at(i2).points.at(i_internal);
//									map.roadSegments.at(rs_2).Lanes.at(i2).points.at(i_internal).pRight = &pL->points.at(i_internal);
								}
							}
						}
					}
				}
			}
		}
	}
}

void MappingHelpers::ConnectBoundariesToWayPoints(RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);

				for(unsigned int ib=0; ib < map.boundaries.size(); ib++)
				{
					if(map.boundaries.at(ib).points.size() > 0)
					{
						Boundary* pB = &map.boundaries.at(ib);
						//calculate center of the boundary
						GPSPoint sum_p;
						for(unsigned int i=0; i < pB->points.size(); i++)
						{
							sum_p.x += pB->points.at(i).pos.x;
							sum_p.y += pB->points.at(i).pos.y;
							sum_p.z += pB->points.at(i).pos.z;
						}

						pB->center.pos.x = sum_p.x / (double)pB->points.size();
						pB->center.pos.y = sum_p.y / (double)pB->points.size();
						pB->center.pos.z = sum_p.z / (double)pB->points.size();

						//add boundary ID to proper waypoint
						double d_to_center = hypot(pWP->pos.y - pB->center.pos.y, pWP->pos.x - pB->center.pos.x);
						if(d_to_center < 100)
						{
							if(PlanningHelpers::PointInsidePolygon(pB->points, *pWP) == 1)
							{
								pWP->boundaryId = pB->id;
							}
						}

					}
				}
			}
		}
	}
}

void MappingHelpers::LinkBoundariesToWayPoints(RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);

				for(unsigned int ib=0; ib < map.boundaries.size(); ib++)
				{
					if(map.boundaries.at(ib).points.size() > 0)
					{
						Boundary* pB = &map.boundaries.at(ib);
						if(pWP->boundaryId == pB->id)
						{
							pWP->pBoundary = pB;
						}
					}
				}
			}
		}
	}
}

void MappingHelpers::LinkMissingBranchingWayPoints(RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);
				for(unsigned int j = 0 ; j < pWP->toIds.size(); j++)
				{
					pWP->pFronts.push_back(FindWaypoint(pWP->toIds.at(j), map));
				}
			}
		}
	}
}

void MappingHelpers::LinkLanesPointers(RoadNetwork& map)
{
	for(auto& rs: map.roadSegments)
	{
		//Clear first
		for(auto& l: rs.Lanes)
		{
			l.toLanes.clear();
			l.fromLanes.clear();
		}
	}

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		//Link Lanes
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			pL->pRoad = &map.roadSegments.at(rs);
			for(unsigned int j = 0 ; j < pL->fromIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->fromIds.at(j))
					{
						pL->fromLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->toIds.size(); j++)
			{
				for(unsigned int l= 0; l < map.roadSegments.at(rs).Lanes.size(); l++)
				{
					if(map.roadSegments.at(rs).Lanes.at(l).id == pL->toIds.at(j))
					{
						pL->toLanes.push_back(&map.roadSegments.at(rs).Lanes.at(l));
					}
				}
			}

			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
				pL->points.at(j).pLane  = pL;
			}
		}
	}
}

void MappingHelpers::LinkMissingBranchingWayPointsV2(RoadNetwork& map)
{
	for(auto& rs: map.roadSegments)
	{
		//Clear first
		for(auto& l: rs.Lanes)
		{
			for(auto& wp: l.points)
			{
				wp.pFronts.clear();
			}
		}
	}

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pLane = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int p= 0; p < pLane->points.size(); p++)
			{
				WayPoint* pWP = &pLane->points.at(p);

				if(p+1 == pLane->points.size()) // Last Point in Lane
				{
					for(unsigned int j = 0 ; j < pLane->toLanes.size(); j++)
					{
						pWP->pFronts.push_back(&pLane->toLanes.at(j)->points.at(0));
					}
				}
				else
				{
					if(pWP->toIds.size() > 1)
					{
						cout << "Error Error Erro ! Lane: " << pWP->laneId << ", Point: " << pWP->originalMapID << endl;
					}
					else
					{
						pWP->pFronts.push_back(&pLane->points.at(p+1));
					}
				}
			}
		}
	}
}

void MappingHelpers::LinkLaneChangeWaypointsPointers(RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);
				if(pWP->pLeft == 0 && pWP->LeftPointId > 0)
				{
					pWP->pLeft = MappingHelpers::FindWaypointV2(pWP->LeftPointId, pWP->laneId, map);

					if(pWP->pLeft != nullptr)
					{
						pWP->LeftLnId = pWP->pLeft->laneId;
						pWP->pLane->pLeftLane = pWP->pLeft->pLane;
						pWP->pLane->lane_change = 1;

						if(pWP->pLeft->RightPointId == pWP->id)
						{
							pWP->pLeft->pRight = pWP;
							pWP->pLeft->RightLnId = pWP->laneId;
							pWP->pLeft->pLane->pRightLane = pWP->pLane;
						}
					}
				}

				if(pWP->pRight == 0 && pWP->RightPointId > 0)
				{
					pWP->pRight = MappingHelpers::FindWaypointV2(pWP->RightPointId, pWP->laneId, map);

					if(pWP->pRight != nullptr)
					{
						pWP->RightLnId = pWP->pRight->laneId;
						pWP->pLane->pRightLane = pWP->pRight->pLane;
						pWP->pLane->lane_change = 1;

						if(pWP->pRight->LeftPointId == pWP->id)
						{
							pWP->pRight->pLeft = pWP;
							pWP->pRight->LeftLnId = pWP->laneId;
							pWP->pRight->pLane->pLeftLane = pWP->pLane;
						}
					}
				}
			}
		}
	}
}

void MappingHelpers::LinkTrafficLightsAndStopLines(RoadNetwork& map)
{

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				for(unsigned int isl = 0; isl < map.stopLines.size(); isl++)
				{
					WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);

					if(map.stopLines.at(isl).linkID == pWP->id)
					{
						map.stopLines.at(isl).laneId = pWP->laneId;
						map.stopLines.at(isl).pLane = pWP->pLane;
						map.roadSegments.at(rs).Lanes.at(i).stopLines.push_back(map.stopLines.at(isl));

						pWP->stopLineID = map.stopLines.at(isl).id;

						for(unsigned int itl = 0; itl < map.trafficLights.size(); itl++)
						{
							bool bFound = false;
							for(unsigned int stli = 0 ; stli < map.stopLines.at(isl).lightIds.size(); stli++)
							{
								if(map.trafficLights.at(itl).id == map.stopLines.at(isl).lightIds.at(stli))
								{
									bFound = true;
									break;
								}
							}

							if(bFound)
							{
								map.trafficLights.at(itl).laneIds.push_back(pWP->laneId);
								map.trafficLights.at(itl).pLanes.push_back(pWP->pLane);
							}
						}
						break;
					}
				}
			}
		}
	}

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int itl = 0; itl < map.trafficLights.size(); itl++)
			{
				for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
				{
					WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);
					if(map.trafficLights.at(itl).linkID == pWP->id)
					{
						map.roadSegments.at(rs).Lanes.at(i).trafficlights.push_back(map.trafficLights.at(itl));
						break;
					}
				}
			}
		}
	}
}

void MappingHelpers::UpdateMapWithSignalPose(const std::vector<WayPoint>& points, RoadNetwork& map, std::vector<WayPoint*>& updated_list, const double& min_affect_radius, const double& hit_cost)
{
	if(map.roadSegments.size() == 0 || points.size() == 0) return;

	for(auto& p: points)
	{
		for(auto& l: map.roadSegments.at(0).Lanes)
		{
			RelativeInfo inf;
			int dummy_index = 0;
			PlanningHelpers::GetRelativeInfoLimited(l.points, p, inf, dummy_index);

			if(!inf.bAfter && !inf.bBefore && fabs(inf.perp_distance) < min_affect_radius)
			{
				if(hit_cost < 0)
				{
					l.points.at(inf.iBack).pFronts.clear();
					l.points.at(inf.iFront).pBacks.clear();
				}
				else
				{
					l.points.at(inf.iBack).actionCost.push_back(std::make_pair(CHANGE_DESTINATION, hit_cost));
					updated_list.push_back(&l.points.at(inf.iBack));
					l.points.at(inf.iFront).actionCost.push_back(std::make_pair(CHANGE_DESTINATION, hit_cost));
					updated_list.push_back(&l.points.at(inf.iFront));
				}
				break;
			}
		}
	}
}

void MappingHelpers::UpdateMapWithOccupancyGrid(OccupancyToGridMap& map_info, const std::vector<int>& data, RoadNetwork& map, std::vector<WayPoint*>& updated_list)
{
	Mat3 rotationMat(- map_info.center.pos.a);
	Mat3 translationMat(-map_info.center.pos.x, -map_info.center.pos.y);
	updated_list.clear();

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);

				GPSPoint relative_point = pWP->pos;
				relative_point = translationMat * relative_point;
				relative_point = rotationMat *relative_point;

				int cell_value = 0;
				if(map_info.GetCellIndexFromPoint(relative_point, data, cell_value) == true)
				{
					if(cell_value == 0)
					{
						bool bFound = false;
						for(unsigned int i_action=0; i_action < pWP->actionCost.size(); i_action++)
						{
							if(pWP->actionCost.at(i_action).first == FORWARD_ACTION)
							{
								pWP->actionCost.at(i_action).second = 100;
								bFound = true;
							}
						}

						if(!bFound)
							pWP->actionCost.push_back(make_pair(FORWARD_ACTION, 100));

						updated_list.push_back(pWP);
					}
				}
			}
		}
	}
}

void MappingHelpers::FixRedundantPointsLanes(std::vector<Lane>& lanes)
{
	//Fix Redundant point for two points in a row
	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		for(int ip = 1; ip < (int)lanes.at(il).points.size(); ip++)
		{
			WayPoint* p1 = &lanes.at(il).points.at(ip-1);
			WayPoint* p2 = &lanes.at(il).points.at(ip);
			WayPoint* p3 = nullptr;
			if(ip+1 < (int)lanes.at(il).points.size())
				p3 = &lanes.at(il).points.at(ip+1);

			double d = hypot(p2->pos.y-p1->pos.y, p2->pos.x-p1->pos.x);
			if(d == 0)
			{
				p1->toIds = p2->toIds;
				p1->originalMapID = p2->originalMapID;
				if(p3 != nullptr)
					p3->fromIds = p2->fromIds;

				lanes.at(il).points.erase(lanes.at(il).points.begin()+ip);
				ip--;

				if(DEBUG_MAP_PARSING)
					cout << "Fixed Redundant Points for Lane:" << lanes.at(il).id << ", Current: " << ip << ", Size: " << lanes.at(il).points.size() << endl;
			}
		}
	}
}

void MappingHelpers::FixTwoPointsLane(Lane& l)
{
	if(l.points.size() == 2)
	{
		RoadNetwork::g_max_point_id++;
		WayPoint wp = l.points.at(0);
		wp.id = RoadNetwork::g_max_point_id;
		wp.fromIds.clear();
		wp.fromIds.push_back(l.points.at(0).id);
		wp.toIds.clear();
		wp.toIds.push_back(l.points.at(1).id);

		l.points.at(0).toIds.clear();
		l.points.at(0).toIds.push_back(wp.id);

		l.points.at(1).fromIds.clear();
		l.points.at(1).fromIds.push_back(wp.id);

		wp.pos.x = (l.points.at(0).pos.x + l.points.at(1).pos.x)/2.0;
		wp.pos.y = (l.points.at(0).pos.y + l.points.at(1).pos.y)/2.0;
		wp.pos.z = (l.points.at(0).pos.z + l.points.at(1).pos.z)/2.0;

		l.points.insert(l.points.begin()+1, wp);
	}
	else if(l.points.size() < 2)
	{
		cout << "## WOW Lane " <<  l.id << " With Size (" << l.points.size() << ") " << endl;
	}
}

void MappingHelpers::FixTwoPointsLanes(std::vector<Lane>& lanes)
{
	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		for(unsigned int ip = 0; ip < lanes.at(il).points.size(); ip++)
		{
			if(lanes.at(il).points.at(ip).id > RoadNetwork::g_max_point_id)
			{
				RoadNetwork::g_max_point_id = lanes.at(il).points.at(ip).id;
			}
		}
	}

	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		FixTwoPointsLane(lanes.at(il));
		PlanningHelpers::CalcAngleAndCost(lanes.at(il).points);
	}
}

void MappingHelpers::InsertWayPointToBackOfLane(const WayPoint& wp, Lane& lane)
{
	if(lane.points.size() > 0)
	{
		WayPoint* pFirst = &lane.points.at(0);
		pFirst->pos = wp.pos;
	}

//	WayPoint f_wp = *pFirst;
//	f_wp.pos = wp.pos;
//
//	//Give old first new ID
//	global_id++;
//	pFirst->id = global_id;
//
//	//link ids
//	f_wp.toIds.clear(); //only see front
//	f_wp.toIds.push_back(pFirst->id);
//
//	pFirst->fromIds.clear();
//	pFirst->fromIds.push_back(f_wp.id);
//
//	if(lane.points.size() > 1)
//	{
//		lane.points.at(1).fromIds.clear();
//		lane.points.at(1).fromIds.push_back(pFirst->id);
//	}
//
//	lane.points.insert(lane.points.begin(), f_wp);
}

void MappingHelpers::InsertWayPointToFrontOfLane(const WayPoint& wp, Lane& lane)
{
	if(lane.points.size() > 0)
	{
		WayPoint* pLast = &lane.points.at(lane.points.size()-1);
		pLast->pos = wp.pos;
	}

//	WayPoint l_wp = *pLast;
//	l_wp.pos = wp.pos;
//
//	//Give old first new ID
//	global_id++;
//	pLast->id = global_id;
//
//	//link ids
//	l_wp.fromIds.clear(); //only see front
//	l_wp.fromIds.push_back(pLast->id);
//
//	pLast->toIds.clear();
//	pLast->toIds.push_back(l_wp.id);
//
//	if(lane.points.size() > 1)
//	{
//		lane.points.at(lane.points.size()-2).toIds.clear();
//		lane.points.at(lane.points.size()-2).toIds.push_back(pLast->id);
//	}
//
//	lane.points.push_back(l_wp);
}

void MappingHelpers::StitchLanes(std::vector<Lane>& lanes, const double& min_stitching_distance, const double& max_stitching_distance)
{
	for(auto& l: lanes)
	{
		if(l.toLanes.size() == 1)
		{
			if(l.points.size() > 1 && l.toLanes.at(0)->points.size() > 1)
			{
				WayPoint* pWP1 = &l.points.at(l.points.size()-1);
				WayPoint* pWP2 = &l.toLanes.at(0)->points.at(0);
				double d = hypot(pWP2->pos.y - pWP1->pos.y, pWP2->pos.x - pWP1->pos.x);
				if(d > min_stitching_distance && d < max_stitching_distance)
				{
					pWP1->pos = pWP2->pos;
				}
			}
		}
	}
}

void MappingHelpers::StitchLanes(RoadNetwork& map, const double& min_stitching_distance, const double& max_stitching_distance)
{
	if(map.roadSegments.size() == 0) return;

	for(auto& l: map.roadSegments.at(0).Lanes)
	{
		if(l.toLanes.size() > 0)
		{
			if(l.points.size() > 1 && l.toLanes.at(0)->points.size() > 1)
			{
				WayPoint* pWP1 = &l.points.at(l.points.size()-1);
				WayPoint* pWP2 = &l.toLanes.at(0)->points.at(0);
				RelativeInfo inf;
				int temp_index = 0;
				PlanningHelpers::GetRelativeInfoLimited(l.toLanes.at(0)->points, *pWP1, inf, temp_index);
				double d = hypot(pWP2->pos.y - pWP1->pos.y, pWP2->pos.x - pWP1->pos.x);

				//Create new waypoint
				if(inf.bBefore && d > min_stitching_distance && d < max_stitching_distance)
				{
					map.g_max_point_id++;
					WayPoint final_wp = *pWP1;
					final_wp.id = map.g_max_point_id;
					final_wp.pos = pWP2->pos;

					final_wp.fromIds.clear();
					final_wp.fromIds.push_back(pWP1->id);

					pWP1->toIds.clear();
					pWP1->toIds.push_back(final_wp.id);

					l.points.push_back(final_wp);
				}
			}
		}
	}
}

void MappingHelpers::FixUnconnectedLanes(std::vector<Lane>& lanes, const int& max_angle_diff)
{
	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		PlanningHelpers::CalcAngleAndCost(lanes.at(il).points);
	}

	std::vector<Lane> sp_lanes = lanes;
	// bool bAtleastOneChange = false;
	//Find before lanes
	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		if(lanes.at(il).fromIds.size() == 0)
		{
			double closest_d = DBL_MAX;
			Lane* pL = nullptr;
			Lane* pFL = nullptr;
			for(int l=0; l < (int)sp_lanes.size(); l ++)
			{
				if(lanes.at(il).id == sp_lanes.at(l).id)
				{
					pFL = &sp_lanes.at(l);
					break;
				}
			}

			RelativeInfo closest_info;
			int closest_index = -1;
			for(int l=0; l < (int)sp_lanes.size(); l ++)
			{
				if(pFL->id != sp_lanes.at(l).id)
				{
					RelativeInfo info;
					WayPoint lastofother = sp_lanes.at(l).points.at(sp_lanes.at(l).points.size()-1);
					PlanningHelpers::GetRelativeInfoLimited(sp_lanes.at(l).points, pFL->points.at(0), info, 0);
					double back_distance = hypot(lastofother.pos.y - pFL->points.at(0).pos.y, lastofother.pos.x - pFL->points.at(0).pos.x);
					bool bCloseFromBack = false;
					if((info.bAfter == true && back_distance < 15.0) || info.bAfter == false)
						bCloseFromBack = true;


					if(fabs(info.perp_distance) < 2 && fabs(info.perp_distance) < closest_d && fabs(info.angle_diff) < max_angle_diff && info.bBefore == false && bCloseFromBack)
					{
						closest_d = fabs(info.perp_distance);
						pL = &sp_lanes.at(l);
						closest_info = info;
						closest_index = l;
					}
				}
			}

			if(pL != nullptr && pFL != nullptr)
			{
				if(closest_info.iFront == (int)pL->points.size()-1)
				{
					pL->toIds.push_back(pFL->id);
					pL->points.at(closest_info.iFront).toIds.push_back(pFL->points.at(0).id);

					pFL->points.at(0).fromIds.push_back(pL->points.at(closest_info.iFront).id);
					pFL->fromIds.push_back(pL->id);
					// bAtleastOneChange = true;
					if(DEBUG_MAP_PARSING)
					{
						cout << "Closest Next Lane For: " << pFL->id << " , Is:" << pL->id << ", Distance=" << fabs(closest_info.perp_distance) << ", Size: " << pL->points.size() << ", back_index: " << closest_info.iBack <<", front_index: " << closest_info.iFront << ", Direct: " << closest_info.angle_diff << endl;
						cout << "Don't Split , Perfect !" << endl;
					}
				}
				else
				{
					 // split from previous point
					if(DEBUG_MAP_PARSING)
						cout << "Closest Next Lane For: " << pFL->id << " , Is:" << pL->id << ", Distance=" << fabs(closest_info.perp_distance) << ", Size: " << pL->points.size() << ", back_index: " << closest_info.iBack <<", front_index: " << closest_info.iFront << ", Direct: " << closest_info.angle_diff << endl;

					Lane front_half, back_half;
					front_half.points.insert(front_half.points.begin(), pL->points.begin()+closest_info.iFront, pL->points.end());
					front_half.toIds = pL->toIds;
					front_half.fromIds.push_back(pL->id);
					front_half.id = front_half.points.at(0).originalMapID;
					front_half.areaId = pL->areaId;
					front_half.num = pL->num;
					front_half.roadId = pL->roadId;
					front_half.speed = pL->speed;
					front_half.type = pL->type;
					front_half.width = pL->width;

					back_half.points.insert(back_half.points.begin(), pL->points.begin(), pL->points.begin()+closest_info.iFront);
					back_half.toIds.clear();
					back_half.toIds.push_back(front_half.id);
					back_half.toIds.push_back(pFL->id);
					back_half.fromIds = pL->fromIds;
					back_half.id = pL->id;
					back_half.areaId = pL->areaId;
					back_half.num = pL->num;
					back_half.roadId = pL->roadId;
					back_half.speed = pL->speed;
					back_half.type = pL->type;
					back_half.width = pL->width;

					WayPoint* last_from_back =  &back_half.points.at(back_half.points.size()-1);
					WayPoint* first_from_front =  &pFL->points.at(0);

					last_from_back->toIds.push_back(first_from_front->id);
					first_from_front->fromIds.push_back(last_from_back->id);

					if(front_half.points.size() > 1 && back_half.points.size() > 1)
					{
						if(DEBUG_MAP_PARSING)
							cout << "Split this one Nicely! first_half_size: " << front_half.points.size() << ", second_hald_size: " << back_half.points.size() << endl;

						pFL->fromIds.push_back(back_half.id);

						if(closest_index >= 0)
							sp_lanes.erase(sp_lanes.begin()+closest_index);
						else
							cout << "## Alert Alert Alert !!!! " << endl;

						// add perp point to lane points
						InsertWayPointToBackOfLane(closest_info.perp_point, front_half);
						InsertWayPointToFrontOfLane(closest_info.perp_point, back_half);

						sp_lanes.push_back(front_half);
						sp_lanes.push_back(back_half);
						// bAtleastOneChange = true;
					}
					else
					{
						if(DEBUG_MAP_PARSING)
							cout << "=> Can't Split this one :( !" << endl;
					}
				}
			}
			else
			{
				if(DEBUG_MAP_PARSING)
					cout << "=> Can't find Before Lanes For:  " << lanes.at(il).id  << endl;
			}
		}
	}

	lanes = sp_lanes;

	//Find to lanes

	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		if(lanes.at(il).toIds.size() == 0)
		{
			double closest_d = DBL_MAX;
			Lane* pL = nullptr;
			Lane* pBL = nullptr;
			for(int l=0; l < (int)sp_lanes.size(); l ++)
			{
				if(lanes.at(il).id == sp_lanes.at(l).id)
				{
					pBL = &sp_lanes.at(l);
					break;
				}
			}

			RelativeInfo closest_info;
			int closest_index = -1;
			for(int l=0; l < (int)sp_lanes.size(); l ++)
			{
				if(pBL->id != sp_lanes.at(l).id )
				{
					RelativeInfo info;
					WayPoint firstofother = sp_lanes.at(l).points.at(0);
					WayPoint last_point = pBL->points.at(pBL->points.size()-1);
					PlanningHelpers::GetRelativeInfoLimited(sp_lanes.at(l).points, last_point, info, 0);
					double front_distance = hypot(firstofother.pos.y - last_point.pos.y, firstofother.pos.x - last_point.pos.x);
					bool bCloseFromFront = false;
					if((info.bBefore == true && front_distance < 15.0) || info.bBefore == false)
						bCloseFromFront = true;

					if(fabs(info.perp_distance) < 2 && fabs(info.perp_distance) < closest_d && fabs(info.angle_diff) < max_angle_diff && info.bAfter == false && bCloseFromFront)
					{
						closest_d = fabs(info.perp_distance);
						pL = &sp_lanes.at(l);
						closest_info = info;
						closest_index = l;
					}
				}
			}

			if(pL != nullptr && pBL != nullptr)
			{
				if(closest_info.iFront == 0)
				{
					pL->fromIds.push_back(pBL->id);
					pL->points.at(closest_info.iFront).fromIds.push_back(pBL->points.at(pBL->points.size()-1).id);

					pBL->points.at(pBL->points.size()-1).toIds.push_back(pL->points.at(closest_info.iFront).id);
					pBL->toIds.push_back(pL->id);

					// bAtleastOneChange = true;
					if(DEBUG_MAP_PARSING)
					{
						cout << "Closest Back Lane For: " << pBL->id << " , Is:" << pL->id << ", Distance=" << fabs(closest_info.perp_distance) << ", Size: " << pL->points.size() << ", back_index: " << closest_info.iBack <<", front_index: " << closest_info.iFront << ", Direct: " << closest_info.angle_diff << endl;
						cout << "Don't Split , Perfect !" << endl;
					}
				}
				else
				{
					 // split from previous point
					if(DEBUG_MAP_PARSING)
						cout << "Closest Back Lane For: " << pBL->id << " , Is:" << pL->id << ", Distance=" << fabs(closest_info.perp_distance) << ", Size: " << pL->points.size() << ", back_index: " << closest_info.iBack <<", front_index: " << closest_info.iFront << ", Direct: " << closest_info.angle_diff << endl;

					Lane front_half, back_half;
					front_half.points.insert(front_half.points.begin(), pL->points.begin()+closest_info.iFront, pL->points.end());
					front_half.toIds = pL->toIds;
					front_half.fromIds.push_back(pL->id);
					front_half.fromIds.push_back(pBL->id);
					front_half.id = front_half.points.at(0).originalMapID;
					front_half.areaId = pL->areaId;
					front_half.num = pL->num;
					front_half.roadId = pL->roadId;
					front_half.speed = pL->speed;
					front_half.type = pL->type;
					front_half.width = pL->width;

					back_half.points.insert(back_half.points.begin(), pL->points.begin(), pL->points.begin()+closest_info.iFront);
					back_half.toIds.push_back(front_half.id);
					back_half.fromIds = pL->fromIds;
					back_half.id = pL->id;
					back_half.areaId = pL->areaId;
					back_half.num = pL->num;
					back_half.roadId = pL->roadId;
					back_half.speed = pL->speed;
					back_half.type = pL->type;
					back_half.width = pL->width;

					WayPoint* first_from_next =  &front_half.points.at(0);
					WayPoint* last_from_front =  &pBL->points.at(pBL->points.size()-1);

					first_from_next->fromIds.push_back(last_from_front->id);
					last_from_front->toIds.push_back(first_from_next->id);

					if(front_half.points.size() > 1 && back_half.points.size() > 1)
					{
						if(DEBUG_MAP_PARSING)
							cout << "Split this one Nicely! first_half_size: " << front_half.points.size() << ", second_hald_size: " << back_half.points.size() << endl;

						pBL->toIds.push_back(front_half.id);

						if(closest_index >= 0)
							sp_lanes.erase(sp_lanes.begin()+closest_index);
						else
							cout << "## Alert Alert Alert !!!! " << endl;

						// add perp point to lane points
						InsertWayPointToBackOfLane(closest_info.perp_point, front_half);
						InsertWayPointToFrontOfLane(closest_info.perp_point, back_half);

						sp_lanes.push_back(front_half);
						sp_lanes.push_back(back_half);
						// bAtleastOneChange = true;
					}
					else
					{
						if(DEBUG_MAP_PARSING)
							cout << "=> Can't Split this one :( !" << endl;
					}
				}
			}
			else
			{
				if(DEBUG_MAP_PARSING)
					cout << "=> Can't find After Lanes For:  " << lanes.at(il).id  << endl;
			}
		}
	}

	lanes = sp_lanes;
}

void MappingHelpers::LinkTrafficLightsAndStopLinesV2(RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
			{
				WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);

				for(unsigned int isl = 0; isl < map.stopLines.size(); isl++)
				{
					if(map.stopLines.at(isl).linkID == pWP->originalMapID)
					{
						map.stopLines.at(isl).laneId = pWP->laneId;
						map.stopLines.at(isl).pLane = pWP->pLane;
						map.roadSegments.at(rs).Lanes.at(i).stopLines.push_back(map.stopLines.at(isl));

						pWP->stopLineID = map.stopLines.at(isl).id;

						for(unsigned int itl = 0; itl < map.trafficLights.size(); itl++)
						{
							bool bFound = false;
							for(unsigned int stli = 0 ; stli < map.stopLines.at(isl).lightIds.size(); stli++)
							{
								if(map.trafficLights.at(itl).id == map.stopLines.at(isl).lightIds.at(stli))
								{
									bFound = true;
									break;
								}
							}

							if(bFound)
							{
								map.trafficLights.at(itl).laneIds.push_back(pWP->laneId);
								map.trafficLights.at(itl).pLanes.push_back(pWP->pLane);

								for(auto& tl_item:map.trafficLights)
								{
									if(tl_item.id != map.trafficLights.at(itl).id && tl_item.groupID == map.trafficLights.at(itl).groupID)
									{
										tl_item.laneIds.push_back(pWP->laneId);
										tl_item.pLanes.push_back(pWP->pLane);
									}
								}
							}
						}
						//break; only one traffic light was connected when this break was activated
					}
				}
			}
		}
	}

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			for(unsigned int itl = 0; itl < map.trafficLights.size(); itl++)
			{
				for(unsigned int p= 0; p < map.roadSegments.at(rs).Lanes.at(i).points.size(); p++)
				{
					WayPoint* pWP = &map.roadSegments.at(rs).Lanes.at(i).points.at(p);
					if(map.trafficLights.at(itl).linkID == pWP->originalMapID)
					{
						map.roadSegments.at(rs).Lanes.at(i).trafficlights.push_back(map.trafficLights.at(itl));
						break;
					}
				}
			}
		}
	}
}

void MappingHelpers::FindAdjacentSingleLane(RoadNetwork& map, const int& lane_id, const int& dir,  const double& min_d, const double& max_d)
{
	Lane* pL =  GetLaneById(lane_id, map);
	if(pL != nullptr)
	{
		for(auto& l: map.roadSegments.at(0).Lanes)
		{
			PlanningHelpers::CalcAngleAndCost(l.points);
		}

		for(auto& l2: map.roadSegments.at(0).Lanes)
		{
			if(pL->id != l2.id)
			{
				for(auto& wp1: pL->points)
				{
					RelativeInfo info;
					PlanningHelpers::GetRelativeInfoLimited(l2.points, wp1, info);
					double angle_diff = Angle::AngleBetweenTwoAnglesPositive(info.perp_point.pos.a, wp1.pos.a)*RAD2DEG;
					if(fabs(info.perp_distance) > min_d && fabs(info.perp_distance) < max_d && !info.bAfter && !info.bBefore && angle_diff < 10)
					{
						WayPoint* wp2 = &l2.points.at(info.iFront);
						if(info.perp_distance < 0 && (dir == 0 || dir == 2))
						{
							pL->lane_change = 1;
							l2.lane_change = 1;

							wp1.pRight = wp2;
							wp1.RightPointId = wp2->id;
							wp1.RightLnId = l2.id;
							pL->pRightLane = &l2;

							wp2->pLeft = &wp1;
							wp2->LeftPointId = wp1.id;
							wp2->LeftLnId = pL->id;
							l2.pLeftLane = pL;
						}
						else if(info.perp_distance >= 0 && (dir == 0 || dir == 1))
						{
							pL->lane_change = 1;
							l2.lane_change = 1;

							wp1.pLeft = wp2;
							wp1.LeftPointId = wp2->id;
							wp1.LeftLnId = l2.id;
							pL->pLeftLane = &l2;

							wp2->pRight = &wp1;
							wp2->RightPointId = wp1.id;
							wp2->RightLnId = pL->id;
							l2.pRightLane = pL;
						}
					}
				}
			}
		}
	}
}

void MappingHelpers::FindAdjacentLanesV2(RoadNetwork& map, const double& min_d, const double& max_d )
{
	if(map.roadSegments.size() == 0) return;
	//Fix The angle for all lane's waypoints

	for(auto& l: map.roadSegments.at(0).Lanes)
	{
		PlanningHelpers::CalcAngleAndCost(l.points);
	}

	for(auto& l1: map.roadSegments.at(0).Lanes)
	{
		for(auto& l2: map.roadSegments.at(0).Lanes)
		{
			if(l1.id != l2.id)
			{
				for(auto& wp1: l1.points)
				{
					RelativeInfo info;
					PlanningHelpers::GetRelativeInfoLimited(l2.points, wp1, info);

					double angle_diff = Angle::AngleBetweenTwoAnglesPositive(info.perp_point.pos.a, wp1.pos.a)*RAD2DEG;
					if(fabs(info.perp_distance) > min_d && fabs(info.perp_distance) < max_d && !info.bAfter && !info.bBefore && angle_diff < 10)
					{
						l1.lane_change = 1;
						l2.lane_change = 1;

						WayPoint* wp2 = &l2.points.at(info.iFront);

						if(info.perp_distance < 0)
						{
							wp1.pRight = wp2;
							wp1.RightPointId = wp2->id;
							wp1.RightLnId = l2.id;
							l1.pRightLane = &l2;

							wp2->pLeft = &wp1;
							wp2->LeftPointId = wp1.id;
							wp2->LeftLnId = l1.id;
							l2.pLeftLane = &l1;
						}
						else
						{
							wp1.pLeft = wp2;
							wp1.LeftPointId = wp2->id;
							wp1.LeftLnId = l2.id;
							l1.pLeftLane = &l2;

							wp2->pRight = &wp1;
							wp2->RightPointId = wp1.id;
							wp2->RightLnId = l1.id;
							l2.pRightLane = &l1;
						}
					}
				}
			}
		}
	}
}

void MappingHelpers::GetMapMaxIds(RoadNetwork& map)
{
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			if(pL->id > RoadNetwork::g_max_lane_id)
				RoadNetwork::g_max_lane_id = pL->id;

			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
				if(pL->points.at(j).id > RoadNetwork::g_max_point_id)
				{
					RoadNetwork::g_max_point_id = pL->points.at(j).id;
				}
			}
		}
	}

	for(unsigned int i=0; i < map.stopLines.size(); i++)
	{
		if(map.stopLines.at(i).id > RoadNetwork::g_max_stop_line_id)
			RoadNetwork::g_max_stop_line_id = map.stopLines.at(i).id;
	}

	for(unsigned int i=0; i < map.trafficLights.size(); i++)
	{
		if(map.trafficLights.at(i).id > RoadNetwork::g_max_traffic_light_id)
			RoadNetwork::g_max_traffic_light_id = map.trafficLights.at(i).id;
	}

	for(unsigned int i=0; i < map.boundaries.size(); i++)
	{
		if(map.boundaries.at(i).id > RoadNetwork::g_max_boundary_area_id)
			RoadNetwork::g_max_boundary_area_id = map.boundaries.at(i).id;
	}

	for(unsigned int i=0; i < map.lines.size(); i++)
	{
		if(map.lines.at(i).id > RoadNetwork::g_max_line_id)
			RoadNetwork::g_max_line_id = map.lines.at(i).id;
	}

	for(unsigned int i=0; i < map.markings.size(); i++)
	{
		if(map.markings.at(i).id > RoadNetwork::g_max_marking_id)
			RoadNetwork::g_max_marking_id = map.markings.at(i).id;
	}

	for(unsigned int i=0; i < map.curbs.size(); i++)
	{
		if(map.curbs.at(i).id > RoadNetwork::g_max_curb_id)
			RoadNetwork::g_max_curb_id = map.curbs.at(i).id;
	}

	for(unsigned int i=0; i < map.crossings.size(); i++)
	{
		if(map.crossings.at(i).id > RoadNetwork::g_max_crossing_id)
			RoadNetwork::g_max_crossing_id = map.crossings.at(i).id;
	}
}

bool MappingHelpers::IsPointExist(const WayPoint& p, const std::vector<WayPoint>& points)
{
	for(unsigned int ip = 0; ip < points.size(); ip++)
	{
		if(points.at(ip).id == p.id)
		{
			return true;
		}
	}
	return false;
}

 std::vector<Lane*> MappingHelpers::GetClosestMultipleLanesFromMap(const WayPoint& pos, RoadNetwork& map, const double& distance)
{
	vector<Lane*> lanesList;
	double d = 0;
	double a_diff = 0;
	for(unsigned int j=0; j< map.roadSegments.size(); j ++)
	{
		for(unsigned int k=0; k< map.roadSegments.at(j).Lanes.size(); k ++)
		{
			for(unsigned int pindex=0; pindex< map.roadSegments.at(j).Lanes.at(k).points.size(); pindex ++)
			{
				d = distance2points(map.roadSegments.at(j).Lanes.at(k).points.at(pindex).pos, pos.pos);
				a_diff = Angle::AngleBetweenTwoAnglesPositive(map.roadSegments.at(j).Lanes.at(k).points.at(pindex).pos.a, pos.pos.a);

				if(d <= distance && a_diff <= M_PI_4)
				{
					bool bLaneExist = false;
					for(unsigned int il = 0; il < lanesList.size(); il++)
					{
						if(lanesList.at(il)->id == map.roadSegments.at(j).Lanes.at(k).id)
						{
							bLaneExist = true;
							break;
						}
					}

					if(!bLaneExist)
						lanesList.push_back(&map.roadSegments.at(j).Lanes.at(k));

					break;
				}
			}
		}
	}

	return lanesList;
}

 void MappingHelpers::InsertUniqueId(std::vector<int>& id_list, int id)
 {
	 for(auto& x: id_list)
	 {
		 if(x == id)
		 {
			 return;
		 }
	 }

	 id_list.push_back(id);
 }

 void MappingHelpers::ConnectWayPoints(RoadNetwork& map)
 {
 	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
 	{
 		for(unsigned int i = 0; i < map.roadSegments.at(rs).Lanes.size(); i++)
 		{
 			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
 			for(int iwp= 0; iwp < (int)pL->points.size(); iwp++)
 			{
 				//make sure to IDs are connected correctly
 				if(iwp < (int)pL->points.size()-1) // Inside the lane
 				{
 					InsertUniqueId(pL->points.at(iwp).toIds, pL->points.at(iwp+1).id);
 				}
 				else // connect to the next lane
 				{
 					for(unsigned int k=0; k< pL->toLanes.size(); k++)
 					{
 						if(pL->toLanes.at(k) != nullptr && pL->toLanes.at(k)->points.size()>0)
 						{
 							InsertUniqueId(pL->points.at(iwp).toIds, pL->toLanes.at(k)->points.at(0).id);
 						}
 					}
 				}

 				//make sure from IDs are connected correctly
 				if(iwp > 0) // Inside the lane
 				{
 					InsertUniqueId(pL->points.at(iwp).fromIds, pL->points.at(iwp-1).id);
 				}
 				else // connect to the next lane
 				{
 					for(unsigned int k=0; k< pL->fromLanes.size(); k++)
 					{
 						if(pL->fromLanes.at(k) != nullptr && pL->fromLanes.at(k)->points.size()>0)
 						{
 							int final_point_index = pL->fromLanes.at(k)->points.size() - 1;
 							InsertUniqueId(pL->points.at(iwp).fromIds, pL->fromLanes.at(k)->points.at(final_point_index).id);
 						}
 					}
 				}
 			}
 		}
 	}
 }

 void MappingHelpers::ConnectLanes(RoadNetwork& map)
 {
	 if(map.roadSegments.size() == 0) return;

	 //Connect next and previous lanes
	for(auto& l : map.roadSegments.at(0).Lanes)
	{
		for(auto& next_lane_id : l.toIds)
		{
			Lane* pToLane = GetLaneById(next_lane_id, map);

			if(pToLane == nullptr)
			{
				std::cout << "Can't Find toLane: " << next_lane_id << " To connect. " << std::endl;
			}
			else
			{
				InsertUniqueId(pToLane->fromIds, l.id);

				//Now connect waypoints
				if(l.points.size() > 0 && pToLane->points.size() > 0)
				{
					WayPoint* p1 = &l.points.at(l.points.size()-1);
					WayPoint* p2 = &pToLane->points.at(0);

					InsertUniqueId(p1->toIds, p2->id);
					InsertUniqueId(p2->fromIds, p1->id);
				}

			}
		}
	}
 }

 void MappingHelpers::ConnectMissingStopLinesAndLanes(RoadNetwork& map)
 {
	 if(map.roadSegments.size() == 0) return;

	 for(auto& l: map.roadSegments.at(0).Lanes)
	 {
		 for(auto& sl: l.stopLines)
		 {
			 if(sl.points.size() > 0)
			 {
				 RelativeInfo inf;
				 int relative_index = 0;
				 PlanningHelpers::GetRelativeInfo(l.points, sl.points.at(0), inf, relative_index);
				 if(inf.iBack >= 0 && inf.iBack < (int)l.points.size())
				 {
					 sl.laneId = l.id;
					 l.points.at(inf.iBack).stopLineID = sl.id;
					 InsertUniqueStopLine(map.stopLines, sl);
				 }
				 else
				 {
					 std::cout << "Can't Assign Stop Line: " <<  sl.id << ", WayPoint Index: " << inf.iBack << ", iFront: " << inf.iFront << std::endl;
				 }
			 }
		 }

		 l.stopLines.clear();
	 }
 }

 void MappingHelpers::ConnectTrafficLightsAndStopLines(RoadNetwork& map)
 {
	 for(auto& tl: map.trafficLights)
	 {
		 for(auto& sl: map.stopLines)
		 {
			 if(sl.id == tl.stopLineID)
			 {
				 InsertUniqueId(sl.lightIds, tl.id);
				 break;
			 }
		 }
	 }

	 for(auto& sl: map.stopLines)
	 {
		 for(auto& id : sl.lightIds)
		 {
			 for(auto& tl: map.trafficLights)
			 {
				 if(tl.id == id && tl.stopLineID == 0)
				 {
					 tl.stopLineID = sl.id;
					 break;
				 }

			 }
		 }
	 }
 }

 void MappingHelpers::ConnectTrafficSignsAndStopLines(RoadNetwork& map)
{
	for(auto& sl: map.stopLines)
	{
		for(auto& ts: map.signs)
		{
			if(ts.id == sl.stopSignID || ts.groupID == sl.stopSignID)
			{
				ts.stopLineID = sl.id;
				break;
			}
		}
	}
}

void MappingHelpers::InsertUniqueStopLine(std::vector<StopLine>& stop_lines, const StopLine& sl)
{
	for(auto& x : stop_lines)
	{
		if(x.id == sl.id)
		{
			for(auto& id : sl.laneIds)
			{
				if(std::find(x.laneIds.begin(), x.laneIds.end(), id) == x.laneIds.end())
				{
					x.laneIds.push_back(id);
				}
			}

			for(auto& id : sl.lightIds)
			{
				if(std::find(x.lightIds.begin(), x.lightIds.end(), id) == x.lightIds.end())
				{
					x.lightIds.push_back(id);
				}
			}
			return;
		}
	}

	stop_lines.push_back(sl);
}

void MappingHelpers::InsertUniqueTrafficLight(std::vector<TrafficLight>& traffic_lights, const TrafficLight& tl)
{
	for(auto& x : traffic_lights)
	{
		if(x.id == tl.id)
		{
			for(auto& id : tl.laneIds)
			{
				if(std::find(x.laneIds.begin(), x.laneIds.end(), id) == x.laneIds.end())
				{
					x.laneIds.push_back(id);
				}
			}
			return;
		}
	}

	traffic_lights.push_back(tl);
}

void MappingHelpers::InsertUniqueTrafficSign(std::vector<TrafficSign>& traffic_signs, const TrafficSign& ts)
{
	for(auto& x : traffic_signs)
	{
		if(x.id == ts.id)
		{
			for(auto& id : ts.laneIds)
			{
				if(std::find(x.laneIds.begin(), x.laneIds.end(), id) == x.laneIds.end())
				{
					x.laneIds.push_back(id);
				}
			}
			return;
		}
	}

	traffic_signs.push_back(ts);
}

void MappingHelpers::GetClosestStopLines(const RoadNetwork& map, const WayPoint& p, const double& search_radius, std::vector<StopLine>& stop_lines)
{
	stop_lines.clear();
	for(auto sl: map.stopLines)
	{
		if(sl.points.size() > 0)
		{
			double d = hypot(sl.points.at(0).pos.y - p.pos.y, sl.points.at(0).pos.x - p.pos.x);
			if(d <= search_radius)
			{
				stop_lines.push_back(sl);
			}
		}
	}
}

void MappingHelpers::TrimPath(std::vector<WayPoint>& points, double trim_angle)
{
	if(points.size() < 3) return;
	std::vector<WayPoint> trimed_points;
	trimed_points.push_back(points.at(0));
	for(int i = 1 ; i < (int)points.size()-1; i++)
	{
		double p1_a = Angle::SplitPositiveAngle(points.at(i).pos.a);
		double p2_a = Angle::SplitPositiveAngle(points.at(i+1).pos.a);

		if(fabs(p2_a - p1_a) > trim_angle)
		{
			trimed_points.push_back(points.at(i));
		}
	}
	trimed_points.push_back(points.back());
	points = trimed_points;
}

void MappingHelpers::llaToxyz_proj(const std::string& proj_str, const WayPoint& origin, const double& lat,
		const double& lon, const double& alt, double& x_out, double& y_out, double& z_out)
{
	if(proj_str.size() < 8) return;

	projPJ pj_latlong, pj_utm;
	pj_latlong = pj_init_plus("+proj=latlong");
	pj_utm = pj_init_plus(proj_str.c_str());

	double _intern_lat = lat;
	double _intern_lon = lon;

	double _z = alt;
	double _x = DEG2RAD*_intern_lon;
	double _y = DEG2RAD*_intern_lat;

	if(pj_latlong != 0 && pj_utm !=0 )
	{
		pj_transform(pj_latlong, pj_utm, 1, 1, &_x, &_y, &_z);
		y_out = _y + origin.pos.y;
		x_out = _x + origin.pos.x;
		z_out = _z + origin.pos.z;
	}
	else
	{
		x_out = y_out = z_out = 0;
	}
}

void MappingHelpers::xyzTolla_proj(const std::string& proj_str, const WayPoint& origin, const double& x_in,
		const double& y_in, const double& z_in, double& lat, double& lon, double& alt)
{
	if(proj_str.size() < 8) return;

	projPJ pj_latlong, pj_utm;
	pj_latlong = pj_init_plus("+proj=latlong");
	pj_utm = pj_init_plus(proj_str.c_str());

	double _lon = x_in - origin.pos.x;
	double _lat = y_in - origin.pos.y;
	double _alt = z_in - origin.pos.z;

	if(pj_latlong != 0 && pj_utm !=0)
	{
		pj_transform(pj_utm,pj_latlong, 1, 1, &_lon, &_lat, &_alt);
		_lon = _lon * RAD2DEG;
		_lat = _lat * RAD2DEG;

		lon = _lon;
		lat = _lat;
		alt = _alt;
	}
	else
	{
		lon = lat = alt = 0;
	}
}

 void MappingHelpers::correct_gps_coor(double& lat,double& lon)
 {
 	double part1 = floor(lat);
 	double part2 = floor((lat-part1)*100.0)/60.0;
 	double part_frac = (lat*100.0) - (int)(lat*100);
 	double part3 = part_frac*100.0 / 3600.0;

 	lat = part1+part2+part3;

 	part1 = floor(lon);
 	part2 = floor((lon - part1)*100.0)/60.0;
 	part_frac = (lon*100.0) - (int)(lon*100);
 	part3 = part_frac*100.0 / 3600.0;
 	lon = part1+part2+part3;
 }

 void MappingHelpers::correct_nmea_coor(double& lat,double& lon)
 {
 	double precomma = trunc(lat/100);
 	double postcomma = (lat-(precomma)*100)/60;
 	lat =  precomma + postcomma;

 	precomma = trunc(lon/100);
 	postcomma = (lon-(precomma)*100)/60;
 	lon = precomma + postcomma;
 }

std::string MappingHelpers::FromMarkColorToText(MARKING_COLOR mark_color)
{
	switch(mark_color)
	{
	case MARK_COLOR_WHITE:
		return "white";
		break;
	case MARK_COLOR_YELLOW:
		return "yellow";
		break;
	case MARK_COLOR_RED:
		return "red";
		break;
	case MARK_COLOR_ORANG:
		return "orange";
		break;
	case MARK_COLOR_BLUE:
		return "blue";
		break;
	default:
		return "white";
		break;
	}

	return "white";
}

std::string MappingHelpers::FromLineTypeToText(LINE_TYPE type)
{
	switch(type)
	{
	case GENERAL_LINE:
		return "default";
		break;
	case DEFAULT_WHITE_LINE:
		return "whiteline";
		break;
	case DEFAULT_YELLOW_LINE:
		return "yellowline";
		break;
	case CONTINUOUS_LINE:
		return "onepiece";
		break;
	case SEPARATION_LINE:
		return "separation";
		break;
	case SUPPORT_LINE:
		return "support";
		break;
	default:
		return "default";
		break;
	}

	return "default";
}

std::string MappingHelpers::FromLightTypeToText(TRAFFIC_LIGHT_TYPE type)
{
	switch(type)
	{
	case RED_LIGHT:
		return "red light";
		break;
	case GREEN_LIGHT:
		return "green light";
		break;
	case YELLOW_LIGHT:
		return "yellow light";
		break;
	case CROSS_GREEN:
		return "crossing green";
		break;
	case CROSS_RED:
		return "crossing red";
		break;
	case LEFT_GREEN:
		return "left arrow";
		break;
	case FORWARD_GREEN:
		return "forward arrow";
		break;
	case RIGHT_GREEN:
		return "right arrow";
		break;
	case FLASH_YELLOW:
		return "flash yellow";
		break;
	case FLASH_RED:
		return "flash red";
		break;
	default:
		return "unknown";

	}
}

std::string MappingHelpers::FromSignTypeToText(TRAFFIC_SIGN_TYPE type)
{
	switch(type)
	{
	case UNKNOWN_SIGN:
		return "unknown sign";
		break;
	case STOP_SIGN:
		return "stop sign";
		break;
	case MAX_SPEED_SIGN:
		return "max speed";
		break;
	case MIN_SPEED_SIGN:
		return "min speed";
		break;
	case NO_PARKING_SIGN:
		return "no parking";
		break;
	case SCHOOL_CROSSING_SIGN:
		return "school crossing";
		break;
	default:
		return "unknown";

	}
}

MARKING_COLOR MappingHelpers::FromTextToMarkColor(std::string mark_color)
{
	if(mark_color.compare("white") == 0)
		return MARK_COLOR_WHITE;
	else if(mark_color.compare("yellow") == 0)
		return MARK_COLOR_YELLOW;
	else if(mark_color.compare("red") == 0)
		return MARK_COLOR_RED;
	else if(mark_color.compare("orange") == 0)
		return MARK_COLOR_ORANG;
	else if(mark_color.compare("blue") == 0)
		return MARK_COLOR_BLUE;
	else
		return MARK_COLOR_WHITE;

}

LINE_TYPE MappingHelpers::FromTextToLineType(std::string type)
{
	if(type.compare("default") == 0)
		return GENERAL_LINE;
	else if(type.compare("whiteline") == 0)
		return DEFAULT_WHITE_LINE;
	else if(type.compare("onepiece") == 0)
		return CONTINUOUS_LINE;
	else if(type.compare("separation") == 0)
		return SEPARATION_LINE;
	else if(type.compare("support") == 0)
		return SUPPORT_LINE;
	else
		return GENERAL_LINE;

}

TRAFFIC_LIGHT_TYPE MappingHelpers::FromTextToLightType(std::string light_type)
{
	if(light_type.compare("unknown") == 0)
		return UNKNOWN_LIGHT;
	else if(light_type.compare("red light") == 0)
		return RED_LIGHT;
	else if(light_type.compare("green light") == 0)
		return GREEN_LIGHT;
	else if(light_type.compare("yellow light") == 0)
		return YELLOW_LIGHT;
	else if(light_type.compare("crossing green") == 0)
		return CROSS_GREEN;
	else if(light_type.compare("crossing red") == 0)
		return CROSS_RED;
	else if(light_type.compare("left arrow") == 0)
		return LEFT_GREEN;
	else if(light_type.compare("forward arrow") == 0)
		return FORWARD_GREEN;
	else if(light_type.compare("right arrow") == 0)
		return RIGHT_GREEN;
	else if(light_type.compare("flash yellow") == 0)
		return FLASH_YELLOW;
	else if(light_type.compare("flash red") == 0)
		return FLASH_RED;
	else
		return UNKNOWN_LIGHT;
}

TRAFFIC_SIGN_TYPE MappingHelpers::FromTextToSignType(std::string sign_type)
{
	if(sign_type.compare("unknown sign") == 0)
		return UNKNOWN_SIGN;
	else if(sign_type.compare("stop sign") == 0)
		return STOP_SIGN;
	else if(sign_type.compare("max speed") == 0)
		return MAX_SPEED_SIGN;
	else if(sign_type.compare("min speed") == 0)
		return MIN_SPEED_SIGN;
	else if(sign_type.compare("no parking") == 0)
		return NO_PARKING_SIGN;
	else if(sign_type.compare("school crossing") == 0)
		return SCHOOL_CROSSING_SIGN;
	else
		return UNKNOWN_SIGN;
}
LINE_TYPE MappingHelpers::FromNumberToLineType(int type)
{
	if(type < 0)
		return GENERAL_LINE;
	else if(type == 0)
		return DEFAULT_WHITE_LINE;
	else if(type == 1)
		return CONTINUOUS_LINE;
	else if(type == 2)
		return SEPARATION_LINE;
	else if(type == 3)
		return SUPPORT_LINE;
	else if(type == 4)
		return GENERAL_LINE;
	else
		return DEFAULT_WHITE_LINE;

}

MARKING_COLOR MappingHelpers::FromNumberToMarkColor(int color)
{
	if(color < 0)
		return MARK_COLOR_BLUE;
	else if(color == 0)
		return MARK_COLOR_WHITE;
	else if(color == 1)
		return MARK_COLOR_YELLOW;
	else if(color == 2)
		return MARK_COLOR_RED;
	else if(color == 3)
		return MARK_COLOR_ORANG;
	else if(color == 4)
		return MARK_COLOR_BLUE;
	else
		return MARK_COLOR_WHITE;
}

TRAFFIC_LIGHT_TYPE MappingHelpers::FromNumberToLightType(int type)
{
	if(type <= 0)
		return UNKNOWN_LIGHT;
	else if(type == 1)
		return RED_LIGHT;
	else if(type == 2)
		return GREEN_LIGHT;
	else if(type == 3)
		return YELLOW_LIGHT;
	else if(type == 4)
		return CROSS_RED;
	else if(type == 5)
		return CROSS_GREEN;

	else if(type == 6)
		return LEFT_GREEN;
	else if(type == 7)
		return FORWARD_GREEN;
	else if(type == 8)
		return RIGHT_GREEN;
	else if(type == 9)
		return FLASH_YELLOW;
	else if(type == 10)
		return FLASH_RED;
	else
		return UNKNOWN_LIGHT;
}

TRAFFIC_SIGN_TYPE MappingHelpers::FromNumberToSignType(int type)
{
	if(type <= 0)
		return UNKNOWN_SIGN;
	else if(type == 1)
		return STOP_SIGN;
	else if(type == 2)
		return MAX_SPEED_SIGN;
	else if(type == 3)
		return MIN_SPEED_SIGN;
	else if(type == 4)
		return NO_PARKING_SIGN;
	else if(type == 5)
		return SCHOOL_CROSSING_SIGN;
	else
		return UNKNOWN_SIGN;
}

void MappingHelpers::LoadProjectionData(const std::string& fileName, RoadNetwork& map)
{
	std::string projFileName = fileName + ".proj.dat";
	ProjectionDataFileReader proj_data(projFileName);
	if(proj_data.ReadAllData() > 0)
	{
		if(proj_data.m_data_list.at(0).proj_type.compare("UTM")==0)
			map.proj = UTM_PROJ;
		else if(proj_data.m_data_list.at(0).proj_type.compare("MGRS")==0)
			map.proj = MGRS_PROJ;
		else
			map.proj = NO_PROJ;

		map.str_proj = proj_data.m_data_list.at(0).proj_str;
		map.origin.pos.lon = proj_data.m_data_list.at(0).lon;
		map.origin.pos.lat = proj_data.m_data_list.at(0).lat;
		map.origin.pos.alt = proj_data.m_data_list.at(0).alt;

		map.origin.pos.x = proj_data.m_data_list.at(0).x;
		map.origin.pos.y = proj_data.m_data_list.at(0).y;
		map.origin.pos.z = proj_data.m_data_list.at(0).z;
	}
}

void MappingHelpers::UpdatePointWithProjection(const RoadNetwork& map, WayPoint& p)
{
	if(map.str_proj.size() > 8)
	{
		xyzTolla_proj(map.str_proj, map.origin, p.pos.x, p.pos.y, p.pos.z, p.pos.lat, p.pos.lon, p.pos.alt);
	}
}

void MappingHelpers::RotateMapUsingIncr(RoadNetwork& map, const double&  angle, const double& x_center, const double& y_center)
{
	Mat3 rotationMat(angle);
	Mat3 translationMat(-x_center, -y_center);
	Mat3 invTranslationMat(x_center, y_center);

	for(auto& sec: map.roadSegments)
		{
			for(auto& l: sec.Lanes)
			{
				for(auto& p: l.points)
				{
					GPSPoint relative_point = p.pos;
					relative_point = translationMat * relative_point;
					relative_point = rotationMat * relative_point;
					relative_point = invTranslationMat * relative_point;
					p.pos = relative_point;
				}
			}
		}

		for(auto& sl: map.stopLines)
		{
			for(auto& p: sl.points)
			{
				GPSPoint relative_point = p.pos;
				relative_point = translationMat * relative_point;
				relative_point = rotationMat * relative_point;
				relative_point = invTranslationMat * relative_point;
				p.pos = relative_point;
			}
		}
}

void MappingHelpers::ShiftMapItemsUsingIncr(std::vector<Lane>& lanes, std::vector<StopLine>& stop_lines, const double& x_inc, const double& y_inc, const double& z_inc)
{
	for(auto& l: lanes)
	{
		for(auto& p: l.points)
		{
			p.pos.x += x_inc;
			p.pos.y += y_inc;
			p.pos.z += z_inc;
		}
	}

	for(auto& sl: stop_lines)
	{
		for(auto& p: sl.points)
		{
			p.pos.x += x_inc;
			p.pos.y += y_inc;
			p.pos.z += z_inc;
		}
	}
}

void MappingHelpers::ShiftMapUsingIncr(RoadNetwork& map, const double& x_inc, const double& y_inc, const double& z_inc)
{
	for(auto& sec: map.roadSegments)
	{
		for(auto& l: sec.Lanes)
		{
			for(auto& p: l.points)
			{
				p.pos.x += x_inc;
				p.pos.y += y_inc;
				p.pos.z += z_inc;
			}
		}
	}

	for(auto& sl: map.stopLines)
	{
		for(auto& p: sl.points)
		{
			p.pos.x += x_inc;
			p.pos.y += y_inc;
			p.pos.z += z_inc;
		}
	}

	for(auto& l: map.lines)
	{
		for(auto& p: l.points)
		{
			p.pos.x += x_inc;
			p.pos.y += y_inc;
			p.pos.z += z_inc;
		}
	}

	for(auto& b: map.boundaries)
	{
		for(auto& p: b.points)
		{
			p.pos.x += x_inc;
			p.pos.y += y_inc;
			p.pos.z += z_inc;
		}

		b.center.pos.x += x_inc;
		b.center.pos.y += y_inc;
		b.center.pos.z += z_inc;
	}

	for(auto& c: map.crossings)
	{
		for(auto& p: c.points)
		{
			p.pos.x += x_inc;
			p.pos.y += y_inc;
			p.pos.z += z_inc;
		}
	}

	for(auto& c: map.curbs)
	{
		for(auto& p: c.points)
		{
			p.pos.x += x_inc;
			p.pos.y += y_inc;
			p.pos.z += z_inc;
		}
	}

	for(auto& m: map.markings)
	{
		for(auto& p: m.points)
		{
			p.pos.x += x_inc;
			p.pos.y += y_inc;
			p.pos.z += z_inc;
		}
	}

	for(auto& s: map.signs)
	{
		for(auto& p: s.points)
		{
			p.pos.x += x_inc;
			p.pos.y += y_inc;
			p.pos.z += z_inc;
		}
		s.pose.pos.x += x_inc;
		s.pose.pos.y += y_inc;
		s.pose.pos.z += z_inc;
	}

	for(auto& s: map.trafficLights)
	{
		s.pose.pos.x += x_inc;
		s.pose.pos.y += y_inc;
		s.pose.pos.z += z_inc;
	}
}

void MappingHelpers::ShiftStopLinesToMatchTrafficLights(RoadNetwork& map)
{
	for(auto& sl: map.stopLines)
	{
		Lane* pLane = GetLaneById(sl.laneId, map);

		if(pLane != nullptr && sl.points.size() > 0 && sl.lightIds.size() > 0)
		{
			RelativeInfo sl_info;
			PlanningHelpers::GetRelativeInfoLimited(pLane->points, sl.points.at(0), sl_info);
			TrafficLight* pTL = nullptr;
			for(auto& tl: map.trafficLights)
			{
				if(tl.id == sl.lightIds.at(0))
				{
					pTL = &tl;
					break;
				}
			}

			if(pTL != nullptr)
			{
				RelativeInfo tl_info;
				PlanningHelpers::GetRelativeInfoLimited(pLane->points, pTL->pose, tl_info);

				if(tl_info.bBefore) // move stop line
				{
					//std::cout << "SLID " << sl.id  << ", back d: " << sl_info.from_back_distance << ", front d: " << sl_info.to_front_distance << ", TLID: " << pTL->id << " ,back d: " << tl_info.from_back_distance << ", front d: " << tl_info.to_front_distance << std::endl;

					for(auto& sl_point: sl.points)
					{
						RelativeInfo sl_point_info;
						PlanningHelpers::GetRelativeInfoLimited(pLane->points, sl_point, sl_point_info);

						sl_point.pos.x = tl_info.perp_point.pos.x + sl_point_info.perp_distance*cos(tl_info.perp_point.pos.a+M_PI_2);
						sl_point.pos.y = tl_info.perp_point.pos.y + sl_point_info.perp_distance*sin(tl_info.perp_point.pos.a+M_PI_2);
						sl_point.pos.z = tl_info.perp_point.pos.z;
					}
				}
			}
		}
//		else
//		{
//			std::cout << "Problem with StopLine connection: lane pointer= " << pLane << std::endl;
//		}

	}
}

void MappingHelpers::ShiftMapUsingInternalOrigin(RoadNetwork& map)
{
	ShiftMapUsingIncr(map, map.origin.pos.x, map.origin.pos.y, map.origin.pos.z);
}

void MappingHelpers::SplitLane(int lane_id, int point_index, RoadNetwork& map, Lane& modified_lane, Lane& added_lane)
{
	for(unsigned int i=0; i < map.roadSegments.size(); i++)
	{
		for(int il=0; il < (int)map.roadSegments.at(i).Lanes.size(); il++)
		{
			if(map.roadSegments.at(i).Lanes.at(il).id == lane_id)
			{
				Lane* pL = &map.roadSegments.at(i).Lanes.at(il);
				if(point_index > 0 && point_index < (int)pL->points.size()-1)
				{
					if(pL->points.at(point_index).fromIds.size() > 1 || pL->points.at(point_index).toIds.size() > 1 || pL->points.at(point_index+1).fromIds.size() > 1)
					{
						std::cout << "Can't Split lane, multi-connection exists, please remove the multiple connection manually first. " << std::endl;
						return;
					}

					pL->points.at(point_index).pos.x = (pL->points.at(point_index-1).pos.x + pL->points.at(point_index).pos.x)/2.0;
					pL->points.at(point_index).pos.y = (pL->points.at(point_index-1).pos.y + pL->points.at(point_index).pos.y)/2.0;
					pL->points.at(point_index).pos.z = (pL->points.at(point_index-1).pos.z + pL->points.at(point_index).pos.z)/2.0;

					Lane p_new_lane = *pL;
					RoadNetwork::g_max_lane_id++;
					p_new_lane.id = RoadNetwork::g_max_lane_id;

					p_new_lane.points.clear();
					p_new_lane.fromIds.clear();
					p_new_lane.toIds.clear();
					p_new_lane.fromLanes.clear();
					p_new_lane.toLanes.clear();

					p_new_lane.fromIds.push_back(pL->id);
					p_new_lane.toIds = pL->toIds;
					pL->toIds.clear();
					pL->toIds.push_back(p_new_lane.id);

					for(auto& id : p_new_lane.toIds)
					{
						Lane* pTempLane = MappingHelpers::GetLaneById(id , map);
						if(pTempLane != nullptr)
						{
							for(unsigned int j = 0; j < pTempLane->fromIds.size(); j++)
							{
								if(pTempLane->fromIds.at(j) == pL->id)
								{
									pTempLane->fromIds.at(j) = p_new_lane.id;
								}
							}
						}
					}

					WayPoint new_start = pL->points.at(point_index);
					RoadNetwork::g_max_point_id++;
					new_start.id = RoadNetwork::g_max_point_id;

					pL->points.at(point_index).toIds.clear();
					pL->points.at(point_index).toIds.push_back(new_start.id);

					new_start.toIds.clear();
					new_start.toIds.push_back(pL->points.at(point_index+1).id);
					new_start.fromIds.clear();
					new_start.fromIds.push_back(pL->points.at(point_index).id);

					pL->points.at(point_index+1).fromIds.clear();
					pL->points.at(point_index+1).fromIds.push_back(new_start.id);

					new_start.pos.x = (pL->points.at(point_index+1).pos.x + new_start.pos.x)/2.0;
					new_start.pos.y = (pL->points.at(point_index+1).pos.y + new_start.pos.y)/2.0;
					new_start.pos.z = (pL->points.at(point_index+1).pos.z + new_start.pos.z)/2.0;

					p_new_lane.points.push_back(new_start);
					p_new_lane.points.insert(p_new_lane.points.begin()+1, pL->points.begin()+point_index+1,pL->points.end());
					pL->points.erase(pL->points.begin()+point_index+1, pL->points.end());

					modified_lane = *pL;
					added_lane = p_new_lane;
					map.roadSegments.at(i).Lanes.push_back(p_new_lane);
				}
				break;
			}
		}
	}
}

void MappingHelpers::InsertPointToEndOfPathWithAngleThreshold(const WayPoint& p, std::vector<WayPoint>& path, double max_angle)
{
	if(path.size() < 2)
	{
		path.push_back(p);
	}
	else
	{
		WayPoint p1, p2;
		p2 = path.at(path.size() - 1);
		p1 = path.at(path.size() - 2);

		double before_angle = atan2(p2.pos.y - p1.pos.y, p2.pos.x - p1.pos.x);
		double after_angle = atan2(p.pos.y - p2.pos.y, p.pos.x - p2.pos.x);
		double angle_diff = Angle::AngleBetweenTwoAnglesPositive(after_angle, before_angle);
		if(angle_diff < max_angle)
		{
			path.push_back(p);
		}
	}
}

} /* namespace mapping */
} /* namespace op */
