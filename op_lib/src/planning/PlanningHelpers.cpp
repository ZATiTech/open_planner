
/// \file PlanningHelpers.cpp
/// \brief Helper functions for planning algorithms
/// \author Hatem Darweesh
/// \date Jun 16, 2016


#include "planning/PlanningHelpers.h"
#include "utilities/matrix_operations.h"
#include <string>
#include <algorithm>
#include <float.h>


namespace op {
	namespace planning {

std::vector<std::pair<GPSPoint, GPSPoint> > PlanningHelpers::m_TestingClosestPoint;

using namespace std;
using namespace utilities;

PlanningHelpers::PlanningHelpers()
{
}

PlanningHelpers::~PlanningHelpers()
{
}

int PlanningHelpers::CheckForEndOfPaths(const std::vector<std::vector<WayPoint> >& paths, const WayPoint& currPose, const double& end_range_distance)
{
	if(paths.size() == 0) return -1;

	RelativeInfo info;
	bool ret = PlanningHelpers::GetRelativeInfoRange(paths, currPose, 0.75, info);
	if(ret == true && info.iGlobalPath >= 0 &&  info.iGlobalPath < (int)paths.size() && info.iFront > 0 && info.iFront < (int)paths.at(info.iGlobalPath).size())
	{
		double remaining_distance = (int)paths.at(info.iGlobalPath).at(paths.at(info.iGlobalPath).size()-1).distanceCost - (paths.at(info.iGlobalPath).at(info.iFront).distanceCost + info.to_front_distance);

		RelativeInfo info_curr;
		PlanningHelpers::GetRelativeInfoLimited(paths.at(info.iGlobalPath), currPose, info_curr, info.iBack);

		//std::cout << "Check End of Path: After: " << info_curr.bAfter << ", Distance: " << remaining_distance << ", param_distance: " << end_range_distance << std::endl;
		if(info_curr.bAfter == true || remaining_distance < end_range_distance)
		{
			return info.iGlobalPath;
		}
	}

	return -1;
}


bool PlanningHelpers::GetRelativeInfoRange(const std::vector<std::vector<WayPoint> >& trajectories, const WayPoint& p,const double& searchDistance, RelativeInfo& info)
{
	if((int)trajectories.size() == 0) return false;

	vector<RelativeInfo> infos;
	for(unsigned int i=0; i < trajectories.size(); i++)
	{
		RelativeInfo info_item;
		GetRelativeInfo(trajectories.at(i), p, info_item);
		double angle_diff = Angle::AngleBetweenTwoAnglesPositive(info_item.perp_point.pos.a, p.pos.a)*RAD2DEG;
		if(angle_diff < 75)
		{
			info_item.iGlobalPath = i;
			infos.push_back(info_item);
		}
	}

	if((int)infos.size() == 0)
		return false;
	else if((int)infos.size() == 1)
	{
		info = infos.at(0);
		return true;
	}

	double minCost = DBL_MAX;
	int min_index = 0;

	for(unsigned int i=0 ; i< infos.size(); i++)
	{
		if(searchDistance > 0)
		{
			double laneChangeCost = trajectories.at(infos.at(i).iGlobalPath).at(infos.at(i).iFront).laneChangeCost;
			if(fabs(infos.at(i).perp_distance) < searchDistance && laneChangeCost < minCost)
			{
				min_index = i;
				minCost = laneChangeCost;
			}
		}
		else
		{
			if(fabs(infos.at(i).perp_distance) < minCost)
			{
				min_index = i;
				minCost = infos.at(i).perp_distance;
			}
		}
	}

	info = infos.at(min_index);

	return true;
}

bool PlanningHelpers::GetRelativeInfoDirection(const std::vector<WayPoint>& trajectory, const WayPoint& p, RelativeInfo& info, const int& prevIndex)
{
	if(trajectory.size() < 2) return false;

	WayPoint p0, p1;
	if((int)trajectory.size()==2)
	{
		p0 = trajectory.at(0);
		p1 = WayPoint((trajectory.at(0).pos.x+trajectory.at(1).pos.x)/2.0,
					  (trajectory.at(0).pos.y+trajectory.at(1).pos.y)/2.0,
					  (trajectory.at(0).pos.z+trajectory.at(1).pos.z)/2.0, trajectory.at(0).pos.a);
		info.iFront = 1;
		info.iBack = 0;
	}
	else
	{
		info.iFront = GetClosestNextPointIndexDirectionFast(trajectory, p, prevIndex);

		if(info.iFront > 0)
			info.iBack = info.iFront -1;
		else
			info.iBack = 0;

		if(info.iFront == 0)
		{
			p0 = trajectory.at(info.iFront);
			p1 = trajectory.at(info.iFront+1);
		}
		else if(info.iFront > 0 && info.iFront < (int)trajectory.size()-1)
		{
			p0 = trajectory.at(info.iFront-1);
			p1 = trajectory.at(info.iFront);
		}
		else
		{
			p0 = trajectory.at(info.iFront-1);
			p1 = WayPoint((p0.pos.x+trajectory.at(info.iFront).pos.x)/2.0, (p0.pos.y+trajectory.at(info.iFront).pos.y)/2.0, (p0.pos.z+trajectory.at(info.iFront).pos.z)/2.0, p0.pos.a);
		}
	}

	WayPoint prevWP = p0;
	Mat3 rotationMat(-p1.pos.a);
	Mat3 translationMat(-p.pos.x, -p.pos.y);
	Mat3 invRotationMat(p1.pos.a);
	Mat3 invTranslationMat(p.pos.x, p.pos.y);

	p0.pos = translationMat*p0.pos;
	p0.pos = rotationMat*p0.pos;

	p1.pos = translationMat*p1.pos;
	p1.pos = rotationMat*p1.pos;

	double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
	info.perp_distance = p1.pos.y - m*p1.pos.x; // solve for x = 0

	if(std::isnan(info.perp_distance) || std::isinf(info.perp_distance)) info.perp_distance = 0;

	info.to_front_distance = fabs(p1.pos.x); // distance on the x axes


	info.perp_point = p1;
	info.perp_point.pos.x = 0; // on the same y axis of the car
	info.perp_point.pos.y = info.perp_distance; //perp distance between the car and the trajectory

	info.perp_point.pos = invRotationMat  * info.perp_point.pos;
	info.perp_point.pos = invTranslationMat  * info.perp_point.pos;

	info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);

	info.angle_diff = Angle::AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a)*RAD2DEG;

	return true;
}

bool PlanningHelpers::GetRelativeInfo(const std::vector<WayPoint>& trajectory, const WayPoint& p, RelativeInfo& info, const int& prevIndex )
{
	if((int)trajectory.size() < 2) return false;

	WayPoint p0, p1;
	if((int)trajectory.size()==2)
	{
		p0 = trajectory.at(0);
		p1 = WayPoint((trajectory.at(0).pos.x+trajectory.at(1).pos.x)/2.0,
					  (trajectory.at(0).pos.y+trajectory.at(1).pos.y)/2.0,
					  (trajectory.at(0).pos.z+trajectory.at(1).pos.z)/2.0, trajectory.at(0).pos.a);
		info.iFront = 1;
		info.iBack = 0;
	}
	else
	{
		info.iFront = GetClosestNextPointIndexFast(trajectory, p, prevIndex);
//		WayPoint p2 = p;
//		int old_index = GetClosestNextPointIndex(trajectory, p2, prevIndex);
//		if(old_index != info.iFront)
//			cout << " Alert Alert !!!! fast: " << info.iFront << ", slow: " << old_index  << endl;

		if(info.iFront > 0)
			info.iBack = info.iFront -1;
		else
			info.iBack = 0;

		if(info.iFront == 0)
		{
			p0 = trajectory.at(info.iFront);
			p1 = trajectory.at(info.iFront+1);
		}
		else if(info.iFront > 0 && info.iFront < (int)trajectory.size()-1)
		{
			p0 = trajectory.at(info.iFront-1);
			p1 = trajectory.at(info.iFront);
		}
		else
		{
			p0 = trajectory.at(info.iFront-1);
			p1 = WayPoint((p0.pos.x+trajectory.at(info.iFront).pos.x)/2.0, (p0.pos.y+trajectory.at(info.iFront).pos.y)/2.0, (p0.pos.z+trajectory.at(info.iFront).pos.z)/2.0, p0.pos.a);
		}
	}

	WayPoint prevWP = p0;
	Mat3 rotationMat(-p1.pos.a);
	Mat3 translationMat(-p.pos.x, -p.pos.y);
	Mat3 invRotationMat(p1.pos.a);
	Mat3 invTranslationMat(p.pos.x, p.pos.y);

	p0.pos = translationMat*p0.pos;
	p0.pos = rotationMat*p0.pos;

	p1.pos = translationMat*p1.pos;
	p1.pos = rotationMat*p1.pos;

	double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
	info.perp_distance = p1.pos.y - m*p1.pos.x; // solve for x = 0

	if(std::isnan(info.perp_distance) || std::isinf(info.perp_distance)) info.perp_distance = 0;

	info.to_front_distance = fabs(p1.pos.x); // distance on the x axes


	info.perp_point = p1;
	info.perp_point.pos.x = 0; // on the same y axis of the car
	info.perp_point.pos.y = info.perp_distance; //perp distance between the car and the trajectory

	info.perp_point.pos = invRotationMat  * info.perp_point.pos;
	info.perp_point.pos = invTranslationMat  * info.perp_point.pos;

	info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);

	info.angle_diff = Angle::AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a)*RAD2DEG;

	return true;
}

bool PlanningHelpers::GetRelativeInfoLimited(const std::vector<WayPoint>& trajectory, const WayPoint& p, RelativeInfo& info, const int& prevIndex )
{
	if((int)trajectory.size() < 2) return false;

	WayPoint p0, p1;

	if((int)trajectory.size()==2)
	{
		vector<WayPoint> _trajectory;
		p0 = trajectory.at(0);
		p1 = p0;
		p1 = WayPoint((trajectory.at(0).pos.x+trajectory.at(1).pos.x)/2.0,
					  (trajectory.at(0).pos.y+trajectory.at(1).pos.y)/2.0,
					  (trajectory.at(0).pos.z+trajectory.at(1).pos.z)/2.0, trajectory.at(0).pos.a);
		_trajectory.push_back(p0);
		_trajectory.push_back(p1);
		_trajectory.push_back(trajectory.at(1));

		info.iFront = GetClosestNextPointIndexFast(_trajectory, p, prevIndex);
		if(info.iFront > 0)
			info.iBack = info.iFront -1;
		else
			info.iBack = 0;

		if(info.iFront == 0)
		{
			p0 = _trajectory.at(info.iFront);
			p1 = _trajectory.at(info.iFront+1);
		}
		else if(info.iFront > 0 && info.iFront < (int)_trajectory.size()-1)
		{
			p0 = _trajectory.at(info.iFront-1);
			p1 = _trajectory.at(info.iFront);
		}
		else
		{
			p0 = _trajectory.at(info.iFront-1);
			p1 = WayPoint((p0.pos.x+_trajectory.at(info.iFront).pos.x)/2.0, (p0.pos.y+_trajectory.at(info.iFront).pos.y)/2.0, (p0.pos.z+_trajectory.at(info.iFront).pos.z)/2.0, p0.pos.a);
		}

		WayPoint prevWP = p0;
		Mat3 rotationMat(-p1.pos.a);
		Mat3 translationMat(-p.pos.x, -p.pos.y);
		Mat3 invRotationMat(p1.pos.a);
		Mat3 invTranslationMat(p.pos.x, p.pos.y);

		p0.pos = translationMat*p0.pos;
		p0.pos = rotationMat*p0.pos;

		p1.pos = translationMat*p1.pos;
		p1.pos = rotationMat*p1.pos;

		double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
		info.perp_distance = p1.pos.y - m*p1.pos.x; // solve for x = 0

		if(std::isnan(info.perp_distance) || std::isinf(info.perp_distance)) info.perp_distance = 0;

		info.to_front_distance = fabs(p1.pos.x); // distance on the x axes

		info.perp_point = p1;
		info.perp_point.pos.x = 0; // on the same y axis of the car
		info.perp_point.pos.y = info.perp_distance; //perp distance between the car and the _trajectory

		info.perp_point.pos = invRotationMat  * info.perp_point.pos;
		info.perp_point.pos = invTranslationMat  * info.perp_point.pos;

		info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);

		info.angle_diff = Angle::AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a)*RAD2DEG;

		info.bAfter = false;
		info.bBefore = false;

		if(info.iFront == 0)
		{
			info.bBefore = true;
		}
		else if(info.iFront == (int)_trajectory.size()-1)
		{
			int s = (int)_trajectory.size();
			double angle_befor_last = Angle::FixNegativeAngle(atan2(_trajectory.at(s-2).pos.y - _trajectory.at(s-1).pos.y, _trajectory.at(s-2).pos.x - _trajectory.at(s-1).pos.x));
			double angle_from_perp = Angle::FixNegativeAngle(atan2(info.perp_point.pos.y - _trajectory.at(s-1).pos.y, info.perp_point.pos.x - _trajectory.at(s-1).pos.x));
			double diff_last_perp = Angle::AngleBetweenTwoAnglesPositive(angle_befor_last, angle_from_perp);
			info.after_angle = diff_last_perp;
			if(diff_last_perp > M_PI_2)
			{
				info.bAfter = true;
			}

		}
	}
	else
	{
		info.iFront = GetClosestNextPointIndexFast(trajectory, p, prevIndex);
		if(info.iFront > 0)
			info.iBack = info.iFront -1;
		else
			info.iBack = 0;

		if(info.iFront == 0)
		{
			p0 = trajectory.at(info.iFront);
			p1 = trajectory.at(info.iFront+1);
		}
		else if(info.iFront > 0 && info.iFront < (int)trajectory.size()-1)
		{
			p0 = trajectory.at(info.iFront-1);
			p1 = trajectory.at(info.iFront);
		}
		else
		{
			p0 = trajectory.at(info.iFront-1);
			p1 = WayPoint((p0.pos.x+trajectory.at(info.iFront).pos.x)/2.0, (p0.pos.y+trajectory.at(info.iFront).pos.y)/2.0, (p0.pos.z+trajectory.at(info.iFront).pos.z)/2.0, p0.pos.a);
		}

		WayPoint prevWP = p0;
		Mat3 rotationMat(-p1.pos.a);
		Mat3 translationMat(-p.pos.x, -p.pos.y);
		Mat3 invRotationMat(p1.pos.a);
		Mat3 invTranslationMat(p.pos.x, p.pos.y);

		p0.pos = translationMat*p0.pos;
		p0.pos = rotationMat*p0.pos;

		p1.pos = translationMat*p1.pos;
		p1.pos = rotationMat*p1.pos;

		double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
		info.perp_distance = p1.pos.y - m*p1.pos.x; // solve for x = 0

		if(std::isnan(info.perp_distance) || std::isinf(info.perp_distance)) info.perp_distance = 0;

		info.to_front_distance = fabs(p1.pos.x); // distance on the x axes

		info.perp_point = p1;
		info.perp_point.pos.x = 0; // on the same y axis of the car
		info.perp_point.pos.y = info.perp_distance; //perp distance between the car and the trajectory

		info.perp_point.pos = invRotationMat  * info.perp_point.pos;
		info.perp_point.pos = invTranslationMat  * info.perp_point.pos;

		info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);

		info.angle_diff = Angle::AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a)*RAD2DEG;

		info.bAfter = false;
		info.bBefore = false;

		if(info.iFront == 0)
		{
			info.bBefore = true;
		}
		else if(info.iFront == (int)trajectory.size()-1)
		{
			int s = trajectory.size();
			double angle_befor_last = Angle::FixNegativeAngle(atan2(trajectory.at(s-2).pos.y - trajectory.at(s-1).pos.y, trajectory.at(s-2).pos.x - trajectory.at(s-1).pos.x));
			double angle_from_perp = Angle::FixNegativeAngle(atan2(info.perp_point.pos.y - trajectory.at(s-1).pos.y, info.perp_point.pos.x - trajectory.at(s-1).pos.x));
			double diff_last_perp = Angle::AngleBetweenTwoAnglesPositive(angle_befor_last, angle_from_perp);
			info.after_angle = diff_last_perp;
			if(diff_last_perp > M_PI_2)
			{
				info.bAfter = true;
			}

		}
	}

	return true;
}

bool PlanningHelpers::GetRelativeInfoDirectionLimited(const std::vector<WayPoint>& trajectory, const WayPoint& p, RelativeInfo& info, const int& prevIndex )
{
	if((int)trajectory.size() < 2) return false;

	WayPoint p0, p1;

	if((int)trajectory.size()==2)
	{
		vector<WayPoint> _trajectory;
		p0 = trajectory.at(0);
		p1 = p0;
		p1 = WayPoint((trajectory.at(0).pos.x+trajectory.at(1).pos.x)/2.0,
					  (trajectory.at(0).pos.y+trajectory.at(1).pos.y)/2.0,
					  (trajectory.at(0).pos.z+trajectory.at(1).pos.z)/2.0, trajectory.at(0).pos.a);
		_trajectory.push_back(p0);
		_trajectory.push_back(p1);
		_trajectory.push_back(trajectory.at(1));

		info.iFront = GetClosestNextPointIndexDirectionFast(_trajectory, p, prevIndex);
		if(info.iFront > 0)
			info.iBack = info.iFront -1;
		else
			info.iBack = 0;

		if(info.iFront == 0)
		{
			p0 = _trajectory.at(info.iFront);
			p1 = _trajectory.at(info.iFront+1);
		}
		else if(info.iFront > 0 && info.iFront < (int)_trajectory.size()-1)
		{
			p0 = _trajectory.at(info.iFront-1);
			p1 = _trajectory.at(info.iFront);
		}
		else
		{
			p0 = _trajectory.at(info.iFront-1);
			p1 = WayPoint((p0.pos.x+_trajectory.at(info.iFront).pos.x)/2.0, (p0.pos.y+_trajectory.at(info.iFront).pos.y)/2.0, (p0.pos.z+_trajectory.at(info.iFront).pos.z)/2.0, p0.pos.a);
		}

		WayPoint prevWP = p0;
		Mat3 rotationMat(-p1.pos.a);
		Mat3 translationMat(-p.pos.x, -p.pos.y);
		Mat3 invRotationMat(p1.pos.a);
		Mat3 invTranslationMat(p.pos.x, p.pos.y);

		p0.pos = translationMat*p0.pos;
		p0.pos = rotationMat*p0.pos;

		p1.pos = translationMat*p1.pos;
		p1.pos = rotationMat*p1.pos;

		double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
		info.perp_distance = p1.pos.y - m*p1.pos.x; // solve for x = 0

		if(std::isnan(info.perp_distance) || std::isinf(info.perp_distance)) info.perp_distance = 0;

		info.to_front_distance = fabs(p1.pos.x); // distance on the x axes

		info.perp_point = p1;
		info.perp_point.pos.x = 0; // on the same y axis of the car
		info.perp_point.pos.y = info.perp_distance; //perp distance between the car and the _trajectory

		info.perp_point.pos = invRotationMat  * info.perp_point.pos;
		info.perp_point.pos = invTranslationMat  * info.perp_point.pos;

		info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);

		info.angle_diff = Angle::AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a)*RAD2DEG;

		info.bAfter = false;
		info.bBefore = false;

		if(info.iFront == 0)
		{
			info.bBefore = true;
		}
		else if(info.iFront == (int)_trajectory.size()-1)
		{
			int s = (int)_trajectory.size();
			double angle_befor_last = Angle::FixNegativeAngle(atan2(_trajectory.at(s-2).pos.y - _trajectory.at(s-1).pos.y, _trajectory.at(s-2).pos.x - _trajectory.at(s-1).pos.x));
			double angle_from_perp = Angle::FixNegativeAngle(atan2(info.perp_point.pos.y - _trajectory.at(s-1).pos.y, info.perp_point.pos.x - _trajectory.at(s-1).pos.x));
			double diff_last_perp = Angle::AngleBetweenTwoAnglesPositive(angle_befor_last, angle_from_perp);
			info.after_angle = diff_last_perp;
			if(diff_last_perp > M_PI_2)
			{
				info.bAfter = true;
			}

		}
	}
	else
	{
		info.iFront = GetClosestNextPointIndexDirectionFast(trajectory, p, prevIndex);
		if(info.iFront > 0)
			info.iBack = info.iFront -1;
		else
			info.iBack = 0;

		if(info.iFront == 0)
		{
			p0 = trajectory.at(info.iFront);
			p1 = trajectory.at(info.iFront+1);
		}
		else if(info.iFront > 0 && info.iFront < (int)trajectory.size()-1)
		{
			p0 = trajectory.at(info.iFront-1);
			p1 = trajectory.at(info.iFront);
		}
		else
		{
			p0 = trajectory.at(info.iFront-1);
			p1 = WayPoint((p0.pos.x+trajectory.at(info.iFront).pos.x)/2.0, (p0.pos.y+trajectory.at(info.iFront).pos.y)/2.0, (p0.pos.z+trajectory.at(info.iFront).pos.z)/2.0, p0.pos.a);
		}

		WayPoint prevWP = p0;
		Mat3 rotationMat(-p1.pos.a);
		Mat3 translationMat(-p.pos.x, -p.pos.y);
		Mat3 invRotationMat(p1.pos.a);
		Mat3 invTranslationMat(p.pos.x, p.pos.y);

		p0.pos = translationMat*p0.pos;
		p0.pos = rotationMat*p0.pos;

		p1.pos = translationMat*p1.pos;
		p1.pos = rotationMat*p1.pos;

		double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
		info.perp_distance = p1.pos.y - m*p1.pos.x; // solve for x = 0

		if(std::isnan(info.perp_distance) || std::isinf(info.perp_distance)) info.perp_distance = 0;

		info.to_front_distance = fabs(p1.pos.x); // distance on the x axes

		info.perp_point = p1;
		info.perp_point.pos.x = 0; // on the same y axis of the car
		info.perp_point.pos.y = info.perp_distance; //perp distance between the car and the trajectory

		info.perp_point.pos = invRotationMat  * info.perp_point.pos;
		info.perp_point.pos = invTranslationMat  * info.perp_point.pos;

		info.from_back_distance = hypot(info.perp_point.pos.y - prevWP.pos.y, info.perp_point.pos.x - prevWP.pos.x);

		info.angle_diff = Angle::AngleBetweenTwoAnglesPositive(p1.pos.a, p.pos.a)*RAD2DEG;

		info.bAfter = false;
		info.bBefore = false;

		if(info.iFront == 0)
		{
			info.bBefore = true;
		}
		else if(info.iFront == (int)trajectory.size()-1)
		{
			int s = (int)trajectory.size();
			double angle_befor_last = Angle::FixNegativeAngle(atan2(trajectory.at(s-2).pos.y - trajectory.at(s-1).pos.y, trajectory.at(s-2).pos.x - trajectory.at(s-1).pos.x));
			double angle_from_perp = Angle::FixNegativeAngle(atan2(info.perp_point.pos.y - trajectory.at(s-1).pos.y, info.perp_point.pos.x - trajectory.at(s-1).pos.x));
			double diff_last_perp = Angle::AngleBetweenTwoAnglesPositive(angle_befor_last, angle_from_perp);
			info.after_angle = diff_last_perp;
			if(diff_last_perp > M_PI_2)
			{
				info.bAfter = true;
			}

		}
	}

	return true;
}

bool PlanningHelpers::GetThreePointsInfo(const WayPoint& p0, const WayPoint& p1, const WayPoint& p2, WayPoint& perp_p, double& long_d, double lat_d)
{

	if(p0.pos.x == p1.pos.x && p0.pos.y == p1.pos.y) return false;
	if(p0.pos.x == p2.pos.x && p0.pos.y == p2.pos.y) return false;
	if(p1.pos.x == p2.pos.x && p1.pos.y == p2.pos.y) return false;

	perp_p = p1;
	WayPoint first_p = p0;
	double angle_x = atan2(p1.pos.y-p0.pos.y, p1.pos.x-p0.pos.x);

	Mat3 rotationMat(-angle_x);
	Mat3 translationMat(-p2.pos.x, -p2.pos.y);
	Mat3 invRotationMat(angle_x);
	Mat3 invTranslationMat(p2.pos.x, p2.pos.y);

	first_p.pos = translationMat*first_p.pos;
	first_p.pos = rotationMat*first_p.pos;

	perp_p.pos = translationMat*perp_p.pos;
	perp_p.pos = rotationMat*perp_p.pos;

	if(perp_p.pos.x-first_p.pos.x == 0) return false;

	double m = (perp_p.pos.y-first_p.pos.y)/(perp_p.pos.x-first_p.pos.x);
	lat_d = perp_p.pos.y - m*perp_p.pos.x; // solve for x = 0

	if(std::isnan(lat_d) || std::isinf(lat_d)) return false;

	if(perp_p.pos.x < 0)
		return false;

	perp_p.pos.x = 0; // on the same y axis of the car
	perp_p.pos.y = lat_d; //perp distance between the car and the trajectory

	perp_p.pos = invRotationMat  * perp_p.pos;
	perp_p.pos = invTranslationMat  * perp_p.pos;

	long_d = hypot(perp_p.pos.y - first_p.pos.y, perp_p.pos.x - first_p.pos.x);

	return true;
}

WayPoint PlanningHelpers::GetFollowPointOnTrajectory(const std::vector<WayPoint>& trajectory, const RelativeInfo& init_p, const double& distance, unsigned int& point_index)
{
	WayPoint follow_point;

	if((int)trajectory.size()==0) return follow_point;

	//condition 1, if far behind the first point on the trajectory
	int local_i = init_p.iFront;

	if(init_p.iBack == 0 && init_p.iBack == init_p.iFront && init_p.from_back_distance > distance)
	{
		follow_point = trajectory.at(init_p.iFront);
		follow_point.pos.x = init_p.perp_point.pos.x + distance * cos(follow_point.pos.a);
		follow_point.pos.y = init_p.perp_point.pos.y + distance * sin(follow_point.pos.a);
	}
	//condition 2, if far after the last point on the trajectory
	else if(init_p.iFront == (int)trajectory.size() - 1)
	{
		follow_point = trajectory.at(init_p.iFront);
		follow_point.pos.x = init_p.perp_point.pos.x + distance * cos(follow_point.pos.a);
		follow_point.pos.y = init_p.perp_point.pos.y + distance * sin(follow_point.pos.a);
	}
	else
	{
		double d = init_p.to_front_distance;
		while(local_i < (int)trajectory.size()-1 && d < distance)
		{
			local_i++;
			d += hypot(trajectory.at(local_i).pos.y - trajectory.at(local_i-1).pos.y, trajectory.at(local_i).pos.x - trajectory.at(local_i-1).pos.x);
		}

		double d_part = distance - d;

		follow_point = trajectory.at(local_i);
		follow_point.pos.x = follow_point.pos.x + d_part * cos(follow_point.pos.a);
		follow_point.pos.y = follow_point.pos.y + d_part * sin(follow_point.pos.a);
	}

	point_index = local_i;

	return follow_point;
}

double PlanningHelpers::GetExactDistanceOnTrajectory(const std::vector<WayPoint>& trajectory, const RelativeInfo& p1,const RelativeInfo& p2)
{
	if((int)trajectory.size() == 0) return 0;

	if(p2.iFront == p1.iFront && p2.iBack == p1.iBack)
	{
		return p2.to_front_distance - p1.to_front_distance;
	}
	else if(p2.iBack >= p1.iFront)
	{
		double d_on_path = p1.to_front_distance + p2.from_back_distance;
		for(int i = p1.iFront; i < p2.iBack; i++)
			d_on_path += hypot(trajectory.at(i+1).pos.y - trajectory.at(i).pos.y, trajectory.at(i+1).pos.x - trajectory.at(i).pos.x);

		return d_on_path;
	}
	else if(p2.iFront <= p1.iBack)
	{
		double d_on_path = p1.from_back_distance + p2.to_front_distance;
		for(int i = p2.iFront; i < p1.iBack; i++)
			d_on_path += hypot(trajectory.at(i+1).pos.y - trajectory.at(i).pos.y, trajectory.at(i+1).pos.x - trajectory.at(i).pos.x);

		return -d_on_path;
	}
	else
	{
		return 0;
	}
}

int PlanningHelpers::GetClosestNextPointIndex_obsolete(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex )
{
	if((int)trajectory.size() == 0 || prevIndex < 0) return 0;

	double d = 0, minD = DBL_MAX;
	int min_index  = prevIndex;

	for(unsigned int i=prevIndex; i < trajectory.size(); i++)
	{
		d  = distance2pointsSqr(trajectory.at(i).pos, p.pos);
		if(d < minD)
		{
			min_index = i;
			minD = d;
		}
	}

//	cout << "Slow=> Start: " << 0;
//	cout << ", End: " << trajectory.size();
//	cout << ", Minimum Before: " << min_index;

	if(min_index < (int)trajectory.size()-2)
	{
		GPSPoint curr, next;
		curr = trajectory.at(min_index).pos;
		next = trajectory.at(min_index+1).pos;
		GPSPoint v_1(p.pos.x - curr.x   ,p.pos.y - curr.y,0,0);
		double norm1 = pointNorm(v_1);
		GPSPoint v_2(next.x - curr.x,next.y - curr.y,0,0);
		double norm2 = pointNorm(v_2);
		double dot_pro = v_1.x*v_2.x + v_1.y*v_2.y;
		double a = Angle::FixNegativeAngle(acos(dot_pro/(norm1*norm2)));
		if(a <= M_PI_2)
			min_index = min_index+1;
	}

//	cout << ", Minimum After: " << min_index << ", Big O: " << trajectory.size() << endl;

	return min_index;
}

int PlanningHelpers::GetClosestNextPointIndexFastV2(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex)
{

	int size = (int)trajectory.size();

		if(size < 2 || prevIndex < 0) return 0;

		double d = 0, minD = DBL_MAX;


		double resolution = hypot(trajectory[1].pos.y -trajectory[0].pos.y , trajectory[1].pos.x -trajectory[0].pos.x);
		double d_to_zero = hypot(p.pos.y -trajectory[0].pos.y , p.pos.x - trajectory[0].pos.x);
		double d_to_size = hypot(trajectory[size-1].pos.y - p.pos.y , trajectory[size-1].pos.x - p.pos.x);

		int iStart = d_to_zero / resolution;
		WayPoint perp_p;
		double lat_d = 0;
		double long_d = 0;

		if(iStart > 0 && iStart < size-1 &&  GetThreePointsInfo(trajectory.at(0), trajectory.at(iStart), p, perp_p, long_d, lat_d))
		{
	//		m_TestingClosestPoint.push_back(make_pair(trajectory.at(0).pos, p.pos));
	//		m_TestingClosestPoint.push_back(make_pair(trajectory.at(iStart).pos, p.pos));
			iStart = long_d / resolution;
		}

		if(iStart>0)
			iStart--;

		int iEnd = size - (d_to_size / resolution);

		if(iEnd >= 0 && iEnd < size-2 &&  GetThreePointsInfo(trajectory.at(size-1), trajectory.at(iEnd), p, perp_p, long_d, lat_d))
		{
	//		m_TestingClosestPoint.push_back(make_pair(trajectory.at(size-1).pos, p.pos));
	//		m_TestingClosestPoint.push_back(make_pair(trajectory.at(iEnd).pos, p.pos));
			iEnd = size - (long_d / resolution);
		}

		if(iEnd < size-1)
			iEnd++;

	//	cout << "Fast=>";
	//	cout << " Start: " << iStart;
	//	cout << ", End: " << iEnd;

		double d_from_start = d_to_zero;
		if(iStart < size)
			d_from_start = hypot(trajectory[iStart].pos.y - p.pos.y , trajectory[iStart].pos.x - p.pos.x);

		double d_from_end = d_to_size;
		if(iEnd >= 0)
			d_from_end = hypot(trajectory[iEnd].pos.y - p.pos.y , trajectory[iEnd].pos.x - p.pos.x);

		if(iStart >= size && iEnd < 0)
		{
			if(d_to_zero < d_to_size)
			{
				iStart = 0;
				iEnd = size/2 -1;
			}
			else
			{
				iStart = size/2;
				iEnd = size - 1;
			}
		}
		else
		{
			if(iStart >=size || (d_from_start > d_to_zero))
				iStart = 0;

			if(iEnd < 0 || (d_from_end > d_to_size))
				iEnd = size-1;
		}

		if(iStart > iEnd)
			iEnd = size-1;

		int min_index  =  iStart;

		int ncout = 0;
		for(int i=iStart; i<= iEnd; i++)
		{
			d  = distance2pointsSqr(trajectory[i].pos, p.pos);
			if(d < minD)
			{
				min_index = i;
				minD = d;
			}
			ncout++;
		}

	//	cout << ", Minimum Before: " << min_index;

		if(min_index < size-2)
		{
			GPSPoint curr, next;
			curr = trajectory[min_index].pos;
			next = trajectory[min_index+1].pos;
			GPSPoint v_1(p.pos.x - curr.x   ,p.pos.y - curr.y,0,0);
			double norm1 = pointNorm(v_1);
			GPSPoint v_2(next.x - curr.x,next.y - curr.y,0,0);
			double norm2 = pointNorm(v_2);
			double dot_pro = v_1.x*v_2.x + v_1.y*v_2.y;
			double a = Angle::FixNegativeAngle(acos(dot_pro/(norm1*norm2)));
			if(a <= M_PI_2)
				min_index = min_index+1;
		}

	//	m_TestingClosestPoint.push_back(make_pair(trajectory.at(min_index).pos, p.pos));

	//	cout << ", Minimum After: " << min_index << ", Big O: " << ncout << endl;
	//	cout << "d_zero: " << d_to_zero << ", d_start: " << d_from_start << endl;
	//	cout << "d_size: " << d_to_size << ", d_end: " << d_from_end << endl;
		return min_index;


}

int PlanningHelpers::GetClosestNextPointIndexFast(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex )
{
	int size = (int)trajectory.size();

		if(size < 2 || prevIndex < 0) return 0;

		double d = 0, minD = DBL_MAX;
		int min_index  = prevIndex;
		int iStart = prevIndex;
		int iEnd = size;
		double resolution = hypot(trajectory[1].pos.y -trajectory[0].pos.y , trajectory[1].pos.x -trajectory[0].pos.x);

		//divide every 5 meters
		int skip_factor = 5;
		if(resolution > skip_factor)
			resolution = skip_factor;


		int skip = 1;
		if(resolution > 0)
			skip = skip_factor/resolution;

		for(int i=0; i< size; i+=skip)
		{
			if(i+skip/2 < size)
				d  = (distance2pointsSqr(trajectory[i].pos, p.pos) + distance2pointsSqr(trajectory[i+skip/2].pos, p.pos))/2.0;
			else
				d  = distance2pointsSqr(trajectory[i].pos, p.pos);
			if(d < minD)
			{
				iStart = i-skip;
				iEnd = i+skip;
				minD = d;
				min_index = i;
			}
		}

		if((size - skip/2 - 1) > 0)
			d  = (distance2pointsSqr(trajectory[size-1].pos, p.pos) + distance2pointsSqr(trajectory[size - skip/2 -1 ].pos, p.pos))/2.0;
		else
			d  = distance2pointsSqr(trajectory[size-1].pos, p.pos);

		if(d < minD)
		{
			iStart = size-skip;
			iEnd = size+skip;
			minD = d;
			min_index = size-1;
		}

		if(iStart < 0) iStart = 0;
		if(iEnd >= size) iEnd = size -1;

		for(int i=iStart; i< iEnd; i++)
		{
			d  = distance2pointsSqr(trajectory[i].pos, p.pos);
			if(d < minD)
			{
				min_index = i;
				minD = d;
			}
		}

		if(min_index < size-1)
		{
			GPSPoint curr, next;
			curr = trajectory[min_index].pos;
			next = trajectory[min_index+1].pos;
			GPSPoint v_1(p.pos.x - curr.x   ,p.pos.y - curr.y,0,0);
			double norm1 = pointNorm(v_1);
			GPSPoint v_2(next.x - curr.x,next.y - curr.y,0,0);
			double norm2 = pointNorm(v_2);
			double dot_pro = v_1.x*v_2.x + v_1.y*v_2.y;
			double a = Angle::FixNegativeAngle(acos(dot_pro/(norm1*norm2)));
			if(a <= M_PI_2)
				min_index = min_index+1;
		}

		//m_TestingClosestPoint.push_back(make_pair(trajectory.at(min_index).pos, p.pos));

		return min_index;
}

int PlanningHelpers::GetClosestNextPointIndexDirectionFast(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex, const bool& debug)
{
	if((int)trajectory.size() < 2 || prevIndex < 0) return 0;

	double d = 0, minD = DBL_MAX;
	int min_index  = prevIndex;
	double min_angle = DBL_MAX;

	for(unsigned int i=prevIndex; i< trajectory.size(); i++)
	{
		d  = distance2pointsSqr(trajectory.at(i).pos, p.pos);
		double angle_diff = Angle::AngleBetweenTwoAnglesPositive(trajectory.at(i).pos.a, p.pos.a)*RAD2DEG;
		double index_diff = (int)i - prevIndex; //just for carla
		//if(d < minD && angle_diff < 45)
		if(d < minD && angle_diff < 45 && index_diff < 100) //just for carla
		{
			min_index = i;
			minD = d;
			min_angle = angle_diff;
		}
	}

	if(min_index < (int)trajectory.size()-2)
	{
		GPSPoint curr, next;
		curr = trajectory.at(min_index).pos;
		next = trajectory.at(min_index+1).pos;
		GPSPoint v_1(p.pos.x - curr.x   ,p.pos.y - curr.y,0,0);
		double norm1 = pointNorm(v_1);
		GPSPoint v_2(next.x - curr.x,next.y - curr.y,0,0);
		double norm2 = pointNorm(v_2);
		double dot_pro = v_1.x*v_2.x + v_1.y*v_2.y;
		double a = Angle::FixNegativeAngle(acos(dot_pro/(norm1*norm2)));
		if(a <= M_PI_2)
		{
			min_index = min_index+1;
		}
	}

	if(debug)
	{
		std::cout << std::endl << "  MinIndex: " << min_index << ", PrevIndex: " << prevIndex << ", MinDist: " << minD << ", MinAngle: " << min_angle << std::endl;
	}

	return min_index;
}

int PlanningHelpers::GetClosestPointIndex(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex )
{
	if((int)trajectory.size() == 0 || prevIndex < 0) return 0;

	double d = 0, minD = DBL_MAX;
	int min_index  = prevIndex;

	for(unsigned int i=prevIndex; i< trajectory.size(); i++)
	{
		d  = distance2pointsSqr(trajectory.at(i).pos, p.pos);
		if(d < minD)
		{
			min_index = i;
			minD = d;
		}
	}

	return min_index;
}

WayPoint PlanningHelpers::GetPerpendicularOnTrajectory_obsolete(const vector<WayPoint>& trajectory, const WayPoint& p, double& distance, const int& prevIndex )
{
	if((int)trajectory.size() < 2) return p;

	WayPoint p0, p1, p2, perp;
	if((int)trajectory.size()==2)
	{
		p0 = trajectory.at(0);
		p1 = WayPoint((trajectory.at(0).pos.x+trajectory.at(1).pos.x)/2.0,
					  (trajectory.at(0).pos.y+trajectory.at(1).pos.y)/2.0,
					  (trajectory.at(0).pos.z+trajectory.at(1).pos.z)/2.0, trajectory.at(0).pos.a);
		p2 = trajectory.at(1);
	}
	else
	{
		int next_index = GetClosestNextPointIndex_obsolete(trajectory, p, prevIndex);

		if(next_index == 0)
		{
			p0 = trajectory[next_index];
			p1 = trajectory[next_index+1];
			p2 = trajectory[next_index+2];
		}
		else if(next_index > 0 && next_index < (int)trajectory.size()-1)
		{
			p0 = trajectory[next_index-1];
			p1 = trajectory[next_index];
			p2 = trajectory[next_index+1];
		}
		else
		{
			p0 = trajectory[next_index-1];
			p2 = trajectory[next_index];

			p1 = WayPoint((p0.pos.x+p2.pos.x)/2.0, (p0.pos.y+p2.pos.y)/2.0, (p0.pos.z+p2.pos.z)/2.0, p0.pos.a);

		}
	}

	Mat3 rotationMat(-p1.pos.a);
	Mat3 translationMat(-p.pos.x, -p.pos.y);
	Mat3 invRotationMat(p1.pos.a);
	Mat3 invTranslationMat(p.pos.x, p.pos.y);

	p0.pos = translationMat*p0.pos;
	p0.pos = rotationMat*p0.pos;

	p1.pos = translationMat*p1.pos;
	p1.pos = rotationMat*p1.pos;

	p2.pos = translationMat*p2.pos;
	p2.pos= rotationMat*p2.pos;

	double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
	double d = p1.pos.y - m*p1.pos.x; // solve for x = 0
	distance = p1.pos.x; // distance on the x axes

	perp = p1;
	perp.pos.x = 0; // on the same y axis of the car
	perp.pos.y = d; //perp distance between the car and the trajectory

	perp.pos = invRotationMat  * perp.pos;
	perp.pos = invTranslationMat  * perp.pos;

	return perp;
}

double PlanningHelpers::GetPerpDistanceToTrajectorySimple_obsolete(const vector<WayPoint>& trajectory, const WayPoint& p,const int& prevIndex)
{

	if((int)trajectory.size() < 2)
		return 0;

	WayPoint p0, p1, p2;
	int next_index = 0;
	if((int)trajectory.size()==2)
	{
		p0 = trajectory.at(0);
		p2 = trajectory.at(1);
		p1 = WayPoint((p0.pos.x+p2.pos.x)/2.0, (p0.pos.y+p2.pos.y)/2.0, (p0.pos.z+p2.pos.z)/2.0, p0.pos.a);

	}
	else
	{
		next_index = GetClosestNextPointIndexFast(trajectory, p, prevIndex);
		if(next_index == 0)
		{
			p0 = trajectory[next_index];
			p1 = trajectory[next_index+1];
			p2 = trajectory[next_index+2];
		}
		else if(next_index > 0 && next_index < (int)trajectory.size()-1)
		{
			p0 = trajectory[next_index-1];
			p1 = trajectory[next_index];
			p2 = trajectory[next_index+1];
		}
		else
		{
			p0 = trajectory[next_index-1];
			p2 = trajectory[next_index];

			p1 = WayPoint((p0.pos.x+p2.pos.x)/2.0, (p0.pos.y+p2.pos.y)/2.0, (p0.pos.z+p2.pos.z)/2.0, p0.pos.a);

		}

	}


	Mat3 rotationMat(-p1.pos.a);
	Mat3 translationMat(-p.pos.x, -p.pos.y);

	p0.pos = translationMat*p0.pos;
	p0.pos = rotationMat*p0.pos;

	p1.pos = translationMat*p1.pos;
	p1.pos = rotationMat*p1.pos;

	p2.pos = translationMat*p2.pos;
	p2.pos = rotationMat*p2.pos;

	double m = (p1.pos.y-p0.pos.y)/(p1.pos.x-p0.pos.x);
	double d = p1.pos.y - m*p1.pos.x;

	if(std::isnan(d) || std::isinf(d))
	{
	  //assert(false);
	  d = 0;
	}

	return d;
}

double PlanningHelpers::GetPerpDistanceToVectorSimple_obsolete(const WayPoint& point1, const WayPoint& point2, const WayPoint& pose)
{
	WayPoint p1 = point1, p2 = point2;
	Mat3 rotationMat(-p1.pos.a);
	Mat3 translationMat(-pose.pos.x, -pose.pos.y);

	p1.pos = translationMat*p1.pos;
	p1.pos = rotationMat*p1.pos;

	p2.pos = translationMat*p2.pos;
	p2.pos = rotationMat*p2.pos;

	double m = (p2.pos.y-p1.pos.y)/(p2.pos.x-p1.pos.x);
	double d = p2.pos.y - m*p2.pos.x;

	if(std::isnan(d) || std::isinf(d))
	{
	  //assert(false);
	  d = 0;
	}

	return d;
}

WayPoint PlanningHelpers::GetNextPointOnTrajectory_obsolete(const vector<WayPoint>& trajectory, const double& distance, const int& currIndex)
{
	assert((int)trajectory.size()>0);

	int local_currIndex = currIndex;

	if(local_currIndex < 0 || local_currIndex >= (int)trajectory.size())
		return trajectory.at(0);

	WayPoint p1 = trajectory.at(local_currIndex);
	WayPoint p2;

	double d = 0;
	while(local_currIndex < ((int)trajectory.size()-1) && d < distance)
	{
		local_currIndex++;
		p2 = p1;
		p1 = trajectory.at(local_currIndex);
		d += distance2points(p1.pos, p2.pos);
	}

	if(local_currIndex >= (int)trajectory.size()-1)
	  return p1;

	double distance_diff = distance -  d;

	p2 = trajectory.at(local_currIndex);
	p1 = trajectory.at(local_currIndex+1);

	GPSPoint uv(p1.pos.x - p2.pos.x, p1.pos.y - p2.pos.y ,0,0);
	double v_norm = pointNorm(uv);

	assert(v_norm != 0);

	uv.x = (uv.x / v_norm) * distance_diff;
	uv.y = (uv.y / v_norm) * distance_diff;

	double ydiff = p1.pos.y-p2.pos.y;
	double xdiff = p1.pos.x-p2.pos.x;
	double a =  atan2(ydiff,xdiff);

	WayPoint abs_waypoint = p2;

	abs_waypoint.pos.x = p2.pos.x + uv.x;
	abs_waypoint.pos.y = p2.pos.y + uv.y;
	abs_waypoint.pos.a = a;

	return abs_waypoint;
}

double PlanningHelpers::GetDistanceOnTrajectory_obsolete(const std::vector<WayPoint>& path, const int& start_index, const WayPoint& p)
{

	int end_point_index = GetClosestPointIndex(path, p);
	if(end_point_index > 0)
		end_point_index--;

	double padding_distance = distance2points(path.at(end_point_index).pos, p.pos);

	double d_on_path = 0;
	if(end_point_index >= start_index)
	{
		for(int i = start_index; i < end_point_index; i++)
			d_on_path += distance2points(path.at(i).pos, path.at(i+1).pos);

		d_on_path += padding_distance;
	}
	else
	{
		for(int i = start_index; i > end_point_index; i--)
			d_on_path -= distance2points(path.at(i).pos, path.at(i-1).pos);
	}

	return d_on_path;
}

bool PlanningHelpers::CompareTrajectories(const std::vector<WayPoint>& path1, const std::vector<WayPoint>& path2)
{
	if(path1.size() != path2.size())
		return false;

	for(unsigned int i=0; i< path1.size(); i++)
	{
		if(path1.at(i).v != path2.at(i).v || path1.at(i).pos.x != path2.at(i).pos.x || path1.at(i).pos.y != path2.at(i).pos.y || path1.at(i).pos.alt != path2.at(i).pos.alt || path1.at(i).pos.lon != path2.at(i).pos.lon)
			return false;
	}

	return true;
}

double PlanningHelpers::GetDistanceToClosestStopLineAndCheck(const std::vector<WayPoint>& path, const WayPoint& p, const double& giveUpDistance, int& stopLineID, int& stopSignID, int& trafficLightID, const int& prevIndex)
{
	trafficLightID = stopSignID = stopLineID = -1;

	RelativeInfo info;
	GetRelativeInfo(path, p, info, prevIndex);

	for(unsigned int i=info.iBack; i<path.size(); i++)
	{
		if(path.at(i).stopLineID > 0 && path.at(i).pLane)
		{

			for(unsigned int j = 0; j < path.at(i).pLane->stopLines.size(); j++)
			{

				if(path.at(i).pLane->stopLines.at(j).id == path.at(i).stopLineID)
				{
					stopLineID = path.at(i).stopLineID;

					RelativeInfo stop_info;
					WayPoint stopLineWP ;
					stopLineWP = path.at(i).pLane->stopLines.at(j).points.at(0);
					GetRelativeInfo(path, stopLineWP, stop_info);
					double localDistance = GetExactDistanceOnTrajectory(path, info, stop_info);

					if(localDistance > giveUpDistance)
					{
						stopSignID = path.at(i).pLane->stopLines.at(j).stopSignID;
						if(path.at(i).pLane->stopLines.at(j).lightIds.size() > 0)
							trafficLightID = path.at(i).pLane->stopLines.at(j).lightIds.at(0);
						return localDistance;
					}
				}
			}
		}
	}

	return -1;
}

double PlanningHelpers::GetDistanceToClosestStopLineAndCheckV2(const std::vector<WayPoint>& path, const WayPoint& p, const std::vector<StopLine>& slines, int& stopLineID, int& stopSignID, std::vector<int>& trafficLightIDs)
{
	stopSignID = stopLineID = -1;

	RelativeInfo info;
	GetRelativeInfo(path, p, info);

//	std::vector<int> uni_lane_ids;
//
//	for(auto& p: path)
//	{
//		if(std::find(uni_lane_ids.begin(), uni_lane_ids.end(), p.laneId) == uni_lane_ids.end())
//		{
//			uni_lane_ids.push_back(p.laneId);
//		}
//	}
//
//	std::cout << std::endl;
//	for(auto& ulid: uni_lane_ids)
//	{
//		std::cout << ulid << ", ";
//	}
//	std::cout << std::endl;


	double min_distance = DBL_MAX;

	for(const auto& sl: slines)
	{
		//validate stop line
		if((int)sl.points.size() == 0) continue;

		//file stop line according to global path lanes
		bool bFound = false;
		for(auto& p: path)
		{
			if((std::find(sl.laneIds.begin(), sl.laneIds.end(), p.laneId) != sl.laneIds.end()) || p.laneId == sl.laneId)
			{
				bFound = true;
				break;
			}
		}

		if(bFound == false) continue;

		WayPoint avg_p;
		for(const auto& p: sl.points)
		{
			avg_p.pos.x += p.pos.x;
			avg_p.pos.y += p.pos.y;
			avg_p.pos.z += p.pos.z;
		}

		avg_p.pos.x = avg_p.pos.x / (double)sl.points.size();
		avg_p.pos.y = avg_p.pos.y / (double)sl.points.size();
		avg_p.pos.z = avg_p.pos.z / (double)sl.points.size();

		RelativeInfo stop_info;
		GetRelativeInfo(path, avg_p, stop_info);

		// stop line center shouldn't be more than 10 meters away from the center path
		if(fabs(stop_info.perp_distance) > 5) continue;

		double d = GetExactDistanceOnTrajectory(path, info, stop_info);

		if(d > 0 && d < min_distance)
		{
			min_distance = d;
			stopLineID = sl.id;
			stopSignID = sl.stopSignID;
			trafficLightIDs = sl.lightIds;
		}
	}

	return min_distance;
}

void PlanningHelpers::CreateManualBranchFromTwoPoints(WayPoint& p1,WayPoint& p2 , const double& distance, const DIRECTION_TYPE& direction, std::vector<WayPoint>& path)
{
	WayPoint endWP, midWP;

	double branch_angle = 0;
	if(direction == FORWARD_RIGHT_DIR)
	{
		branch_angle = p1.pos.a-M_PI_2;
	}
	else if(direction == FORWARD_LEFT_DIR)
	{
		branch_angle = p1.pos.a+M_PI_2;
	}
	endWP.pos.y = p2.pos.y + distance*sin(branch_angle);
	endWP.pos.x = p2.pos.x + distance*cos(branch_angle);

	midWP = p2;
	midWP.pos.x = (p1.pos.x+p2.pos.x)/2.0;
	midWP.pos.y = (p1.pos.y+p2.pos.y)/2.0;
	endWP.bDir = midWP.bDir = direction;

	path.clear();
	path.push_back(p1);
	path.push_back(p2);
	path.push_back(endWP);

	//PlanningHelpers::SmoothPath(path, 0.4, 0.1);
	PlanningHelpers::FixPathDensity(path, 1);
	PlanningHelpers::SmoothPath(path, 0.4, 0.25);
	PlanningHelpers::FixPathDensity(path, 0.5);
	PlanningHelpers::SmoothPath(path, 0.25, 0.4);

	for(unsigned int i=0; i < path.size(); i++)
	{
		if(direction == FORWARD_LEFT_DIR)
		{
			path.at(i).state = INITIAL_STATE;
			path.at(i).beh_state = BEH_BRANCH_LEFT_STATE;
			path.at(i).laneId = -2;
		}
		if(direction == FORWARD_RIGHT_DIR)
		{
			path.at(i).state = INITIAL_STATE;
			path.at(i).beh_state = BEH_BRANCH_RIGHT_STATE;
			path.at(i).laneId = -3;
		}
	}
}

void PlanningHelpers::CreateManualBranch(std::vector<WayPoint>& path, const DIRECTION_TYPE& direction)
{
	if((int)path.size() < 5) return;

	//start branch point
	WayPoint branch_start = path.at((int)path.size()-5);
	WayPoint last_wp = path.at((int)path.size()-1);


	WayPoint endWP;
	vector<WayPoint> goal_path;
	double branch_angle = 0;
	if(direction == FORWARD_RIGHT_DIR)
	{
		branch_angle = last_wp.pos.a-M_PI_2;
	}
	else if(direction == FORWARD_LEFT_DIR)
	{
		branch_angle = last_wp.pos.a+M_PI_2;
	}
	endWP.pos.y = last_wp.pos.y + 10*sin(branch_angle);
	endWP.pos.x = last_wp.pos.x + 10*cos(branch_angle);

	WayPoint wp = last_wp;
	wp.pos.x = (last_wp.pos.x+endWP.pos.x)/2.0;
	wp.pos.y = (last_wp.pos.y+endWP.pos.y)/2.0;
	endWP.bDir = wp.bDir = direction;
	goal_path.push_back(wp);
	goal_path.push_back(endWP);

	goal_path.insert(goal_path.begin(), path.end()-5, path.end());
	PlanningHelpers::SmoothPath(goal_path, 0.25, 0.25);
	PlanningHelpers::FixPathDensity(goal_path, 0.75);
	PlanningHelpers::SmoothPath(goal_path, 0.25, 0.35);

	path.erase(path.end()-5, path.end());
	path.insert(path.end(), goal_path.begin(), goal_path.end());

	PlanningHelpers::CalcAngleAndCost(path);

	for(unsigned int i=0; i < path.size(); i++)
	{
		if(direction == FORWARD_LEFT_DIR)
		{
			path.at(i).state = INITIAL_STATE;
			path.at(i).beh_state = BEH_BRANCH_LEFT_STATE;
		}
		if(direction == FORWARD_RIGHT_DIR)
		{
			path.at(i).state = INITIAL_STATE;
			path.at(i).beh_state = BEH_BRANCH_RIGHT_STATE;
		}
	}


}

// Just add new point if distance between any two points is bigger than the res distance
void PlanningHelpers::FixPathResolution(std::vector<WayPoint>& path, const double& res)
{
	bool bChange = true;
	while (bChange && (int)path.size()>1)
	{
		bChange = false;
		WayPoint p1 =  path.at((int)path.size()-1);
		for(unsigned int i=0; i< path.size(); i++)
		{
			WayPoint p2 = path.at(i);
			double d = hypot(p2.pos.y- p1.pos.y, p2.pos.x - p1.pos.x);
			if(d > res)
			{
				WayPoint center_p = p1;
				center_p.pos.x = (p2.pos.x + p1.pos.x)/2.0;
				center_p.pos.y = (p2.pos.y + p1.pos.y)/2.0;
				path.insert(path.begin()+i, center_p);
				bChange = true;
				break;
			}

			p1 = p2;
		}
	}
}

//Change points position along the path so the distance between all points are the same and equal to density distance
void PlanningHelpers::FixPathDensity(vector<WayPoint>& path, const double& distanceDensity)
{
	if((int)path.size() == 0 || distanceDensity==0) return;

	double d = 0, a = 0;
	double margin = distanceDensity*0.01;
	double remaining = 0;
	int nPoints = 0;
	vector<WayPoint> fixedPath;
	fixedPath.push_back(path.at(0));
	for(unsigned int si = 0, ei=1; ei < path.size(); )
	{
		d += hypot(path.at(ei).pos.x- path.at(ei-1).pos.x, path.at(ei).pos.y- path.at(ei-1).pos.y) + remaining;
		a = atan2(path.at(ei).pos.y - path.at(si).pos.y, path.at(ei).pos.x - path.at(si).pos.x);
		double z = path.at(ei).pos.z;

		if(d < distanceDensity - margin ) // skip
		{
			ei++;
			remaining = 0;
		}
		else if(d > (distanceDensity +  margin)) // insert
		{
			WayPoint pm = path.at(si);
			nPoints = d  / distanceDensity;
			for(int k = 0; k < nPoints; k++)
			{
				pm.pos.x = pm.pos.x + distanceDensity * cos(a);
				pm.pos.y = pm.pos.y + distanceDensity * sin(a);
				pm.pos.z = z;
				int closest_index = GetClosestPointIndex(path, pm);
				WayPoint pm_with_correct_info = pm;
				if(closest_index >=0 && closest_index < (int)path.size())
				{
					pm_with_correct_info = path.at(closest_index);
					pm_with_correct_info.pos = pm.pos;
				}

				fixedPath.push_back(pm_with_correct_info);
			}
			remaining = d - nPoints*distanceDensity;
			si++;
			path.at(si).pos = pm.pos;
			ei++;
			d = 0;
		}
		else // push
		{
			d = 0;
			remaining = 0;
			fixedPath.push_back(path.at(ei));
			ei++;
			si = ei - 1;
		}
	}

	/**
	 * The following code is added on 26 September 2020, to solve the missing last point issue,
	 * Also it handles the case when the given resolution is larger than the length of the path.
	 */
	if((int)fixedPath.size() > 1)
	{
		WayPoint e_p_0 = fixedPath.at((int)fixedPath.size()-2);
		WayPoint e_p_1 = fixedPath.at((int)fixedPath.size()-1);
		WayPoint e_p = path.at((int)path.size()-1);
		double d0 = hypot(e_p.pos.y - e_p_0.pos.y, e_p.pos.x - e_p_0.pos.x);
		double d1 = hypot(e_p.pos.y - e_p_1.pos.y, e_p.pos.x - e_p_1.pos.x);

		if(d0 > distanceDensity && d1 > distanceDensity/4.0)
		{
			fixedPath.push_back(e_p);
		}
		else
		{
			fixedPath.erase(fixedPath.end());
			fixedPath.push_back(e_p);
		}
	}
	else
	{
		if((int)path.size() > 1)
		{
			fixedPath.push_back(path.at((int)path.size()-1));
		}
	}
	path = fixedPath;
}

void PlanningHelpers::FixPathDensity(vector<GPSPoint>& path, const double& distanceDensity)
{
	if((int)path.size() == 0 || distanceDensity==0) return;

	double d = 0, a = 0;
	double margin = distanceDensity*0.01;
	double remaining = 0;
	int nPoints = 0;
	vector<GPSPoint> fixedPath;
	fixedPath.push_back(path.at(0));
	for(unsigned int si = 0, ei=1; ei < path.size(); )
	{
		d += hypot(path.at(ei).x- path.at(ei-1).x, path.at(ei).y- path.at(ei-1).y) + remaining;
		a = atan2(path.at(ei).y - path.at(si).y, path.at(ei).x - path.at(si).x);
		double z = path.at(ei).z;

		if(d < distanceDensity - margin ) // skip
		{
			ei++;
			remaining = 0;
		}
		else if(d > (distanceDensity +  margin)) // skip
		{
			GPSPoint pm = path.at(si);
			nPoints = d  / distanceDensity;
			for(int k = 0; k < nPoints; k++)
			{
				pm.x = pm.x + distanceDensity * cos(a);
				pm.y = pm.y + distanceDensity * sin(a);
				pm.z = z;
				fixedPath.push_back(pm);
			}
			remaining = d - nPoints*distanceDensity;
			si++;
			path.at(si) = pm;
			d = 0;
			ei++;
		}
		else
		{
			d = 0;
			remaining = 0;
			fixedPath.push_back(path.at(ei));
			ei++;
			si = ei - 1;
		}
	}

	path = fixedPath;
}

void PlanningHelpers::SmoothPath(vector<WayPoint>& path, double weight_data,
		double weight_smooth, double tolerance)
{

	if ((int)path.size() <= 2 )
	{
		//cout << "Can't Smooth Path, Path_in Size=" << path.size() << endl;
		return;
	}

	vector<WayPoint> path_in = path;
	vector<WayPoint> smoothPath_out =  path;

	double change = tolerance;
	double xtemp, ytemp;
	int nIterations = 0;

	int size = (int)path_in.size();

	while (change >= tolerance)
	{
		change = 0.0;
		for (int i = 1; i < size - 1; i++)
		{
//			if (smoothPath_out[i].pos.a != smoothPath_out[i - 1].pos.a)
//				continue;

			xtemp = smoothPath_out[i].pos.x;
			ytemp = smoothPath_out[i].pos.y;

			smoothPath_out[i].pos.x += weight_data
					* (path_in[i].pos.x - smoothPath_out[i].pos.x);
			smoothPath_out[i].pos.y += weight_data
					* (path_in[i].pos.y - smoothPath_out[i].pos.y);

			smoothPath_out[i].pos.x += weight_smooth
					* (smoothPath_out[i - 1].pos.x + smoothPath_out[i + 1].pos.x
							- (2.0 * smoothPath_out[i].pos.x));
			smoothPath_out[i].pos.y += weight_smooth
					* (smoothPath_out[i - 1].pos.y + smoothPath_out[i + 1].pos.y
							- (2.0 * smoothPath_out[i].pos.y));

			change += fabs(xtemp - smoothPath_out[i].pos.x);
			change += fabs(ytemp - smoothPath_out[i].pos.y);

		}
		nIterations++;
	}

	path = smoothPath_out;
}

void PlanningHelpers::SmoothPath(vector<GPSPoint>& path, double weight_data,
		double weight_smooth, double tolerance)
{

	if ((int)path.size() <= 2 )
		return;

	vector<GPSPoint> path_in = path;
	vector<GPSPoint> smoothPath_out = path;

	double change = tolerance;
	double xtemp, ytemp;
	int nIterations = 0;

	int size = (int)path_in.size();

	while (change >= tolerance)
	{
		change = 0.0;
		for (int i = 1; i < size - 1; i++)
		{
			xtemp = smoothPath_out[i].x;
			ytemp = smoothPath_out[i].y;

			smoothPath_out[i].x += weight_data
					* (path_in[i].x - smoothPath_out[i].x);
			smoothPath_out[i].y += weight_data
					* (path_in[i].y - smoothPath_out[i].y);

			smoothPath_out[i].x += weight_smooth
					* (smoothPath_out[i - 1].x + smoothPath_out[i + 1].x
							- (2.0 * smoothPath_out[i].x));
			smoothPath_out[i].y += weight_smooth
					* (smoothPath_out[i - 1].y + smoothPath_out[i + 1].y
							- (2.0 * smoothPath_out[i].y));

			change += fabs(xtemp - smoothPath_out[i].x);
			change += fabs(ytemp - smoothPath_out[i].y);

		}
		nIterations++;
	}

	path = smoothPath_out;
}

void PlanningHelpers::PredictConstantTimeCostForTrajectory(std::vector<WayPoint>& path, const WayPoint& currPose, const double& minVelocity)
{
	if((int)path.size() == 0) return;

	for(unsigned int i = 0 ; i < path.size(); i++)
		path.at(i).timeCost = -1;

	if(currPose.v == 0 || currPose.v < minVelocity) return;

	RelativeInfo info;
	PlanningHelpers::GetRelativeInfo(path, currPose, info);

	double total_distance = 0;
	double accum_time = 0;

	path.at(info.iFront).timeCost = 0;
	if(info.iFront == 0 ) info.iFront++;

	for(unsigned int i=info.iFront; i<path.size(); i++)
	{
		total_distance += hypot(path.at(i).pos.x- path.at(i-1).pos.x,path.at(i).pos.y- path.at(i-1).pos.y);
		accum_time = total_distance/currPose.v;
		path.at(i).timeCost = accum_time;
	}
}

void PlanningHelpers::FixAngleOnly(std::vector<WayPoint>& path)
{
	if((int)path.size() <= 2) return;

	path[0].pos.a = Angle::FixNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x ));

	for(int j = 1; j < (int)path.size()-1; j++)
		path[j].pos.a 		= Angle::FixNegativeAngle(atan2(path[j+1].pos.y - path[j].pos.y, path[j+1].pos.x - path[j].pos.x ));

	int j = (int)path.size()-1;

	path[j].pos.a = path[j-1].pos.a;

	for(int j = 0; j < (int)path.size()-1; j++)
	{
		if(path.at(j).pos.x == path.at(j+1).pos.x && path.at(j).pos.y == path.at(j+1).pos.y)
			path.at(j).pos.a = path.at(j+1).pos.a;
	}
}

double PlanningHelpers::CalcAngleAndCost(vector<WayPoint>& path, const double& lastCost)
{
	if((int)path.size() < 2) return 0;
	if((int)path.size() == 2)
	{
		path[0].pos.a = Angle::FixNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x ));
		path[0].distanceCost = lastCost;
		path[1].pos.a = path[0].pos.a;
		path[1].distanceCost = path[0].distanceCost +  distance2points(path[0].pos, path[1].pos);
		return path[1].distanceCost;
	}

	path[0].pos.a = Angle::FixNegativeAngle(atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x ));
	path[0].distanceCost = lastCost;

	for(int j = 1; j < (int)path.size()-1; j++)
	{
		path[j].pos.a 		= Angle::FixNegativeAngle(atan2(path[j+1].pos.y - path[j].pos.y, path[j+1].pos.x - path[j].pos.x ));
		path[j].distanceCost 	= path[j-1].distanceCost +  distance2points(path[j-1].pos, path[j].pos);
	}

	int j = (int)path.size()-1;

	path[j].pos.a 		= path[j-1].pos.a;
	path[j].distanceCost 	= path[j-1].distanceCost + distance2points(path[j-1].pos, path[j].pos);

	for(int j = 0; j < (int)path.size()-1; j++)
	{
		if(path.at(j).pos.x == path.at(j+1).pos.x && path.at(j).pos.y == path.at(j+1).pos.y)
			path.at(j).pos.a = path.at(j+1).pos.a;
	}

	return path[j].distanceCost;
}

void PlanningHelpers::CalcAngleAndCurvatureCost(vector<WayPoint>& path)
{
	if((int)path.size() < 2) return;

	path[0].pos.a 	= atan2(path[1].pos.y - path[0].pos.y, path[1].pos.x - path[0].pos.x );

	double k = 0;
	GPSPoint center;

	for(unsigned int j = 1; j < path.size()-1; j++)
	{
		k =  CalcCircle(path[j-1].pos,path[j].pos, path[j+1].pos, center);
		if(k > 150.0 || std::isnan(k))
			k = 150.0;

		if(k<1.0)
			path[j].curvatureCost = 0;
		else
			path[j].curvatureCost = 1.0-1.0/k;

		path[j].pos.a 	= atan2(path[j+1].pos.y - path[j].pos.y, path[j+1].pos.x - path[j].pos.x );
	}
	unsigned int j = path.size()-1;

	path[0].curvatureCost    = path[1].curvatureCost;
	path[j].curvatureCost 	= path[j-1].curvatureCost;
	path[j].pos.a 	= path[j-1].pos.a;
	path[j].curvatureCost 	= path[j-1].curvatureCost ;
}

double PlanningHelpers::CalcCircle(const GPSPoint& pt1, const GPSPoint& pt2, const GPSPoint& pt3, GPSPoint& center)
{
	double yDelta_a= pt2.y - pt1.y;
	double xDelta_a= pt2.x - pt1.x;
	double yDelta_b= pt3.y - pt2.y;
	double xDelta_b= pt3.x - pt2.x;

	if (fabs(xDelta_a) <= 0.000000000001 && fabs(yDelta_b) <= 0.000000000001)
	{
		center.x= 0.5*(pt2.x + pt3.x);
		center.y= 0.5*(pt1.y + pt2.y);
		return distance2points(center,pt1);
	}

	double aSlope=yDelta_a/xDelta_a;
	double bSlope=yDelta_b/xDelta_b;
	if (fabs(aSlope-bSlope) <= 0.000000000001)
	{
		return 100000;
	}

	center.x= (aSlope*bSlope*(pt1.y - pt3.y) + bSlope*(pt1.x + pt2 .x) - aSlope*(pt2.x+pt3.x) )/(2.0* (bSlope-aSlope) );
	center.y = -1.0*(center.x - (pt1.x+pt2.x)/2.0)/aSlope +  (pt1.y+pt2.y)/2.0;

	return  distance2points(center,pt1);
}

// double PlanningHelpers::CalcCircleV2(const WayPoint& p1, const WayPoint& p2, const WayPoint& p3, WayPoint& center)
// {
// //	double max_slope = 999999.0;
// //	double min_slope = 1.0/max_slope;
// //
// //	double m1 = (p2.pos.y-p1.pos.y)/(p2.pos.x-p1.pos.x);
// //	double m1_perp = 0;
// //	if(isnan(m1) || isinf(m1) || isinf(fabs(m1)))
// //	{
// //		m1 = max_slope;
// //		m1_perp = 0;
// //	}
// //	else if(m1 > fabs(max_slope))
// //	{
// //		m1 = max_slope;
// //		m1_perp = -1.0/m1;
// //	}
// //	else if(m1 < fabs(min_slope))
// //	{
// //		m1 = min_slope;
// //		m1_perp = -1.0/m1;
// //	}
// //	else
// //	{
// //		m1_perp = -1.0/m1;
// //	}
// //
// //	double m2 = (p3.pos.y-p2.pos.y)/(p3.pos.x-p2.pos.x);
// //	double m2_perp = 0;
// //	if(isnan(m2) || isinf(m2) || isinf(fabs(m2)))
// //	{
// //		m2 = max_slope;
// //		m2_perp = 0;
// //	}
// //	else if(m2 > fabs(max_slope))
// //	{
// //		m2 = max_slope;
// //		m2_perp = -1.0/m2;
// //	}
// //	else if(m2 < fabs(min_slope))
// //	{
// //		m2 = min_slope;
// //		m2_perp = -1.0/m2;
// //	}
// //	else
// //	{
// //		m2_perp = -1.0/m2;
// //	}

// 	return 0;

// }

void PlanningHelpers::CalcDtLaneInfo(vector<WayPoint>& path)
{
	if((int)path.size() < 2) return;

	double r_limit = 90000000.0;

	path.at(0).rot.z = Angle::FixNegativeAngle(atan2(path.at(1).pos.y - path.at(0).pos.y, path.at(1).pos.x - path.at(0).pos.x));
	path.at(0).rot.w = r_limit;
	path.at(0).distanceCost = 0;
	path.at(0).rot.y = 0;

	for(int j = 1; j < (int)path.size()-1; j++)
	{
		GPSPoint center;
		double r =  CalcCircle(path.at(j+1).pos,path.at(j).pos, path.at(j-1).pos, center);
		if(r > r_limit || std::isnan(r) || std::isinf(fabs(r)))
			path.at(j).rot.w = r_limit;
		else
			path.at(j).rot.w = r;

		path.at(j).rot.z = Angle::FixNegativeAngle(atan2(path.at(j+1).pos.y - path.at(j).pos.y, path.at(j+1).pos.x - path.at(j).pos.x ));
		double d = hypot(path.at(j).pos.y - path.at(j-1).pos.y, path.at(j).pos.x - path.at(j-1).pos.x);
		double z_diff = path.at(j).pos.z - path.at(j-1).pos.z;
		double a = sqrt((d*d) - (z_diff*z_diff));

		path.at(j).distanceCost = path.at(j-1).distanceCost + d;
		if(a != 0)
		{
			path.at(j).rot.y = (z_diff/a) * 100.0; // percentile road slope calculation
		}

	}

	int j = (int)path.size()-1;

	if((int)path.size() > 2)
		path.at(0).rot.w = path.at(1).rot.w;

	path.at(0).rot.y = path.at(1).rot.y;

	path.at(j).rot.z = path.at(j-1).rot.z;
	path.at(j).rot.w = path.at(j-1).rot.w;

	double d = hypot(path.at(j).pos.y - path.at(j-1).pos.y, path.at(j).pos.x - path.at(j-1).pos.x);
	double z_diff = path.at(j).pos.z - path.at(j-1).pos.z;
	double a = sqrt(d*d - z_diff*z_diff);

	path.at(j).distanceCost = path.at(j-1).distanceCost + d;

	if(a != 0)
	{
		path.at(j).rot.y = (z_diff/a) * 100.0;
	}
}

int PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(const vector<WayPoint>& originalPath, const WayPoint& pos, const double& minDistance,
		const double& pathDensity, vector<WayPoint>& extractedPath, int prev_index)
{
	if((int)originalPath.size() < 2 ) return 0;

	extractedPath.clear();

	int close_index = GetClosestNextPointIndexDirectionFast(originalPath, pos, prev_index);
	double d = 0;

	if(close_index + 1 >= (int)originalPath.size())
		close_index = (int)originalPath.size() - 2;

	for(int i=close_index; i >=  0; i--)
	{
		extractedPath.insert(extractedPath.begin(),  originalPath.at(i));
		if(i < (int)originalPath.size())
			d += hypot(originalPath.at(i).pos.y - originalPath.at(i+1).pos.y, originalPath.at(i).pos.x - originalPath.at(i+1).pos.x);
		if(d > 10)
			break;
	}

	//extractedPath.push_back(info.perp_point);
	d = 0;
	for(int i=close_index+1; i < (int)originalPath.size(); i++)
	{
		extractedPath.push_back(originalPath.at(i));
		if(i > 0)
			d += hypot(originalPath.at(i).pos.y - originalPath.at(i-1).pos.y, originalPath.at(i).pos.x - originalPath.at(i-1).pos.x);
		if(d > minDistance)
			break;
	}

	if((int)extractedPath.size() < 2)
	{
		cout << endl << "### Planner Z . Extracted Rollout Path is too Small, Size = " << extractedPath.size() << endl;
		return close_index;
	}

	FixPathDensity(extractedPath, pathDensity);
	CalcAngleAndCost(extractedPath);

	return close_index;
}

void PlanningHelpers::ExtractPartFromPointToDistanceFast(const vector<WayPoint>& originalPath, const WayPoint& pos, const double& minDistance,
		const double& pathDensity, vector<WayPoint>& extractedPath)
{
	extractedPath.clear();
	RelativeInfo info;
	GetRelativeInfo(originalPath, pos, info);
	double d = 0;
	if(info.iBack > 0)
		info.iBack--;

	for(int i=info.iBack; i >=  0; i--)
	{
		extractedPath.insert(extractedPath.begin(),  originalPath.at(i));
		if(i < (int)originalPath.size())
			d += hypot(originalPath.at(i).pos.y - originalPath.at(i+1).pos.y, originalPath.at(i).pos.x - originalPath.at(i+1).pos.x);
		if(d > 10)
			break;
	}

	//extractedPath.push_back(info.perp_point);
	d = 0;
	for(int i=info.iBack+1; i < (int)originalPath.size(); i++)
	{
		extractedPath.push_back(originalPath.at(i));
		if(i > 0)
			d += hypot(originalPath.at(i).pos.y - originalPath.at(i-1).pos.y, originalPath.at(i).pos.x - originalPath.at(i-1).pos.x);
		if(d > minDistance)
			break;
	}

	if((int)extractedPath.size() < 2)
	{
		cout << endl << "### Planner Z . Extracted Rollout Path is too Small, Size = " << extractedPath.size() << endl;
		return;
	}

	FixPathDensity(extractedPath, pathDensity);
	CalcAngleAndCost(extractedPath);
}

void PlanningHelpers::CalculateRollInTrajectories(const WayPoint& carPos, const double& speed, const vector<WayPoint>& originalCenter, int& start_index,
		int& end_index, vector<double>& end_laterals ,
		vector<vector<WayPoint> >& rollInPaths, const double& max_roll_distance,
		const double&  carTipMargin, const double& rollInMargin,
		const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
		const int& rollOutsNumber, const double& SmoothDataWeight, const double& SmoothWeight,
		const double& SmoothTolerance, const bool& bHeadingSmooth,
		std::vector<WayPoint>& sampledPoints)
{
	WayPoint p;

	//Get Closest Index
	int close_index = -1;
	RelativeInfo info;
	GetRelativeInfoDirection(originalCenter, carPos, info);
	close_index = info.iBack;
	double remaining_distance = 0;
	for(unsigned int i=close_index; i< originalCenter.size()-1; i++)
	{
		if(i>0)
		{
			remaining_distance += distance2points(originalCenter[i].pos, originalCenter[i+1].pos);
		}
	}

	int nRollOuts = rollOutsNumber;
	if(originalCenter.at(close_index).custom_type == CUSTOM_AVOIDANCE_DISABLED)
	{
		nRollOuts = 0;
	}

	double initial_roll_in_distance = info.perp_distance ; //GetPerpDistanceToTrajectorySimple(originalCenter, carPos, close_index);

	vector<WayPoint> RollOutStratPath;
	///***   Smoothing From Car Heading Section ***///
	if(bHeadingSmooth)
	{
		unsigned int num_of_strait_points = carTipMargin*0.75 / pathDensity;
		int closest_for_each_iteration = 0;
		RelativeInfo ret_inf;
		GetRelativeInfo(originalCenter, carPos, ret_inf, closest_for_each_iteration);
		ret_inf.perp_point.pos.x = carPos.pos.x;
		ret_inf.perp_point.pos.y = carPos.pos.y;
		ret_inf.perp_point.pos.a = carPos.pos.a;

		RollOutStratPath.push_back(ret_inf.perp_point);
		for(unsigned int i = 0; i < num_of_strait_points; i++)
		{
			p = RollOutStratPath.at(i);
			p.pos.x = p.pos.x +  pathDensity*cos(p.pos.a);
			p.pos.y = p.pos.y +  pathDensity*sin(p.pos.a);
			GetRelativeInfo(originalCenter, p, ret_inf, closest_for_each_iteration);
			ret_inf.perp_point.pos = p.pos;
			RollOutStratPath.push_back(ret_inf.perp_point);
		}

		GetRelativeInfo(originalCenter, RollOutStratPath.at((int)RollOutStratPath.size()-1), ret_inf, close_index);
		initial_roll_in_distance = ret_inf.perp_distance;
	}
	///***   -------------------------------- ***///

	//calculate the starting index
	double d_limit = 0;
	unsigned int far_index = close_index;

	//calculate end index
	double start_distance = rollInSpeedFactor*speed+rollInMargin;
	if(start_distance > remaining_distance)
	{
		start_distance = remaining_distance;
	}

	d_limit = 0;
	for(unsigned int i=close_index; i< originalCenter.size(); i++)
	  {
		  if(i>0)
		  {
			  d_limit += distance2points(originalCenter[i].pos, originalCenter[i-1].pos);
		  }

		  if(d_limit >= start_distance)
		  {
			  far_index = i;
			  break;
		  }
	  }

	int centralTrajectoryIndex = nRollOuts/2;
	vector<double> end_distance_list;
	for(int i=0; i< nRollOuts+1; i++)
	{
		double end_roll_in_distance = rollOutDensity*(i - centralTrajectoryIndex);
		end_distance_list.push_back(end_roll_in_distance);
	}

	start_index = close_index;
	end_index = far_index;
	end_laterals = end_distance_list;

	//calculate the actual calculation starting index
	d_limit = 0;
	unsigned int smoothing_start_index = start_index;
	unsigned int smoothing_end_index = end_index;

	for(unsigned int i=smoothing_start_index; i< originalCenter.size(); i++)
	{
		if(i > 0)
		{
			d_limit += distance2points(originalCenter[i].pos, originalCenter[i-1].pos);
		}
		if(d_limit > carTipMargin)
		{
			break;
		}

		smoothing_start_index++;
	}

	d_limit = 0;
	for(unsigned int i=end_index; i< originalCenter.size(); i++)
	{
		if(i > 0)
		{
			d_limit += distance2points(originalCenter[i].pos, originalCenter[i-1].pos);
		}

		if(d_limit > carTipMargin)
		{
			break;
		}

		smoothing_end_index++;
	}

	int nSteps = end_index - smoothing_start_index;

	vector<double> inc_list;
	rollInPaths.clear();
	vector<double> inc_list_inc;
	for(int i=0; i< nRollOuts+1; i++)
	{
		double diff = end_laterals.at(i)-initial_roll_in_distance;
		inc_list.push_back(diff/(double)nSteps);
		rollInPaths.push_back(vector<WayPoint>());
		inc_list_inc.push_back(0);
	}

	vector<vector<WayPoint> > execluded_from_smoothing;
	for(int i=0; i< nRollOuts+1 ; i++)
	{
		execluded_from_smoothing.push_back(vector<WayPoint>());
	}

	//Insert First strait points within the tip of the car range
	int iLimitIndex = (carTipMargin/0.3)/pathDensity;
	if(iLimitIndex >= (int)originalCenter.size())
	{
		iLimitIndex = (int)originalCenter.size() - 1;
	}

	for(unsigned int j = start_index; j < smoothing_start_index; j++)
	{
		p = originalCenter.at(j);
		double original_speed = p.v;
		for(int i=0; i< nRollOuts+1 ; i++)
		{
			p.pos.x = originalCenter.at(j).pos.x -  initial_roll_in_distance*cos(p.pos.a + M_PI_2);
			p.pos.y = originalCenter.at(j).pos.y -  initial_roll_in_distance*sin(p.pos.a + M_PI_2);
			if((int)i!=centralTrajectoryIndex)
			{
				p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
			}
			else
			{
				p.v = original_speed;
			}

			if((int)j < iLimitIndex)
			{
				execluded_from_smoothing.at(i).push_back(p);
			}
			else
			{
				rollInPaths.at(i).push_back(p);
			}
			sampledPoints.push_back(p);
		}
	}

	for(int j = smoothing_start_index; j < end_index; j++)
	{
		p = originalCenter.at(j);
		double original_speed = p.v;

		for(int i=0; i< nRollOuts+1 ; i++)
		{
			inc_list_inc[i] += inc_list[i];
			double d = inc_list_inc[i];
			p.pos.x = originalCenter.at(j).pos.x -  initial_roll_in_distance*cos(p.pos.a + M_PI_2) - d*cos(p.pos.a+ M_PI_2);
			p.pos.y = originalCenter.at(j).pos.y -  initial_roll_in_distance*sin(p.pos.a + M_PI_2) - d*sin(p.pos.a+ M_PI_2);
			if((int)i!=centralTrajectoryIndex)
			{
				p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
			}
			else
			{
				p.v = original_speed ;
			}

			rollInPaths.at(i).push_back(p);
			sampledPoints.push_back(p);
		}
	}

	//Insert last strait points to make better smoothing
	for(unsigned int j = end_index; j < smoothing_end_index; j++)
	{
		p = originalCenter.at(j);
		double original_speed = p.v;
		for(int i=0; i< nRollOuts+1 ; i++)
		{
			double d = end_laterals.at(i);
			p.pos.x  = originalCenter.at(j).pos.x - d*cos(p.pos.a + M_PI_2);
			p.pos.y  = originalCenter.at(j).pos.y - d*sin(p.pos.a + M_PI_2);
			if((int)i!=centralTrajectoryIndex)
			{
				p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
			}
			else
			{
				p.v = original_speed;
			}

			rollInPaths.at(i).push_back(p);
			sampledPoints.push_back(p);
		}
	}

	for(int i=0; i< nRollOuts+1 ; i++)
	{
		rollInPaths.at(i).insert(rollInPaths.at(i).begin(), execluded_from_smoothing.at(i).begin(), execluded_from_smoothing.at(i).end());
	}

	///***   Smoothing From Car Heading Section ***///
	if(bHeadingSmooth)
	{
		for(int i=0; i< nRollOuts+1 ; i++)
		{
			unsigned int cut_index = GetClosestNextPointIndexFastV2(rollInPaths.at(i), RollOutStratPath.at(RollOutStratPath.size()-1));
			rollInPaths.at(i).erase(rollInPaths.at(i).begin(), rollInPaths.at(i).begin()+cut_index);
			rollInPaths.at(i).insert(rollInPaths.at(i).begin(), RollOutStratPath.begin(), RollOutStratPath.end());
		}
	}
	///***   -------------------------------- ***///

	d_limit = 0;
	for(unsigned int j = smoothing_end_index; j < originalCenter.size(); j++)
	{
		if(j > 0)
		{
			d_limit += distance2points(originalCenter.at(j).pos, originalCenter.at(j-1).pos);
		}

		if(d_limit > max_roll_distance)
		{
			break;
		}

		p = originalCenter.at(j);
		double original_speed = p.v;
		for(unsigned int i=0; i< rollInPaths.size() ; i++)
		{
			double d = end_laterals.at(i);
			p.pos.x  = originalCenter.at(j).pos.x - d*cos(p.pos.a + M_PI_2);
			p.pos.y  = originalCenter.at(j).pos.y - d*sin(p.pos.a + M_PI_2);

			if((int)i!=centralTrajectoryIndex)
			{
				p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
			}
			else
			{
				p.v = original_speed;
			}

			rollInPaths.at(i).push_back(p);

			sampledPoints.push_back(p);
		}
	}

	for(int i=0; i< nRollOuts+1 ; i++)
	{
		SmoothPath(rollInPaths.at(i), SmoothDataWeight, SmoothWeight, SmoothTolerance);
		CalcAngleAndCost(rollInPaths.at(i));
	}
}

bool PlanningHelpers::FindInList(const std::vector<int>& list,const int& x)
{
	for(unsigned int i = 0 ; i < list.size(); i++)
	{
		if(list.at(i) == x)
			return true;
	}
	return false;
}

std::vector<int> PlanningHelpers::GetUniqueLeftRightIds(const std::vector<WayPoint>& path)
{
	 vector<int> sideLanes;
	for(unsigned int iwp = 0; iwp < path.size(); iwp++)
	 {
		 if(path.at(iwp).LeftPointId>0)
		 {
			 bool bFound = false;
			 for(unsigned int is = 0 ; is < sideLanes.size(); is++)
			 {
				 if(sideLanes.at(is) == path.at(iwp).LeftPointId)
				 {
					 bFound = true;
					 break;
				 }
			 }

			 if(!bFound)
				 sideLanes.push_back(path.at(iwp).LeftPointId);
		 }

		 if(path.at(iwp).RightPointId>0)
		 {
			 bool bFound = false;
			 for(unsigned int is = 0 ; is < sideLanes.size(); is++)
			 {
				 if(sideLanes.at(is) == path.at(iwp).RightPointId)
				 {
					 bFound = true;
					 break;
				 }
			 }

			 if(!bFound)
				 sideLanes.push_back(path.at(iwp).RightPointId);
		 }
	 }
	return sideLanes;
}

void PlanningHelpers::SmoothSpeedProfiles(vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance	)
{

	if ((int)path_in.size() <= 1)
		return;
	vector<WayPoint> newpath = path_in;

	double change = tolerance;
	double xtemp;
	int nIterations = 0;
	int size = (int)newpath.size();

	while (change >= tolerance)
	{
		change = 0.0;
		for (int i = 1; i < size -1; i++)
		{
			xtemp = newpath[i].v;
			newpath[i].v += weight_data * (path_in[i].v - newpath[i].v);
			newpath[i].v += weight_smooth * (newpath[i - 1].v + newpath[i + 1].v - (2.0 * newpath[i].v));
			change += fabs(xtemp - newpath[i].v);

		}
		nIterations++;
	}

	path_in = newpath;
}

void PlanningHelpers::SmoothCurvatureProfiles(vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance)
{
	if ((int)path_in.size() <= 1)
	{
		return;
	}

	vector<WayPoint> newpath = path_in;
	double change = tolerance;
	double xtemp;
	int nIterations = 0;
	int size = (int)newpath.size();

	while (change >= tolerance)
	{
		change = 0.0;
		for (int i = 1; i < size -1; i++)
		{
			xtemp = newpath[i].curvatureCost;
			newpath[i].curvatureCost += weight_data * (path_in[i].curvatureCost - newpath[i].curvatureCost);
			newpath[i].curvatureCost += weight_smooth * (newpath[i - 1].curvatureCost + newpath[i + 1].curvatureCost - (2.0 * newpath[i].curvatureCost));
			change += fabs(xtemp - newpath[i].curvatureCost);

		}
		nIterations++;
	}
	path_in = newpath;
}

void PlanningHelpers::SmoothWayPointsDirections(vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance	)
{

	if ((int)path_in.size() <= 1)
		return;

	vector<WayPoint> newpath = path_in;

	double change = tolerance;
	double xtemp;
	int nIterations = 0;
	int size = (int)newpath.size();

	while (change >= tolerance)
	{
		change = 0.0;
		for (int i = 1; i < size -1; i++)
		{
			xtemp = newpath[i].pos.a;
			newpath[i].pos.a += weight_data * (path_in[i].pos.a - newpath[i].pos.a);
			newpath[i].pos.a += weight_smooth * (newpath[i - 1].pos.a + newpath[i + 1].pos.a - (2.0 * newpath[i].pos.a));
			change += fabs(xtemp - newpath[i].pos.a);

		}
		nIterations++;
	}
	path_in = newpath;
}

void PlanningHelpers::SmoothZ(vector<WayPoint>& path_in, double weight_data, double weight_smooth, double tolerance	)
{

	if ((int)path_in.size() <= 1)
		return;

	vector<WayPoint> newpath = path_in;

	double change = tolerance;
	double xtemp;
	int nIterations = 0;
	int size = (int)newpath.size();

	while (change >= tolerance)
	{
		change = 0.0;
		for (int i = 1; i < size -1; i++)
		{
			xtemp = newpath[i].pos.z;
			newpath[i].pos.z += weight_data * (path_in[i].pos.z - newpath[i].pos.z);
			newpath[i].pos.z += weight_smooth * (newpath[i - 1].pos.z + newpath[i + 1].pos.z - (2.0 * newpath[i].pos.z));
			change += fabs(xtemp - newpath[i].pos.z);

		}
		nIterations++;
	}
	path_in = newpath;
}

void PlanningHelpers::SmoothGlobalPathSpeed(vector<WayPoint>& path)
{
	CalcAngleAndCurvatureCost(path);
	SmoothSpeedProfiles(path, 0.45,0.25, 0.01);
}

void PlanningHelpers::ShiftRecommendedSpeed(std::vector<WayPoint>& path, const double& max_speed, const double& curr_speed, const double& inc_ratio, const double& path_density)
{
	if(max_speed == 0 || curr_speed == 0) return;

	double shift_distance = max_speed*0.5;
	// double curr_speed_ratio = (2.0 * curr_speed) / max_speed;

	if(curr_speed < 2)
	{
		shift_distance += 0;
	}
	else if(curr_speed < 4)
	{
		shift_distance += 4*inc_ratio;
	}
	else if(curr_speed < 6)
	{
		shift_distance += 6*inc_ratio;
	}
	else if(curr_speed < 8)
	{
		shift_distance += 8*inc_ratio;
	}
	else if(curr_speed < 10)
	{
		shift_distance += 10*inc_ratio;
	}
	else if(curr_speed < 15)
	{
		shift_distance += 15*inc_ratio;
	}
	else
	{
		shift_distance += 20*inc_ratio;
	}

	int shift_index = shift_distance / path_density;

	for(unsigned int i=0; i < path.size()-1; i++)
	{
		if(path.at(i).v < path.at(i+1).v)
		{
			int vel_min = path.at(i).v;
			if((int)i < shift_index)
			{
				shift_index = i;
			}

			for(unsigned int j=i-shift_index; j<i; j++)
			{
				path.at(j).v = vel_min;
			}
		}
	}

	SmoothSpeedProfiles(path, 0.4,0.3, 0.01);
}

void PlanningHelpers::GenerateRecommendedSpeed(vector<WayPoint>& path, const double& max_speed, const double& speedProfileFactor)
{
	CalcAngleAndCurvatureCost(path);
	SmoothCurvatureProfiles(path, 0.4, 0.3, 0.01);
	double v = 0;

	for(unsigned int i = 0 ; i < path.size(); i++)
	{
		double k_ratio = path.at(i).curvatureCost*10.0;

		double local_max = path.at(i).v;
		if(local_max == 0)
			local_max = max_speed;
		else if(local_max > max_speed)
			local_max = max_speed;

		if(k_ratio >= 9.5)
			v = local_max;
		else if(k_ratio <= 8.5)
			v = 1.0*speedProfileFactor;
		else
		{
			k_ratio = k_ratio - 8.5;
			v = (local_max - 1.0) * k_ratio + 1.0;
			v = v*speedProfileFactor;
		}

		if(v > local_max)
			path.at(i).v = local_max;
		else
			path.at(i).v = v;

	}

	SmoothSpeedProfiles(path, 0.4,0.3, 0.01);
}

double PlanningHelpers::GetACCVelocityModelBased(const double& dt, const double& CurrSpeed, const CAR_BASIC_INFO& vehicleInfo,
		const ControllerParams& ctrlParams, const BehaviorState& CurrBehavior)
{
	double desiredVel = 0;

	if(CurrBehavior.state == FORWARD_STATE || CurrBehavior.state == OBSTACLE_AVOIDANCE_STATE )
	{
		double acceleration_critical = vehicleInfo.max_acceleration * ctrlParams.accelPushRatio;

		if(CurrBehavior.maxVelocity < CurrSpeed)
		{
			acceleration_critical = vehicleInfo.max_deceleration * ctrlParams.brakePushRatio;
		}

		double incr_vel = acceleration_critical * dt;

		if(CurrSpeed < 1.0 && CurrBehavior.maxVelocity > 1.0)
		{
			incr_vel += 1.0;
		}

		desiredVel = incr_vel + CurrSpeed;

		//std::cout << "Forward: max_velocity: " <<  CurrBehavior.maxVelocity << ", currSpeed: " << CurrSpeed << ", desiredVel: " << desiredVel << ", acceleration: " << acceleration_critical << std::endl;
	}
	else if(CurrBehavior.state == STOPPING_STATE || CurrBehavior.state == TRAFFIC_LIGHT_STOP_STATE || CurrBehavior.state == STOP_SIGN_STOP_STATE)
	{
		double deceleration_critical = vehicleInfo.max_deceleration;
		double distance_to_stop = CurrBehavior.stopDistance ;
		if(distance_to_stop != 0)
		{
			deceleration_critical = (-CurrSpeed*CurrSpeed)/(2.0*distance_to_stop);
		}

		deceleration_critical = deceleration_critical * ctrlParams.brakePushRatio;

		desiredVel = (deceleration_critical * dt) + CurrSpeed;
		if(CurrSpeed < 1.0)
		{
			desiredVel = 0;
		}

		//std::cout << "STOP: stop_distance: " <<  distance_to_stop << ", desiredVel: " << desiredVel << ", Deceleration: " << deceleration_critical << ", dt: " << dt << std::endl;
	}
	else if(CurrBehavior.state == FOLLOW_STATE)
	{
		double crash_d = CurrBehavior.followDistance;
		double safe_d = CurrBehavior.stopDistance;
		// double min_follow_distance = ctrlParams.min_safe_follow_distance + CurrSpeed;
		double diff = crash_d - safe_d;
		double target_a = 0;

		/**
		 * Following Conditions
		 */
		if(diff < ctrlParams.min_safe_follow_distance)
		{
			double brake_distance = crash_d - ctrlParams.min_safe_follow_distance;

			if(brake_distance > 0)
			{
				target_a = (-CurrSpeed*CurrSpeed)/(2.0*brake_distance);
			}
			else
			{
				target_a = -9.8*4; //stop with -4G
			}
		}
		else if(diff > (ctrlParams.min_safe_follow_distance + CurrSpeed))
		{
			target_a = vehicleInfo.max_acceleration;
		}

		/**
		 * When in a curve , should driver slower then followed vehicle
		 */
		if(CurrSpeed > CurrBehavior.maxVelocity && target_a > vehicleInfo.max_deceleration)
		{

			target_a = vehicleInfo.max_deceleration;
		}

		/**
		 * Apply acceleration push factors
		 */
		if(target_a > 0)
		{
			target_a = target_a * ctrlParams.accelPushRatio;
		}
		else
		{
			target_a = target_a * ctrlParams.brakePushRatio;
		}

		desiredVel = (target_a * dt) + CurrSpeed;

		if(CurrSpeed < 1.0 && CurrBehavior.maxVelocity > 1.0 && desiredVel > 0)
		{
			desiredVel += 1.0;
		}

		//std::cout << "Follow: safe_d: " <<  safe_d << ", follow_d: " << crash_d << ", diff: " << diff << ", accel-decel: " << target_a << ", desiredVel: " << desiredVel << std::endl;
	}
	else
	{
		desiredVel = 0;
	}

	if(desiredVel > CurrBehavior.maxVelocity)
	{
		desiredVel = CurrBehavior.maxVelocity;
	}

	if(desiredVel < 0)
	{
		desiredVel = 0;
	}

	return desiredVel;
}

WayPoint* PlanningHelpers::BuildPlanningSearchTreeV2(WayPoint* pStart,
		const WayPoint& goalPos,
		const vector<int>& globalPath,
		const double& DistanceLimit,
		const bool& bEnableLaneChange,
		vector<WayPoint*>& all_cells_to_delete)
{
	if(!pStart) return NULL;

	vector<pair<WayPoint*, WayPoint*> >nextLeafToTrace;

	WayPoint* pZero = 0;
	WayPoint* wp    = new WayPoint();
	*wp = *pStart;
	nextLeafToTrace.push_back(make_pair(pZero, wp));
	all_cells_to_delete.push_back(wp);

	double 		distance 		= 0;
	double 		before_change_distance	= 0;
	WayPoint* 	pGoalCell 		= 0;
	double 		nCounter 		= 0;


	while((int)nextLeafToTrace.size()>0)
	{
		nCounter++;

		unsigned int min_cost_index = 0;
		double min_cost = DBL_MAX;

		for(unsigned int i=0; i < nextLeafToTrace.size(); i++)
		{
			if(nextLeafToTrace.at(i).second->cost < min_cost)
			{
				min_cost = nextLeafToTrace.at(i).second->cost;
				min_cost_index = i;
			}
		}

		WayPoint* pH 	= nextLeafToTrace.at(min_cost_index).second;

		assert(pH != 0);

		nextLeafToTrace.erase(nextLeafToTrace.begin()+min_cost_index);

		double distance_to_goal = distance2points(pH->pos, goalPos.pos);
		double angle_to_goal = Angle::AngleBetweenTwoAnglesPositive(Angle::FixNegativeAngle(pH->pos.a), Angle::FixNegativeAngle(goalPos.pos.a));
		if( distance_to_goal <= 0.1 && angle_to_goal < M_PI_4)
		{
			cout << "Goal Found, LaneID: " << pH->laneId <<", Distance : " << distance_to_goal << ", Angle: " << angle_to_goal*RAD2DEG << endl;
			pGoalCell = pH;
			break;
		}
		else
		{

			if(pH->pLeft && !CheckLaneExits(all_cells_to_delete, pH->pLeft->pLane) && !CheckNodeExits(all_cells_to_delete, pH->pLeft) && bEnableLaneChange && before_change_distance > LANE_CHANGE_MIN_DISTANCE)
			{
				wp = new WayPoint();
				*wp = *pH->pLeft;
				double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
				distance += d;
				before_change_distance = -LANE_CHANGE_MIN_DISTANCE*2;

				for(unsigned int a = 0; a < wp->actionCost.size(); a++)
				{
					//if(wp->actionCost.at(a).first == LEFT_TURN_ACTION)
						d += wp->actionCost.at(a).second;
				}

				wp->cost = pH->cost + d;
				wp->pRight = pH;
				wp->pLeft = 0;

				nextLeafToTrace.push_back(make_pair(pH, wp));
				all_cells_to_delete.push_back(wp);
			}

			if(pH->pRight && !CheckLaneExits(all_cells_to_delete, pH->pRight->pLane) && !CheckNodeExits(all_cells_to_delete, pH->pRight) && bEnableLaneChange && before_change_distance > LANE_CHANGE_MIN_DISTANCE)
			{
				wp = new WayPoint();
				*wp = *pH->pRight;
				double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
				distance += d;
				before_change_distance = -LANE_CHANGE_MIN_DISTANCE*2;

				for(unsigned int a = 0; a < wp->actionCost.size(); a++)
				{
					//if(wp->actionCost.at(a).first == RIGHT_TURN_ACTION)
						d += wp->actionCost.at(a).second;
				}

				wp->cost = pH->cost + d;
				wp->pLeft = pH;
				wp->pRight = 0;
				nextLeafToTrace.push_back(make_pair(pH, wp));
				all_cells_to_delete.push_back(wp);
			}

			for(unsigned int i =0; i< pH->pFronts.size(); i++)
			{
				if(CheckLaneIdExits(globalPath, pH->pLane) && pH->pFronts.at(i) && !CheckNodeExits(all_cells_to_delete, pH->pFronts.at(i)))
				{
					wp = new WayPoint();
					*wp = *pH->pFronts.at(i);

					double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
					distance += d;
					before_change_distance += d;

					for(unsigned int a = 0; a < wp->actionCost.size(); a++)
					{
						//if(wp->actionCost.at(a).first == FORWARD_ACTION)
							d += wp->actionCost.at(a).second;
					}

					wp->cost = pH->cost + d;
					wp->pBacks.push_back(pH);

					nextLeafToTrace.push_back(make_pair(pH, wp));
					all_cells_to_delete.push_back(wp);
				}
			}
		}

		if(distance > DistanceLimit && (int)globalPath.size()==0)
		{
			//if(!pGoalCell)
			cout << "Goal Not Found, LaneID: " << pH->laneId <<", Distance : " << distance << endl;
			pGoalCell = pH;
			break;
		}

		//pGoalCell = pH;
	}

	while((int)nextLeafToTrace.size() != 0)
		nextLeafToTrace.pop_back();
	//closed_nodes.clear();

	return pGoalCell;
}

WayPoint* PlanningHelpers::BuildPlanningSearchTreeStraight(WayPoint* pStart,
		const double& DistanceLimit,
		vector<WayPoint*>& all_cells_to_delete)
{
	if(!pStart) return NULL;

	vector<pair<WayPoint*, WayPoint*> >nextLeafToTrace;

	WayPoint* pZero = 0;
	WayPoint* wp    = new WayPoint();
	*wp = *pStart;
	wp->cost = 0;
	nextLeafToTrace.push_back(make_pair(pZero, wp));
	all_cells_to_delete.push_back(wp);

	double 		distance 		= 0;
	WayPoint* 	pGoalCell 		= 0;
	double 		nCounter 		= 0;

	while((int)nextLeafToTrace.size()>0)
	{
		nCounter++;

		unsigned int min_cost_index = 0;
		double min_cost = DBL_MAX;

		for(unsigned int i=0; i < nextLeafToTrace.size(); i++)
		{
			if(nextLeafToTrace.at(i).second->cost < min_cost)
			{
				min_cost = nextLeafToTrace.at(i).second->cost;
				min_cost_index = i;
			}
		}

		WayPoint* pH 	= nextLeafToTrace.at(min_cost_index).second;
		assert(pH != 0);

		nextLeafToTrace.erase(nextLeafToTrace.begin()+min_cost_index);

		for(unsigned int i =0; i< pH->pFronts.size(); i++)
		{
			if(pH->pFronts.at(i) && !CheckNodeExits(all_cells_to_delete, pH->pFronts.at(i)))
			{
				wp = new WayPoint();
				*wp = *pH->pFronts.at(i);

				double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
				distance += d;

//				for(unsigned int a = 0; a < wp->actionCost.size(); a++)
//				{
//					//if(wp->actionCost.at(a).first == FORWARD_ACTION)
//						d += wp->actionCost.at(a).second;
//				}

				wp->cost = pH->cost + d;
				wp->pBacks.push_back(pH);
				if(wp->cost < DistanceLimit)
				{
					nextLeafToTrace.push_back(make_pair(pH, wp));
					all_cells_to_delete.push_back(wp);
				}
				else
					delete wp;
			}
		}

//		if(pH->pLeft && !CheckLaneExits(all_cells_to_delete, pH->pLeft->pLane))
//		{
//			wp = new WayPoint();
//			*wp = *pH->pLeft;
//			double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
//
//			for(unsigned int a = 0; a < wp->actionCost.size(); a++)
//			{
//				//if(wp->actionCost.at(a).first == LEFT_TURN_ACTION)
//					d += wp->actionCost.at(a).second;
//			}
//
//			wp->cost = pH->cost + d + LANE_CHANGE_COST;
//			wp->pRight = pH;
//			wp->pRight = 0;
//
//			nextLeafToTrace.push_back(make_pair(pH, wp));
//			all_cells_to_delete.push_back(wp);
//		}
//
//		if(pH->pRight && !CheckLaneExits(all_cells_to_delete, pH->pRight->pLane))
//		{
//			wp = new WayPoint();
//			*wp = *pH->pRight;
//			double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);;
//
//			for(unsigned int a = 0; a < wp->actionCost.size(); a++)
//			{
//				//if(wp->actionCost.at(a).first == RIGHT_TURN_ACTION)
//					d += wp->actionCost.at(a).second;
//			}
//
//			wp->cost = pH->cost + d + LANE_CHANGE_COST;
//			wp->pLeft = pH;
//			wp->pRight = 0;
//			nextLeafToTrace.push_back(make_pair(pH, wp));
//			all_cells_to_delete.push_back(wp);
//		}

		pGoalCell = pH;
	}

	while((int)nextLeafToTrace.size()!=0)
		nextLeafToTrace.pop_back();

	return pGoalCell;
}

int PlanningHelpers::PredictiveIgnorIdsDP(WayPoint* pStart, const double& DistanceLimit,
		vector<WayPoint*>& all_cells_to_delete,vector<WayPoint*>& end_waypoints, std::vector<int>& lanes_ids)
{
	if(!pStart) return 0;

		vector<pair<WayPoint*, WayPoint*> >nextLeafToTrace;

		WayPoint* pZero = 0;
		WayPoint* wp    = new WayPoint();
		*wp = *pStart;
		wp->cost = 0;
		wp->pLeft = 0;
		wp->pRight = 0;
		nextLeafToTrace.push_back(make_pair(pZero, wp));
		all_cells_to_delete.push_back(wp);

		double 		distance 		= 0;
		end_waypoints.clear();
		double 		nCounter 		= 0;

		while((int)nextLeafToTrace.size()>0)
		{
			nCounter++;

			WayPoint* pH 	= nextLeafToTrace.at(0).second;

			assert(pH != 0);

			nextLeafToTrace.erase(nextLeafToTrace.begin()+0);

			for(unsigned int i =0; i< pH->pFronts.size(); i++)
			{
				if(pH->pFronts.at(i) && !CheckNodeExits(all_cells_to_delete, pH->pFronts.at(i)))
				{
					if(pH->cost < DistanceLimit)
					{
						wp = new WayPoint();
						*wp = *pH->pFronts.at(i);

						double d = distance2points(wp->pos, pH->pos);
						distance += d;
						wp->cost = pH->cost + d;
						wp->pBacks.push_back(pH);
						wp->pLeft = 0;
						wp->pRight = 0;

						bool bFoundLane = false;
						for(unsigned int k = 0 ; k < lanes_ids.size(); k++)
						{
							if(wp->laneId == lanes_ids.at(k))
							{
								bFoundLane = true;
								break;
							}
						}

						if(!bFoundLane)
							nextLeafToTrace.push_back(make_pair(pH, wp));
						all_cells_to_delete.push_back(wp);
					}
					else
					{
						end_waypoints.push_back(pH);
					}
				}
			}
		}

		while((int)nextLeafToTrace.size()!=0)
			nextLeafToTrace.pop_back();
		//closed_nodes.clear();

		return (int)end_waypoints.size();
}

int PlanningHelpers::PredictiveDP(WayPoint* pStart, const double& DistanceLimit,
		vector<WayPoint*>& all_cells_to_delete,vector<WayPoint*>& end_waypoints)
{
	if(!pStart) return 0;

	vector<pair<WayPoint*, WayPoint*> >nextLeafToTrace;

	WayPoint* pZero = 0;
	WayPoint* wp    = new WayPoint();
	*wp = *pStart;
	wp->pLeft = 0;
	wp->pRight = 0;
	nextLeafToTrace.push_back(make_pair(pZero, wp));
	all_cells_to_delete.push_back(wp);

	double 		distance 		= 0;
	end_waypoints.clear();
	double 		nCounter 		= 0;

	while((int)nextLeafToTrace.size()>0)
	{
		nCounter++;

		WayPoint* pH 	= nextLeafToTrace.at(0).second;

		assert(pH != 0);

		nextLeafToTrace.erase(nextLeafToTrace.begin()+0);

		for(unsigned int i =0; i< pH->pFronts.size(); i++)
		{
			if(pH->pFronts.at(i) && !CheckNodeExits(all_cells_to_delete, pH->pFronts.at(i)))
			{
				if(pH->cost < DistanceLimit)
				{
					wp = new WayPoint();
					*wp = *pH->pFronts.at(i);

					double d = distance2points(wp->pos, pH->pos);
					distance += d;
					wp->cost = pH->cost + d;
					wp->pBacks.push_back(pH);
					wp->pLeft = 0;
					wp->pRight = 0;

					nextLeafToTrace.push_back(make_pair(pH, wp));
					all_cells_to_delete.push_back(wp);
				}
				else
				{
					end_waypoints.push_back(pH);
				}
			}
		}
	}

	while((int)nextLeafToTrace.size()!=0)
		nextLeafToTrace.pop_back();
	//closed_nodes.clear();

	return (int)end_waypoints.size();
}

bool PlanningHelpers::CheckLaneIdExits(const std::vector<int>& lanes, const Lane* pL)
{
	if((int)lanes.size()==0) return true;

	for(unsigned int i=0; i< lanes.size(); i++)
	{
		if(lanes.at(i) == pL->id)
			return true;
	}

	return false;
}

WayPoint* PlanningHelpers::CheckLaneExits(const vector<WayPoint*>& nodes, const Lane* pL)
{
	if((int)nodes.size()==0) return nullptr;

	for(unsigned int i=0; i< nodes.size(); i++)
	{
		if(nodes.at(i)->pLane == pL)
			return nodes.at(i);
	}

	return nullptr;
}

WayPoint* PlanningHelpers::CheckNodeExits(const vector<WayPoint*>& nodes, const WayPoint* pL)
{
	if((int)nodes.size()==0) return nullptr;

	for(unsigned int i=0; i< nodes.size(); i++)
	{
		if(nodes.at(i)->laneId == pL->laneId && nodes.at(i)->id == pL->id)
			return nodes.at(i);
	}

	return nullptr;
}

WayPoint* PlanningHelpers::CreateLaneHeadCell(Lane* pLane, WayPoint* pLeft, WayPoint* pRight,
		WayPoint* pBack)
{
	if(!pLane) return nullptr;
	if((int)pLane->points.size()==0) return nullptr;

	WayPoint* c = new WayPoint;
	c->pLane 		= pLane;
	c->pos 			= pLane->points.at(0).pos;
	c->v			= pLane->speed;
	c->laneId  		= pLane->id;
	c->pLeft 		= pLeft;
	if(pLeft)
		c->cost		= pLeft->cost;

	c->pRight		= pRight;
	if(pRight)
		c->cost = pRight->cost;

	if(pBack)
	{
		pBack->pFronts.push_back(c);
		c->pBacks.push_back(pBack);
		c->cost = pBack->cost + distance2points(c->pos, pBack->pos);

		for(unsigned int i=0; i< c->pBacks.size(); i++)
		{
				if(c->pBacks.at(i)->cost < c->cost)
					c->cost = c->pBacks.at(i)->cost;
		}
	}
	return c;
}

double PlanningHelpers::GetLanePoints(Lane* l,
		const double& minDistance , const double& prevCost, vector<WayPoint>& points)
{
	if(l == NULL || minDistance<=0) return 0;

	int index = 0;
	WayPoint  p1, p2;
	WayPoint idx;

	p2 = p1 = l->points.at(index);
	p1.pLane = l;
	p1.distanceCost = prevCost;
	p2.distanceCost = p1.distanceCost + distance2points(p1.pos, p2.pos);

	points.push_back(p1);

	for(unsigned int i=index+1; i<l->points.size(); i++)
	{

		p2 = l->points.at(i);
		p2.pLane = l;
		p2.distanceCost = p1.distanceCost + distance2points(p1.pos, p2.pos);
		points.push_back(p2);

		if(p2.distanceCost >= minDistance)
				break;
		p1 = p2;
	}
	return p2.distanceCost;
}

WayPoint* PlanningHelpers::GetMinCostCell(const vector<WayPoint*>& cells, const vector<int>& globalPathIds)
{
	if((int)cells.size() == 1)
	{
//		for(unsigned int j = 0; j < cells.at(0)->actionCost.size(); j++)
//			cout << "Cost (" << cells.at(0)->laneId << ") of going : " << cells.at(0)->actionCost.at(j).first << ", is : " << cells.at(0)->actionCost.at(j).second << endl;
		return cells.at(0);
	}

	WayPoint* pC = cells.at(0); //cost is distance
	for(unsigned int i=1; i < cells.size(); i++)
	{
		bool bFound = false;
		if(globalPathIds.size()==0)
			bFound = true;

		int iLaneID = cells.at(i)->id;
		for(unsigned int j=0; j < globalPathIds.size(); j++)
		{
			if(globalPathIds.at(j) == iLaneID)
			{
				bFound = true;
				break;
			}
		}

//		for(unsigned int j = 0; j < cells.at(0)->actionCost.size(); j++)
//			cout << "Cost ("<< i <<") of going : " << cells.at(0)->actionCost.at(j).first << ", is : " << cells.at(0)->actionCost.at(j).second << endl;


		if(cells.at(i)->cost < pC->cost && bFound == true)
			pC = cells.at(i);
	}


	return pC;
}

void PlanningHelpers::ExtractPlanAlernativesSection(const WayPoint& startPose, const double& plan_distance, std::vector<WayPoint>& path)
{
	RelativeInfo start_info;
	PlanningHelpers::GetRelativeInfo(startPose.pLane->points, startPose, start_info);
	vector<WayPoint*> local_cell_to_delete;
	WayPoint* pStart = &startPose.pLane->points.at(start_info.iFront);
	WayPoint* pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeStraight(pStart, plan_distance, local_cell_to_delete);
	if(pLaneCell)
	{
		vector<vector<WayPoint> > tempCurrentForwardPathss;
		vector<int> globalPathIds;
		PlanningHelpers::TraversePathTreeBackwards(pLaneCell, pStart, globalPathIds, path, tempCurrentForwardPathss);
	}
}

void PlanningHelpers::RemoveFromPathUntil(std::vector<WayPoint>& path, const double& distance)
{
	if((int)path.size() < 2) return;

	CalcAngleAndCost(path);

	int cut_off_index = (int)path.size() - 2;
	for(unsigned int i = 0; i < path.size() - 2; i++)
	{
		if(path.at(i).distanceCost > distance)
		{
			cut_off_index = i;
			break;
		}
	}

	if(cut_off_index > 0)
	{
		path.erase(path.begin(), path.begin() + cut_off_index);
	}
}

void PlanningHelpers::ExtractPlanAlernativesV2(const std::vector<WayPoint>& singlePath, const double& plan_distance, const double& lane_change_distane, std::vector<std::vector<WayPoint> >& allPaths)
{
	if((int)singlePath.size() == 0) return;

	std::vector<WayPoint> path_sections;
	std::vector<WayPoint> straight_path;
	allPaths.clear();

	for(unsigned int i = 0; i < singlePath.size(); i++)
	{
		if(singlePath.at(i).bDir != FORWARD_DIR && singlePath.at(i).pLane && (int)singlePath.at(i).pFronts.size() > 0 && i > 0)
		{
			straight_path.clear();
			ExtractPlanAlernativesSection(singlePath.at(i-1), plan_distance, straight_path);
			straight_path.insert(straight_path.begin(), path_sections.begin(), path_sections.end());
			RemoveFromPathUntil(straight_path, lane_change_distane);
			allPaths.push_back(straight_path);
			path_sections.clear();
		}
		else
		{
			path_sections.push_back(singlePath.at(i));
		}
	}

	RemoveFromPathUntil(path_sections, lane_change_distane);
	allPaths.push_back(path_sections);

//	allPaths.clear();
//	for(unsigned int i= 1; i < singlePath.size(); i++)
//	{
//		if(singlePath.at(i).bDir != FORWARD_DIR && singlePath.at(i).pLane && singlePath.at(i).pFronts.size() > 0)
//		{
//			WayPoint start_point = singlePath.at(i-1);
//
//
//				if(straight_path.size() > 2)
//				{
//					//std::cout << "Generated Parallel Path : " << straight_path.size() << std::endl;
//					double avg_cost_distance = 0;
// 					for(auto& p: straight_path)
//					{
//						RelativeInfo change_inf;
//						int dummy_index = 0;
//						GetRelativeInfo(singlePath, p, change_inf, dummy_index);
//						avg_cost_distance += change_inf.perp_distance;
//						//std::cout << "Distance to Side Path: " << change_inf.perp_distance << std::endl;
//					}
//
// 					avg_cost_distance = avg_cost_distance/straight_path.size();
// 					//std::cout << "Generated Parallel Path Cost: " << fabs(avg_cost_distance) << std::endl;
//
//					straight_path.insert(straight_path.begin(), singlePath.begin(), singlePath.begin()+(i-1));
//					for(unsigned int ic = 0; ic < straight_path.size(); ic++)
//					{
//						straight_path.at(ic).laneChangeCost = fabs(avg_cost_distance);
//					}
//					allPaths.push_back(straight_path);
//				}
//			}
//		}
//	}
//
//	allPaths.push_back(singlePath);
}

void PlanningHelpers::ExtractPlanAlernatives(const std::vector<WayPoint>& singlePath, const double& plan_distance, std::vector<std::vector<WayPoint> >& allPaths, double lane_change_distance)
{
	if((int)singlePath.size() == 0) return;

	allPaths.clear();
	std::vector<WayPoint> path;
	path.push_back(singlePath.at(0));
	double d = 0;
	bool bStartSkip = false;
	for(unsigned int i= 1; i < singlePath.size(); i++)
	{
		if(singlePath.at(i).bDir != FORWARD_DIR && singlePath.at(i).pLane && singlePath.at(i).pFronts.size() > 0)
		{
			bStartSkip = true;
			WayPoint start_point = singlePath.at(i-1);
			RelativeInfo start_info;
			PlanningHelpers::GetRelativeInfo(start_point.pLane->points, start_point, start_info);
			vector<WayPoint*> local_cell_to_delete;
			WayPoint* pStart = &start_point.pLane->points.at(start_info.iFront);
			WayPoint* pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeStraight(pStart, plan_distance, local_cell_to_delete);
			if(pLaneCell)
			{
				vector<WayPoint> straight_path;
				vector<vector<WayPoint> > tempCurrentForwardPathss;
				vector<int> globalPathIds;
				PlanningHelpers::TraversePathTreeBackwards(pLaneCell, pStart, globalPathIds, straight_path, tempCurrentForwardPathss);
				if((int)straight_path.size() > 2)
				{
					//std::cout << "Generated Parallel Path : " << straight_path.size() << std::endl;
					double avg_cost_distance = 0;
 					for(auto& p: straight_path)
					{
						RelativeInfo change_inf;
						int dummy_index = 0;
						GetRelativeInfo(singlePath, p, change_inf, dummy_index);
						avg_cost_distance += change_inf.perp_distance;
						//std::cout << "Distance to Side Path: " << change_inf.perp_distance << std::endl;
					}

 					avg_cost_distance = avg_cost_distance/(int)straight_path.size();
 					//std::cout << "Generated Parallel Path Cost: " << fabs(avg_cost_distance) << std::endl;

					straight_path.insert(straight_path.begin(), path.begin(), path.end());
					for(unsigned int ic = 0; ic < straight_path.size(); ic++)
					{
						straight_path.at(ic).laneChangeCost = fabs(avg_cost_distance);
					}
					allPaths.push_back(straight_path);
				}
			}
		}
		else
		{
			if(bStartSkip)
			{
				d += hypot(singlePath.at(i).pos.y - singlePath.at(i-1).pos.y, singlePath.at(i).pos.x - singlePath.at(i-1).pos.x);
				if(d > lane_change_distance)
				{
					d = 0;
					bStartSkip = false;
				}
			}

			if(!bStartSkip)
			{
				path.push_back(singlePath.at(i));
			}
		}
	}

	allPaths.push_back(path);
}

void PlanningHelpers::TraversePathTreeBackwards(WayPoint* pHead, WayPoint* pStartWP,const vector<int>& globalPathIds,
		vector<WayPoint>& localPath, std::vector<std::vector<WayPoint> >& localPaths)
{
	if(pHead != NULL && pHead->id != pStartWP->id)
	{
		if((int)pHead->pBacks.size()>0)
		{
			localPaths.push_back(localPath);
			TraversePathTreeBackwards(GetMinCostCell(pHead->pBacks, globalPathIds),pStartWP, globalPathIds, localPath, localPaths);
			pHead->bDir = FORWARD_DIR;
			localPath.push_back(*pHead);
		}
		else if(pHead->pLeft && pHead->cost > 0)
		{
			//vector<Vector2D> forward_path;
			//TravesePathTreeForwards(pHead->pLeft, forward_path, FORWARD_RIGHT);
			//localPaths.push_back(forward_path);
			cout << "Global Lane Change  Right " << endl;
			TraversePathTreeBackwards(pHead->pLeft,pStartWP, globalPathIds, localPath, localPaths);
			pHead->bDir = FORWARD_RIGHT_DIR;
			localPath.push_back(*pHead);
		}
		else if(pHead->pRight && pHead->cost > 0)
		{
			//vector<Vector2D> forward_path;
			//TravesePathTreeForwards(pHead->pRight, forward_path, FORWARD_LEFT);
			//localPaths.push_back(forward_path);

			cout << "Global Lane Change  Left " << endl;
			TraversePathTreeBackwards(pHead->pRight,pStartWP, globalPathIds, localPath, localPaths);
			pHead->bDir = FORWARD_LEFT_DIR;
			localPath.push_back(*pHead);
		}
//		else
//			cout << "Err: PlannerZ -> NULL Back Pointer " << pHead;
	}
	else
		assert(pHead);
}

// ACTION_TYPE PlanningHelpers::GetBranchingDirection(WayPoint& currWP, WayPoint& nextWP)
// {
// 	ACTION_TYPE t = FORWARD_ACTION;

// //	//first Get the average of the next 3 waypoint directions
// //	double angle = 0;
// //	if(nextWP.pLane->id == 487)
// //		angle = 11;
// //
// //	int counter = 0;
// //	angle = 0;
// //
// //	for(unsigned int i=0; i < nextWP.pLane->points.size() && counter < 10; i++, counter++)
// //	{
// //		angle += nextWP.pLane->points.at(i).pos.a;
// //	}
// //	angle = angle / counter;
// //
// //	//Get Circular angle for correct subtraction
// //	double circle_angle = Angle::GetCircularAngle(currWP.pos.a, angle);
// //
// //	if( currWP.pos.a - circle_angle > (7.5*DEG2RAD))
// //	{
// //		t = RIGHT_TURN_ACTION;
// //		cout << "Right Lane, Average Angle = " << angle*RAD2DEG << ", Circle Angle = " << circle_angle*RAD2DEG << ", currAngle = " << currWP.pos.a*RAD2DEG << endl;
// //	}
// //	else if( currWP.pos.a - circle_angle < (-7.5*DEG2RAD))
// //	{
// //		t = LEFT_TURN_ACTION;
// //		cout << "Left Lane, Average Angle = " << angle*RAD2DEG << ", Circle Angle = " << circle_angle*RAD2DEG << ", currAngle = " << currWP.pos.a*RAD2DEG << endl;
// //	}

// 	return t;
// }

void PlanningHelpers::CalcContourPointsForDetectedObjects(const WayPoint& currPose, vector<DetectedObject>& obj_list, const double& filterDistance)
{
	vector<DetectedObject> res_list;
	for(unsigned int i = 0; i < obj_list.size(); i++)
	{
		GPSPoint center = obj_list.at(i).center.pos;
		double distance = distance2points(center, currPose.pos);

		if(distance < filterDistance)
		{
			DetectedObject obj = obj_list.at(i);

			Mat3 rotationMat(center.a);
			Mat3 translationMat(center.x, center.y);
			double w2 = obj.w/2.0;
			double h2 = obj.l/2.0;
			double z = center.z + obj.h/2.0;

			GPSPoint left_bottom(-w2, -h2, z,0);
			GPSPoint right_bottom(w2,-h2, z,0);
			GPSPoint right_top(w2,h2, z,0);
			GPSPoint left_top(-w2,h2, z,0);

			left_bottom 	= rotationMat * left_bottom;
			right_bottom 	= rotationMat * right_bottom;
			right_top 		= rotationMat * right_top;
			left_top 		= rotationMat * left_top;

			left_bottom 	= translationMat * left_bottom;
			right_bottom 	= translationMat * right_bottom;
			right_top 		= translationMat * right_top;
			left_top 		= translationMat * left_top;

			obj.contour.clear();
			obj.contour.push_back(left_bottom);
			obj.contour.push_back(right_bottom);
			obj.contour.push_back(right_top);
			obj.contour.push_back(left_top);

			res_list.push_back(obj);
		}
	}

	obj_list = res_list;
}

double PlanningHelpers::GetCurvatureCostAhead(const std::vector<WayPoint>& path, const RelativeInfo& info,int& prev_index, const double& search_distance)
{
	if((int)path.size()==0) return 0;

	double min_cost = path.at(info.iBack).curvatureCost;
	double d = info.to_front_distance;

	int local_i = info.iFront;
	while(local_i < (int)path.size()-1 && d < search_distance)
	{
		local_i++;
		d += hypot(path.at(local_i).pos.y - path.at(local_i-1).pos.y, path.at(local_i).pos.x - path.at(local_i-1).pos.x);
		if(path.at(local_i).curvatureCost < min_cost)
		{
			min_cost = path.at(local_i).curvatureCost;
		}
	}

	if(local_i < prev_index && prev_index < (int)path.size())
	{
		min_cost = path.at(prev_index).curvatureCost;
	}
	else
	{
		prev_index = local_i;
	}

	return min_cost;
}

double PlanningHelpers::GetVelocityAhead(const std::vector<WayPoint>& path, const RelativeInfo& info, int& prev_index, const double& reasonable_brake_distance)
{
	if((int)path.size()==0) return 0;


	double min_v = path.at(info.iBack).v;
	double d = info.to_front_distance;

	int local_i = info.iFront;
	while(local_i < (int)path.size()-1 && d < reasonable_brake_distance)
	{
		local_i++;
		d += hypot(path.at(local_i).pos.y - path.at(local_i-1).pos.y, path.at(local_i).pos.x - path.at(local_i-1).pos.x);
		if(path.at(local_i).v < min_v)
			min_v = path.at(local_i).v;
	}

	if(local_i < prev_index && prev_index < (int)path.size())
	{
		min_v = path.at(prev_index).v;
	}
	else
		prev_index = local_i;

	return min_v;
}

void PlanningHelpers::WritePathToFile(const string& fileName, const vector<WayPoint>& path)
{
	DataRW  dataFile;
	ostringstream str_header;
	str_header << "laneID" << "," << "wpID"  << "," "x" << "," << "y" << "," << "a"<<","<< "cost" << "," << "Speed" << "," ;
	vector<string> dataList;
	 for(unsigned int i=0; i<path.size(); i++)
	 {
		 ostringstream strwp;
		 strwp << path.at(i).laneId << "," << path.at(i).id <<","<<path.at(i).pos.x<<","<< path.at(i).pos.y
				 <<","<< path.at(i).pos.a << "," << path.at(i).curvatureCost << "," << path.at(i).v << ",";
		 dataList.push_back(strwp.str());
	 }

	 dataFile.WriteLogData("", fileName, str_header.str(), dataList);
}

LIGHT_INDICATOR PlanningHelpers::GetIndicatorsFromPath(const std::vector<WayPoint>& path, const WayPoint& pose,  const double& seachDistance)
{
	if((int)path.size() < 2)
		return INDICATOR_NONE;

	LIGHT_INDICATOR ind = INDICATOR_NONE;
	RelativeInfo info;
	PlanningHelpers::GetRelativeInfo(path, pose, info);

	if((int)info.perp_point.actionCost.size() > 0)
	{
		if(info.perp_point.actionCost.at(0).first == LEFT_TURN_ACTION)
			ind = INDICATOR_LEFT;
		else if(info.perp_point.actionCost.at(0).first == RIGHT_TURN_ACTION)
			ind = INDICATOR_RIGHT;
	}

	double total_d = 0;
	for(unsigned int i=info.iFront; i < path.size()-2; i++)
	{

		total_d+= hypot(path.at(i+1).pos.y - path.at(i).pos.y, path.at(i+1).pos.x - path.at(i).pos.x);
		if((int)path.at(i).actionCost.size() > 0)
		{
			if(path.at(i).actionCost.at(0).first == LEFT_TURN_ACTION)
				return INDICATOR_LEFT;
			else if(path.at(i).actionCost.at(0).first == RIGHT_TURN_ACTION)
				return INDICATOR_RIGHT;
		}

		if(total_d > seachDistance)
			break;
	}

	return ind;
}

std::string PlanningHelpers::MakePathID(const std::vector<WayPoint>& _path)
{
	std::ostringstream str_id;
	if((int)_path.size() == 0) return str_id.str();

	int prev_id = -10;
	int curr_id = -10;

	for(unsigned int i = 0; i < _path.size(); i++)
	{
		curr_id = _path.at(i).laneId;
		if(curr_id != prev_id)
		{
			str_id << curr_id;
			str_id << "_";
			prev_id = curr_id;
		}
	}

	return str_id.str();
}

std::string PlanningHelpers::MakePathDirectionID(const std::vector<WayPoint>& _path)
{
	if((int)_path.size() == 0) return "";

	for(unsigned int i = 0; i < _path.size(); i++)
	{
		if((int)_path.at(i).actionCost.size() > 0)
		{
			if(_path.at(i).actionCost.at(0).first == LEFT_TURN_ACTION)
			{
				return "L";
			}
			else if(_path.at(i).actionCost.at(0).first == RIGHT_TURN_ACTION)
			{
				return "R";
			}
		}
	}

	return "F";
}

WayPoint PlanningHelpers::GetRealCenter(const WayPoint& currState, const double& wheel_base)
{
	WayPoint pose_center = currState;
	Mat3 rotationMat(-currState.pos.a);
	Mat3 translationMat(-currState.pos.x, -currState.pos.y);

	Mat3 rotationMatInv(currState.pos.a);
	Mat3 translationMatInv(currState.pos.x, currState.pos.y);

	pose_center.pos = translationMat*pose_center.pos;
	pose_center.pos = rotationMat*pose_center.pos;

	pose_center.pos.x += wheel_base/3.0;

	pose_center.pos = rotationMatInv*pose_center.pos;
	pose_center.pos = translationMatInv*pose_center.pos;

	return pose_center;
}

void PlanningHelpers::GetCubeAndCenterofTwoPoints(const WayPoint& p1, const WayPoint& p2, double width, double depth,
		WayPoint& center_p, WayPoint& min_p, WayPoint& max_p)
{
 	double y_diff = hypot(p2.pos.y - p1.pos.y, p2.pos.x - p1.pos.x);
	double z_diff = p1.pos.z - p2.pos.z;
	double d = hypot(y_diff, z_diff);
	double a = atan2(p2.pos.y- p1.pos.y, p2.pos.x- p1.pos.x); // rotation around z
	double b = atan2(z_diff, y_diff);// rotation around y

	center_p.pos.x = (p1.pos.x + p2.pos.x)/2.0;
	center_p.pos.y = (p1.pos.y + p2.pos.y)/2.0;
	center_p.pos.z = (p1.pos.z + p2.pos.z)/2.0;
	center_p.pos.a = a;
	center_p.rot.z = a;
	center_p.rot.y = b;
	center_p.rot.x = 0;

	//cout << " Z_diff : " << z_diff << ", y_diff " << y_diff << endl << endl;
	min_p = center_p;
	max_p = center_p;

	min_p.pos.x -= d/2.0;
	min_p.pos.y -= width;
	min_p.pos.z -= depth;

	max_p.pos.x += d/2.0;
	max_p.pos.y += width;
	max_p.pos.z += depth;
}

double PlanningHelpers::GetDistanceFromPoseToEnd(const WayPoint& pose, const std::vector<WayPoint>& path)
{
	RelativeInfo info;
	PlanningHelpers::GetRelativeInfoDirectionLimited(path, pose, info);

	if(info.bAfter)
		return -info.from_back_distance;

	 double d = 0;
	 for(unsigned int i = info.iFront; i < path.size()-1; i++)
	 {
		 d += hypot(path.at(i+1).pos.y - path.at(i).pos.y, path.at(i+1).pos.x - path.at(i).pos.x);
	 }

	 if(info.bBefore)
		 d += info.to_front_distance;

	 return d;
}

double PlanningHelpers::CalculateLookAheadDistance(const double& steering_delay, const double& curr_velocity, const double& min_distance, double speed_factor, double delay_factor)
{
	// the bigger the steering delay the farther we want to look ahead, it will contribute by the following percentage
	// assume that maximum steering delay is 3 seconds
	double steering_delay_percentage = delay_factor * MAX_STEERING_ALLOWED_DELAY;
	if(steering_delay < 3.0)
	{
		steering_delay_percentage = delay_factor * (steering_delay / MAX_STEERING_ALLOWED_DELAY);
	}
	
	//only 25% of the current velocity is added to the minimum pursuit distance, the faster we driver the farther we want to look ahead
	double total_percentage = steering_delay_percentage + speed_factor;
	double d = min_distance + (total_percentage * fabs(curr_velocity));

	if(d < min_distance)
	{
		d = min_distance;
	}

	return d;
}

void PlanningHelpers::EstimateFuturePosition(const WayPoint& currPose, const double& currSteering, const double& est_distance,
		const double& est_resolution, const double& wheel_base, WayPoint& estimatedPose)
{
	int delta_distance = (est_distance/est_resolution) + 1;
	estimatedPose = currPose;
	for(int i = 0; i < delta_distance; i++)
	{
		PlanningHelpers::PredictMotionDistanceBased(estimatedPose.pos.x, estimatedPose.pos.y, estimatedPose.pos.a, currSteering, est_resolution, wheel_base);
	}

	//Keep same position, only estimate future heading angle (theta)
	//estimatedPose.pos.x = currPose.pos.x;
	//estimatedPose.pos.y = currPose.pos.y;
}

void PlanningHelpers::PredictMotionDistanceBased(double& x, double &y, double& heading, double steering, double distance, double wheelbase)
{
	x += distance *  cos(heading);
	y += distance *  sin(heading);
	heading = heading + ((distance * tan(steering))/(wheelbase));
}

void PlanningHelpers::PredictMotionTimeBased(double& x, double &y, double& heading, double steering, double velocity, double wheelbase, double time_elapsed)
{
	x += velocity * time_elapsed *  cos(heading);
	y += velocity * time_elapsed *  sin(heading);
	heading = heading + ((velocity*time_elapsed*tan(steering))  / (wheelbase) );
}

void PlanningHelpers::InitializeSafetyPolygon(const WayPoint& curr_state, const CAR_BASIC_INFO& car_info,
                                                  const VehicleState& vehicle_state, const double& lateral_safe_d,
                                                  const double& long_safe_d, const bool& use_turning_angle, PolygonShape& car_border)
{

	double c_lateral_d = (car_info.width/2.0) + lateral_safe_d;
	double c_long_front_d = car_info.wheel_base + car_info.front_length + long_safe_d;
	double c_car_front_d = car_info.wheel_base + car_info.front_length;
	double c_long_back_d = car_info.back_length + long_safe_d;

  Mat3 inv_rotation_mat(curr_state.pos.a - M_PI_2);
  Mat3 inv_translation_mat(curr_state.pos.x, curr_state.pos.y);

  double corner_slide_distance = c_lateral_d / 2.0;
  double ratio_to_angle = corner_slide_distance / car_info.max_wheel_angle;
  double slide_distance = vehicle_state.steer * ratio_to_angle;

  GPSPoint bottom_left(-c_lateral_d, -c_long_back_d, curr_state.pos.z, 0);
  GPSPoint bottom_right(c_lateral_d, -c_long_back_d, curr_state.pos.z, 0);

  GPSPoint top_right_car(c_lateral_d, c_car_front_d, curr_state.pos.z, 0);
  GPSPoint top_left_car(-c_lateral_d, c_car_front_d, curr_state.pos.z, 0);

  GPSPoint top_right(c_lateral_d - slide_distance, c_long_front_d, curr_state.pos.z, 0);
  GPSPoint top_left(-c_lateral_d - slide_distance, c_long_front_d, curr_state.pos.z, 0);

  bottom_left = inv_rotation_mat * bottom_left;
  bottom_left = inv_translation_mat * bottom_left;

  top_right = inv_rotation_mat * top_right;
  top_right = inv_translation_mat * top_right;

  bottom_right = inv_rotation_mat * bottom_right;
  bottom_right = inv_translation_mat * bottom_right;

  top_left = inv_rotation_mat * top_left;
  top_left = inv_translation_mat * top_left;

  top_right_car = inv_rotation_mat * top_right_car;
  top_right_car = inv_translation_mat * top_right_car;

  top_left_car = inv_rotation_mat * top_left_car;
  top_left_car = inv_translation_mat * top_left_car;

  car_border.points.clear();
  car_border.points.push_back(bottom_left);
  car_border.points.push_back(bottom_right);
  car_border.points.push_back(top_right_car);
  if(use_turning_angle == true)
  {
	  car_border.points.push_back(top_right);
	  car_border.points.push_back(top_left);
  }
  car_border.points.push_back(top_left_car);
}

int PlanningHelpers::PointInsidePolygon(const std::vector<GPSPoint>& points,const GPSPoint& p)
{
        int counter = 0;
          int i;
          double xinters;
          GPSPoint p1,p2;
          int N = (int)points.size();
          if(N <=0 ) return -1;

          p1 = points.at(0);
          for (i=1;i<=N;i++)
          {
            p2 = points.at(i % N);

            if (p.y > MIN(p1.y,p2.y))
            {
              if (p.y <= MAX(p1.y,p2.y))
              {
                if (p.x <= MAX(p1.x,p2.x))
                {
                  if (p1.y != p2.y)
                  {
                    xinters = (p.y-p1.y)*(p2.x-p1.x)/(p2.y-p1.y)+p1.x;
                    if (p1.x == p2.x || p.x <= xinters)
                      counter++;
                  }
                }
              }
            }
            p1 = p2;
          }

          if (counter % 2 == 0)
            return 0;
          else
            return 1;
}

int PlanningHelpers::PointInsidePolygon(const std::vector<WayPoint>& points,const WayPoint& p)
{
	int counter = 0;
	int i;
	double xinters;
	WayPoint p1,p2;
	int N = (int)points.size();
	if(N <= 0 ) return -1;

	p1 = points.at(0);
	for (i=1; i <= N; i++)
	{
		p2 = points.at(i % N);

		if (p.pos.y > MIN(p1.pos.y,p2.pos.y))
		{
		  if (p.pos.y <= MAX(p1.pos.y,p2.pos.y))
		  {
			if (p.pos.x <= MAX(p1.pos.x,p2.pos.x))
			{
			  if (p1.pos.y != p2.pos.y)
			  {
				xinters = (p.pos.y-p1.pos.y)*(p2.pos.x-p1.pos.x)/(p2.pos.y-p1.pos.y)+p1.pos.x;
				if (p1.pos.x == p2.pos.x || p.pos.x <= xinters)
				  counter++;
			  }
			}
		  }
		}

		p1 = p2;
	}

	if (counter % 2 == 0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

bool PlanningHelpers::CheckFrontLane(WayPoint* pWP1, WayPoint* pWP2, int search_level)
{
	std::vector<Lane*> pLanes;
	pLanes.push_back(pWP1->pLane);
	int level_count = 0;
	while(level_count < search_level && (int)pLanes.size() > 0)
	{
		std::vector<Lane*> pNextLevel;
		for(auto& pL: pLanes)
		{
			if(pWP2->pLane == pL)
			{
				return true;
			}

			pNextLevel.insert(pNextLevel.end(), pL->toLanes.begin(), pL->toLanes.end());
		}

		pLanes = pNextLevel;

		level_count++;
	}

	return false;
}

bool PlanningHelpers::CheckBackLane(WayPoint* pWP1, WayPoint* pWP2, int search_level)
{
	std::vector<Lane*> pLanes;
	pLanes.push_back(pWP1->pLane);
	int level_count = 0;
	while(level_count < search_level && (int)pLanes.size() > 0)
	{
		for(auto& pL: pLanes)
		{
			if(pWP2->pLane == pL)
			{
				return true;
			}

			pLanes.insert(pLanes.end(), pL->fromLanes.begin(), pL->fromLanes.end());
		}
		level_count++;
	}

	return false;
}

bool PlanningHelpers::CheckRightLane(WayPoint* pWP1, WayPoint* pWP2)
{
	WayPoint* p2 = nullptr;
	Lane* pRL = nullptr;

	if(pWP2->pRight != nullptr)
	{
		p2 = pWP2->pRight;
		pRL = p2->pLane;
	}

	while(pRL != nullptr)
	{
		if(pWP1->pLane == pRL)
		{
			return true;
		}

		if(p2->pRight != nullptr)
		{
			p2 = p2->pRight;
			pRL = p2->pLane;
		}
		else
		{
			return false;
		}
	}

	return false;
}

bool PlanningHelpers::CheckLeftLane(WayPoint* pWP1, WayPoint* pWP2)
{
	WayPoint* p2 = nullptr;
	Lane* pLL = nullptr;

	if(pWP2->pLeft != nullptr)
	{
		p2 = pWP2->pLeft;
		pLL = p2->pLane;
	}

	while(pLL != nullptr)
	{
		if(pWP1->pLane == pLL)
		{
			return true;
		}

		if(p2->pLeft != nullptr)
		{
			p2 = p2->pLeft;
			pLL = p2->pLane;
		}
		else
		{
			return false;
		}
	}

	return false;
}

void PlanningHelpers::FilterWaypoints(std::vector<WayPoint*>& wp_list, WayPoint* pPrevWP)
{
	//Only insert waypoint from differnt lanes
	std::vector<WayPoint*> p_wp_filtered;
	while((int)wp_list.size() > 0)
	{
		bool bFound = false;
		for(auto& p_p: p_wp_filtered)
		{
			if(p_p->pLane == wp_list.at(0)->pLane)
			{
				bFound = true;
				break;
			}
		}

		if(bFound == false)
		{
			p_wp_filtered.push_back(wp_list.at(0));
		}

		wp_list.erase(wp_list.begin()+0);
	}

	if(pPrevWP == nullptr)
	{
		wp_list = p_wp_filtered;
		return;
	}

	wp_list.clear();
	//Check each two points
	WayPoint* pWP1 = pPrevWP;

	while((int)p_wp_filtered.size() > 0)
	{
		WayPoint*  pWP2 = p_wp_filtered.at(0);
		p_wp_filtered.erase(p_wp_filtered.begin()+0);

		if(CheckLeftLane(pWP2, pWP1) || CheckRightLane(pWP2, pWP1) || CheckFrontLane(pWP1, pWP2, 4))
		{
			wp_list.push_back(pWP2);
			return;
		}
		else if((int)p_wp_filtered.size() == 0) // if only one waypoint left ! just use it
		{
			wp_list.push_back(pWP2);
			return;
		}
	}

}


double PlanningHelpers::frunge ( double x )
{
  double fx;

  fx = 1.0 / ( 1.0 + 25.0 * x * x );

  return fx;
}

double PlanningHelpers::fprunge ( double x )
{
  double bot;
  double fx;

  bot = 1.0 + 25.0 * x * x;
  fx = -50.0 * x / ( bot * bot );

  return fx;
}

double PlanningHelpers::fpprunge ( double x )
{
  double bot;
  double fx;

  bot = 1.0 + 25.0 * x * x;
  fx = ( -50.0 + 3750.0 * x * x ) / ( bot * bot * bot );

  return fx;
}

WayPoint PlanningHelpers::CalcCenterPoint(const std::vector<WayPoint>& points)
{
	WayPoint min(DBL_MAX, DBL_MAX, DBL_MAX, 0);
	WayPoint max(DBL_MIN, DBL_MIN, DBL_MIN, 0);

	for(auto& p : points)
	{
		if(p.pos.x > max.pos.x) max.pos.x = p.pos.x;
		if(p.pos.y > max.pos.y) max.pos.y = p.pos.y;
		if(p.pos.z > max.pos.z) max.pos.z = p.pos.z;

		if(p.pos.x < min.pos.x) min.pos.x = p.pos.x;
		if(p.pos.y < min.pos.y) min.pos.y = p.pos.y;
		if(p.pos.z < min.pos.z) min.pos.z = p.pos.z;
	}

	return WayPoint((max.pos.x+min.pos.x)/2.0, (max.pos.y+min.pos.y)/2.0, (max.pos.z+min.pos.z)/2.0, 0);
}

} /* namespace planning */
} /* namespace op */
