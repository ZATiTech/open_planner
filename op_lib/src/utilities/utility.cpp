/// \file UtilityH.cpp
/// \brief General Math and Control utility functions
/// \author Hatem Darweesh
/// \date May 14, 2016

#include "utilities/utility.h"
#include <iostream>
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>


namespace op
{
  namespace utilities
  {

using namespace std;


 //return 0 if equal, -1 if a < b , 1 if a > b
 int Angle::CompareDouble(const double& a, const double& b)
 {
	 if (fabs(a - b) < 1e-9)
	 {
		 return 0;
	 }
	 else if(a < b)
	 {
		 return -1;
	 }
	 else
	 {
		 return 1;
	 }
 }



 int Angle::GetSign(double x)
 {
	 if(x < 0 )
		 return -1;
	 else
		 return 1;
 }

 double Angle::FixNegativeAngle(const double& a)
{
  double angle = 0;
  if (a < -2.0 * M_PI || a >= 2.0 * M_PI) {
    angle = fmod(a, 2.0 * M_PI);
  } else {
    angle = a;
  }

  if(angle < 0) {
    angle = 2.0 * M_PI + angle;
  }

  return angle;
}

 double Angle::SplitPositiveAngle(const double& a)
{
  double angle = a;

  if (a < -2.0 * M_PI || a >= 2.0 * M_PI) {
    angle = fmod(a, 2.0 * M_PI);
  }

  if (angle >= M_PI) {
    angle -= 2.0 * M_PI;
  } else if (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  
  return angle;
}

double Angle::InverseAngle(const double& a)
{

   double angle = 0;
   if(a < M_PI) {
     angle =  a + M_PI;
   } else {
     angle = a - M_PI;
   }
   
   return angle;
}

double Angle::AngleBetweenTwoAnglesPositive(const double& a1, const double& a2)
{
   double diff = a1 - a2;
   if(diff < 0)
	   diff = a2 - a1;

   if(diff > M_PI)
	   diff = 2.0*M_PI - diff;

   return diff;
}

double Angle::GetCircularAngle(const double& prevContAngle, const double& prevAngle, const double& currAngle)
{

	double diff = currAngle - prevAngle;
	if(diff > M_PI)
		diff = diff - 2.0*M_PI;
	if(diff < -M_PI)
		diff = diff + 2.0*M_PI;

	double c_ang = 0;
	if(prevContAngle == 0 || fabs(diff) < M_PI_2)
		 c_ang = prevContAngle + diff;
	else
		c_ang = prevContAngle;

	return c_ang;
}

void Time::GetTickCount(struct timespec& t)
{
	while(clock_gettime(0, & t) == -1);
}

double Time::GetTimeDiff(const struct timespec& old_t,const struct timespec& curr_t)
{
	return (curr_t.tv_sec - old_t.tv_sec) + ((double)(curr_t.tv_nsec - old_t.tv_nsec)/ 1000000000.0);
}

double Time::GetTimeDiffNow(const struct timespec& old_t)
{
	struct timespec curr_t;
	GetTickCount(curr_t);
	return (curr_t.tv_sec - old_t.tv_sec) + ((double)(curr_t.tv_nsec - old_t.tv_nsec)/ 1000000000.0);
}

string Time::GetFilePrefixHourMinuteSeconds()
{
	struct timespec now_time;
	GetTickCount(now_time);
	tm *gmtm = localtime(&now_time.tv_sec);
	ostringstream str;

	str << "Y" << gmtm->tm_year;
	str << "-";
	str << "M" << gmtm->tm_mon;
	str << "-";
	str << "D" << gmtm->tm_mday;
	str << "-";
	str << "H" << gmtm->tm_hour;
	str << "-";
	str << "M" << gmtm->tm_min;
	str << "-";
	str << "S" << gmtm->tm_sec;

	return str.str();
}

string Time::GetDateTimeStr()
{
	time_t now = time(0);
	char* dateStr = ctime(&now);
	string str(dateStr, strlen(dateStr)-1);
	int index = str.find(" ");
	while(index > 0)
	{
		str.replace(index,1, "_");
		index = str.find(" ");
	}

	index = str.find(":");
	while(index > 0)
	{
		str.replace(index,1, "-");
		index = str.find(":");
	}
	return str;
}

int  Time::tsCompare (struct  timespec  time1,   struct  timespec  time2, int micro_tolerance)
{
  long nanoDiff =
    (time1.tv_sec * 1000000000 + time1.tv_nsec) - 
    (time2.tv_sec * 1000000000 + time2.tv_nsec);
  
  if (nanoDiff < -micro_tolerance) {
    return -1;  // time1 is less than time2
  } else if (nanoDiff > micro_tolerance) {
    return 1;  // time1 is greater than time2
  } else {
    return 0;  // time1 is equal to time2
  }
}

timespec Time::GetTimeSpec(const time_t& srcT)
{
	timespec dstT;
	dstT.tv_sec = srcT/1000000000;
	dstT.tv_nsec = srcT - (dstT.tv_sec*1000000000);
	return dstT;
}

time_t Time::GetLongTime(const struct timespec& srcT)
{
	time_t dstT;
	dstT = srcT.tv_sec * 1000000000 + srcT.tv_nsec;
	return dstT;
}

std::string System::GetFileNameFromPathName(const std::string& path_with_name)
{
	int index_last = path_with_name.find_last_of("/");
	std::string file_name_only;

	if(index_last > 0 && (index_last+1) < (int)path_with_name.size())
	{
		return path_with_name.substr(index_last, path_with_name.size());
	}

	return path_with_name;
}

std::string System::GetFileNameWithoutExtention(const std::string& file_name)
{
	int index_last = file_name.find_last_of(".");
	if(index_last > 0)
	{
		return file_name.substr(0, index_last);
	}

	return file_name;
}


void System::GetFileNameInAllSubfolders(const std::string& in_path, const std::vector<std::string>& extentions, std::vector<std::string>& out_list, int dir_level)
{
	std::string path = in_path;
	if(path.at(path.size()-1) != '/')
	{
		path.push_back('/');
	}

	DIR *dir = opendir (path.c_str());

	if(dir == NULL || dir_level > 3) return;

	struct dirent *ent = NULL;

	while ((ent = readdir (dir)) != NULL)
	{
		std::string d_name(ent->d_name);
		if(ent->d_type == DT_DIR && d_name.compare(".") != 0 && d_name.compare("..") != 0)
		{
			GetFileNameInFolder(path + d_name, extentions, out_list);

			GetFileNameInAllSubfolders(path + d_name, extentions, out_list, dir_level+1);
		}
	}

	closedir (dir);
}

void System::GetImageNamesInAllSubfoldersCam1Cam2(const std::string& in_path, const std::vector<std::string>& extentions, std::vector<std::string>& out_list, int dir_level)
{
	std::string path = in_path;
	if(path.at(path.size()-1) != '/')
	{
		path.push_back('/');
	}

	DIR *dir = opendir (path.c_str());

	if(dir == NULL || dir_level > 3) return;

	struct dirent *ent = NULL;

	while ((ent = readdir (dir)) != NULL)
	{
		std::string d_name(ent->d_name);
		if(ent->d_type == DT_DIR && d_name.compare(".") != 0 && d_name.compare("..") != 0)
		{
//			std::cout << "Level: " << dir_level << ", Type: " << (ent->d_type == DT_DIR) << ", Name: " << d_name << std::endl;

			if(dir_level >= 1 && (d_name.compare("Camera1") == 0 || d_name.compare("Camera2") == 0))
			{
				GetFileNameInFolder(path + d_name, extentions, out_list);
			}

			GetImageNamesInAllSubfoldersCam1Cam2(path + d_name, extentions, out_list, dir_level+1);
		}
	}

	closedir (dir);
}

void System::GetFileNameInFolder(const std::string& in_path, const std::vector<std::string>& extentions, std::vector<std::string>& out_list, bool bFullPath)
{
	std::string path = in_path;

	if(path.at(path.size()-1) != '/')
	{
		path.push_back('/');
	}

	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (path.c_str())) != NULL)
	{
	  while ((ent = readdir (dir)) != NULL)
	  {
		string str(ent->d_name);
		std::string file_extention;
		int index_last = str.find_last_of(".");
		if(index_last > 0)
		{
			file_extention = str.substr(index_last, str.size());
		}

		for(auto& c : file_extention)
		{
			c = std::toupper(c);
		}

		for(auto& e: extentions)
		{
			std::string ext = e;
			for(auto& c : ext)
			{
				c = std::toupper(c);
			}

			if(file_extention.compare(ext) == 0)
			{
				if(bFullPath)
				{
					out_list.push_back(path+str);
				}
				else
				{
					out_list.push_back(str);
				}
			}
		}
	  }
	  closedir (dir);
	}
}

 std::string System::GetHomeDirectory()
 {
	struct passwd *pw = getpwuid(getuid());
	const char *homedir = pw->pw_dir;
	return string(homedir);
 }

PIDController::PIDController()
{
	kp = kp_v = 0;
	ki = ki_v = 0;
	kd = kd_v = 0;
	pid_lim = pid_v = 0;
	upper_limit = lower_limit = 0;
	bEnableLimit= false;
	accumErr = 0;
	prevErr = 0;
	bResetD = true;
	bResetI = false;
	m_dt = 0;
	m_LastValidErrorDiff = 0;
}

PIDController::PIDController(const double& kp, const double& ki, const double& kd)
{
	Init(kp, ki, kd);
	upper_limit = lower_limit = 0;
	bEnableLimit= false;
	accumErr = 0;
	prevErr  = 0;
	bResetD = true;
	bResetI = false;
	m_LastValidErrorDiff = 0;
}

void PIDController::Setlimit(const double& upper,const double& lower)
{
	upper_limit = upper;
	lower_limit = lower;
	bEnableLimit = true;
}

double PIDController::getPID(const double& currValue, const double& targetValue)
{
	double e = targetValue - currValue;
	return getPID(e);
}

double PIDController::getTimeDependentPID(const double& e, const double& dt)
{
	m_dt = dt;

	if(bResetI)
	{
		bResetI = false;
		accumErr = 0;
	}

	if(bResetD)
	{
		bResetD = false;
		prevErr = e;
	}

	if(pid_v < upper_limit && pid_v > lower_limit)
	{
		accumErr = accumErr + (e*dt);
	}


	double edot = (e - prevErr) / dt;
//	if(dt > 0 && fabs(edot) > 0)
//	{
//		m_LastValidErrorDiff = edot/dt;
//	}
//
//	m_edot_list.push_back(m_LastValidErrorDiff);
//
//	if(m_edot_list.size() > 5)
//	{
//		double _sum = 0;
//		for(auto& v: m_edot_list)
//		{
//			_sum += v;
//		}
//
//		m_LastValidErrorDiff = _sum/m_edot_list.size();
//		m_edot_list.erase(m_edot_list.begin()+0);
//	}

	kp_v = kp * e;
	ki_v = ki * accumErr;
	kd_v = kd * edot;

	pid_v = kp_v + ki_v + kd_v;
	pid_lim = pid_v;
	if(bEnableLimit)
	{
		if(pid_v > upper_limit)
		{
			pid_lim = upper_limit;
		}
		else if ( pid_v < lower_limit)
		{
			pid_lim = lower_limit;
		}
	}

	prevErr = e;

	return pid_lim;
}

double PIDController::getPID(const double& e)
{
	//TODO Remember to add sampling time and multiply the time elapsed by the error
	//complex PID error calculation
	//TODO //De = ( e(i) + 3*e(i-1) - 3*e(i-2) - e(i-3) ) / 6


	if(bResetI)
	{
		bResetI = false;
		accumErr = 0;
	}

	if(bResetD)
	{
		bResetD = false;
		prevErr = e;
	}

	if(pid_v < upper_limit && pid_v > lower_limit)
	{
		accumErr += e;
	}

	double edot= (e - prevErr);

	kp_v = kp * e;
	ki_v = ki * accumErr;
	kd_v = kd * edot;

	pid_v = kp_v + ki_v + kd_v;
	pid_lim = pid_v;
	if(bEnableLimit)
	{
		if(pid_v > upper_limit)
		{
			pid_lim = upper_limit;
		}
		else if ( pid_v < lower_limit)
		{
			pid_lim = lower_limit;
		}
	}

	prevErr = e;

	return pid_lim;
}

std::string PIDController::ToStringHeader()
{
	std::ostringstream str_out;
	str_out << "Time" << "," <<"KP" << "," << "KI" << "," << "KD" << "," << "KP_v" << "," << "KI_v" << "," << "KD_v"
			<< "," << "pid_v_cut" << "," << "pid_v" << "," << "err" << "," << "accumErr" << "," ;
	return str_out.str();
}

std::string PIDController::ToString(const double& dt)
{
	std::ostringstream str_out;
	timespec t_stamp;
	Time::GetTickCount(t_stamp);
	if(m_dt > 0)
	{
		str_out << m_dt << "," <<kp << "," << ki << "," << kd << "," << kp_v << "," << ki_v << "," << kd_v
					 << "," << pid_lim << "," << pid_v << "," << prevErr << "," << accumErr << "," ;
	}
	else
	{
		str_out << dt << "," <<kp << "," << ki << "," << kd << "," << kp_v << "," << ki_v << "," << kd_v
							 << "," << pid_lim << "," << pid_v << "," << prevErr << "," << accumErr << "," ;
	}

	return str_out.str();

}

void PIDController::ResetD()
{
	bResetD = true;
}

void PIDController::ResetI()
{
	bResetI = true;
}

void PIDController::Reset()
{
	bResetI = true;
	bResetD = true;
}

void PIDController::Init(const double& kp, const double& ki, const double& kd)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

LowpassFilter::LowpassFilter()
{
	A = 0;
	d1 = 0;
	d2 = 0;
	w0 = 0;
	w1 = 0;
	w2 = 0;

	m = 0;
	sampleF = 0;
	cutOffF = 0;
}

LowpassFilter::~LowpassFilter()
{
//	if(A)
//		delete A;
//	if(d1)
//		delete d1;
//	if(d2)
//		delete d2;
//	if(w0)
//		delete w0;
//	if(w1)
//		delete w1;
//	if(w2)
//		delete w2;
}

LowpassFilter::LowpassFilter(const int& filterOrder, const double& sampleFreq, const double& cutOffFreq)
{
	Init(filterOrder, sampleFreq, cutOffFreq);
}

void LowpassFilter::Init(const double& sampleFreq, const double& cutOffFreq, const int& n)
{
	if(!(n == 2 || n == 4 || n == 6 || n == 8))
	{
		cout << "Undefined LowpassFilter order ! " << endl;

		A = 0;
		d1 = 0;
		d2 = 0;
		w0 = 0;
		w1 = 0;
		w2 = 0;

		m = 0;
		sampleF = 0;
		cutOffF = 0;
	}
	else
	{
		m = n/2;
		sampleF = sampleFreq;
		cutOffF = cutOffFreq;
		double ep = 1;
		double s = sampleFreq;
		double f = cutOffFreq;
		double a = tan(M_PI*f/s);
		double a2 = a*a;
		double u = log((1.0+sqrt(1.0+ep*ep))/ep);
		double su = sinh(u/(double)n);
		double cu = cosh(u/(double)n);
		double b, c;

		for(int i=0; i< m; ++i)
		{
		    b = sin(M_PI*(2.0*i+1.0)/(2.0*n))*su;
		    c = cos(M_PI*(2.0*i+1.0)/(2.0*n))*cu;
		    c = b*b + c*c;
		    s = a2*c + 2.0*a*b + 1.0;
		    A = a2/(4.0*s); // 4.0
		    d1 = 2.0*(1-a2*c)/s;
		    d2 = -(a2*c - 2.0*a*b + 1.0)/s;
		}
	}
}

double LowpassFilter::getFilter(const double& value)
{
	double ep = 2.3/1.0; // used to normalize
	double x = value;
	for(int i=0; i<m; ++i)
	{
		w0 = d1*w1 + d2*w2 + x;
		x = A*(w0 + 2.0*w1 + w2);
		w2 = w1;
		w1 = w0;
	}
	return ep*x;
}


int XmlHelpers::findElements(std::string name, TiXmlElement* parent_element, std::vector<TiXmlElement*>& element_list)
{
	if(parent_element == nullptr)
	{
		return element_list.size();
	}
	else if(name.compare(parent_element->Value()) == 0)
	{
		element_list.push_back(parent_element);
	}

	findElements(name, parent_element->FirstChildElement(), element_list);
	findElements(name, parent_element->NextSiblingElement(), element_list);

	return element_list.size();
}

int XmlHelpers::findFirstElement(std::string name, TiXmlElement* parent_element, std::vector<TiXmlElement*>& element_list)
{
	if(parent_element == nullptr  || element_list.size()>0)
	{
		return element_list.size();
	}

	else if(name.compare(parent_element->Value()) == 0)
	{
		element_list.push_back(parent_element);
		return element_list.size();
	}

	findFirstElement(name, parent_element->FirstChildElement(), element_list);
	findFirstElement(name, parent_element->NextSiblingElement(), element_list);

	return element_list.size();
}

int XmlHelpers::getIntAttribute(TiXmlElement* p_elem, std::string name, int def_val )
{
	if(p_elem != nullptr && p_elem->Attribute(name) != nullptr)
		return strtol(p_elem->Attribute(name.c_str()), NULL, 10);
	else
		return def_val;
}

double XmlHelpers::getDoubleAttribute(TiXmlElement* p_elem, std::string name, double def_val )
{
	if(p_elem != nullptr && p_elem->Attribute(name) != nullptr)
	{
		return strtod(p_elem->Attribute(name.c_str()), NULL);
	}
	else
		return def_val;
}

std::string XmlHelpers::getStringAttribute(TiXmlElement* p_elem, std::string name, std::string def_val)
{
	if(p_elem != nullptr && p_elem->Attribute(name) != nullptr)
		return std::string(p_elem->Attribute(name.c_str()));
	else
		return def_val;
}

std::string XmlHelpers::getStringValue(TiXmlElement* p_elem, std::string def_val)
{
	if(p_elem != nullptr && p_elem->Value() != nullptr)
		return p_elem->ValueStr();
	else
		return def_val;
}

std::vector<std::string> XmlHelpers::splitString(const std::string& str, const std::string& token)
{
	std::vector<std::string> str_parts;
	int iFirstPart = str.find(token);
	if(iFirstPart > 0)
	{
		str_parts.push_back(str.substr(0, iFirstPart));
	}
	else
	{
		str_parts.push_back(str.substr(0, str.size()));
	}

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

}
}
