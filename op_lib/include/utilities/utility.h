
/// \file UtilityH.h
/// \brief General Math and Control utility functions
/// \author Hatem Darweesh
/// \date May 14, 2016

#ifndef UTILITYH_H_
#define UTILITYH_H_

#include <assert.h>
#include <string>
#include <math.h>
#include <vector>
#include <tinyxml.h>
#include "dirent.h"


namespace op 
{
  namespace utilities
{

#define SIGN(x) (x > 0) ? 1 : ((x < 0) ? -1 : 0)
#define MIN(x,y) (x <= y ? x : y)
#define MAX(x,y) (x >= y ? x : y)
#ifndef DEG2RAD
	#define DEG2RAD M_PI / 180.0
#endif
#ifndef RAD2DEG
	#define RAD2DEG 180.0 / M_PI
#endif

class Angle
{
public:
  static double FixNegativeAngle(const double& a);
  static double SplitPositiveAngle(const double& a);
  static double InverseAngle(const double& a);
  static double AngleBetweenTwoAnglesPositive(const double& a1, const double& a2);
  static double GetCircularAngle(const double& prevContAngle, const double& prevAngle, const double& currAngle);
  static int GetSign(double x);
  static double GetMomentumScaleFactor(const double& v);
  static int CompareDouble(const double& a, const double& b);
};

class Time
{
public:
  static void GetTickCount(struct timespec& t);
  static std::string GetFilePrefixHourMinuteSeconds();
  static double GetTimeDiffNow(const struct timespec& old_t);
  static double GetTimeDiff(const struct timespec& old_t,const struct timespec& curr_t);
  static std::string GetDateTimeStr();
  static int tsCompare (struct  timespec  time1,   struct  timespec  time2, int micro_tolerance = 10);
  static timespec GetTimeSpec(const time_t& srcT);
  static time_t GetLongTime(const struct timespec& srcT);
};

class System
{
  public:
    static std::string GetHomeDirectory();
	static void GetFileNameInFolder(const std::string& path, const std::vector<std::string>& extentions, std::vector<std::string>& out_list, bool bFullPath = true);

	//This function will go only 3 levels
	static void GetFileNameInAllSubfolders(const std::string& path, const std::vector<std::string>& extentions, std::vector<std::string>& out_list, int dir_level = 0);
	static void GetImageNamesInAllSubfoldersCam1Cam2(const std::string& path, const std::vector<std::string>& extentions, std::vector<std::string>& out_list, int dir_level = 0);
	static std::string GetFileNameFromPathName(const std::string& path_with_name);
	static std::string GetFileNameWithoutExtention(const std::string& file_name);
};

class PIDController
{
public:
	PIDController();
	PIDController(const double& kp, const double& ki, const double& kd);
	void Init(const double& kp, const double& ki, const double& kd);
	void Setlimit(const double& upper,const double& lower);
	double getPID(const double& currValue, const double& targetValue);
	double getPID(const double& e);
	double getTimeDependentPID(const double& e, const double& dt);
	void ResetD();
	void ResetI();
	void Reset();
	std::string ToString(const double& dt = 0);
	std::string ToStringHeader();


private:
	double kp;
	double ki;
	double kd;
	double kp_v;
	double ki_v;
	double kd_v;
	double pid_v;
	double pid_lim;
	double upper_limit;
	double lower_limit;
	bool   bEnableLimit;
	double accumErr;
	double prevErr;
	bool bResetD;
	bool bResetI;
	double m_dt;
	double m_LastValidErrorDiff;
	std::vector<double> m_edot_list;
};

class LowpassFilter
{
public:
	LowpassFilter();
	virtual ~LowpassFilter();

	LowpassFilter(const int& filterOrder, const double& sampleFreq, const double& cutOffFreq);
	void Init(const double& sampleFreq, const double& cutOffFreq, const int& filterOrder = 4);
	double getFilter(const double& value);


private:
	int m;
	double sampleF;
	double cutOffF;
	double A  ;
	double d1 ;
	double d2 ;
	double w0 ;
	double w1 ;
	double w2 ;

};

class XmlHelpers
{

public:

	static int findElements(std::string name, TiXmlElement* parent_element, std::vector<TiXmlElement*>& element_list);
	static int findFirstElement(std::string name, TiXmlElement* parent_element, std::vector<TiXmlElement*>& element_list);
	static int getIntAttribute(TiXmlElement* p_elem, std::string name, int def_val = 0);
	static double getDoubleAttribute(TiXmlElement* p_elem, std::string name, double def_val = 0.0);
	static std::string getStringAttribute(TiXmlElement* p_elem, std::string name, std::string def_val);
	static std::string getStringValue(TiXmlElement* p_elem, std::string def_val);
	static std::vector<std::string> splitString(const std::string& str, const std::string& token);
};

}
}

#endif


