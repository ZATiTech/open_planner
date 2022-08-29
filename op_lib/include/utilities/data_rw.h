
/// \file DataRW.h
/// \brief File operations for loading vector map files, loading kml map files and writing log .csv files
/// \author Hatem Darweesh
/// \date Jun 23, 2016


#ifndef DATARW_H_
#define DATARW_H_

#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include <ostream>
#include <limits>
#include <utilities/utility.h>

namespace op 
{
  namespace utilities
{

class DataRW
{
public:
	DataRW();
	virtual ~DataRW();

	static std::string LoggingMainfolderName;
	static std::string ControlLogFolderName;
	static std::string PathLogFolderName;
	static std::string GlobalPathLogFolderName;
	static std::string StatesLogFolderName;
	static std::string SimulationFolderName;
	static std::string KmlMapsFolderName;
	static std::string PredictionFolderName;
	static std::string TrackingFolderName;
	static std::string ExperimentsFolderName;

	static void WriteKMLFile(const std::string& fileName, const std::vector<std::string>& gps_list);
	static void WriteKMLFile(const std::string& fileName, const std::vector<std::vector<std::string> >& gps_list);
	static void WriteLogData(const std::string& logFolder, const std::string& logTitle, const std::string& header, const std::vector<std::string>& logData);
	static void CreateLoggingMainFolder();
	static void CreateLoggingFolders(const std::string& mainFolderName);
	static void CreateExperimentFolder(const std::string& folderName);

	static void writeCSVFile(const std::string& folder, const std::string& title,
			const std::string& header,
			const std::vector<std::string>& data_list);
};

class CSV_File_Reader_Base
{
protected:
	std::ifstream* p_file_;
	std::vector<std::string> headers_;
	std::vector<std::string> data_titles_header_;
	std::vector<std::vector<std::vector<std::string> > > all_data_;

	void ReadHeaders()
	{
		if(!p_file_->is_open()) return;
		std::string strLine;
		headers_.clear();
		if(!p_file_->eof())
		{
			getline(*p_file_, strLine);
			headers_.push_back(strLine);
			ParseHeaderLine(strLine);
		}
	}
	void ParseHeaderLine(const std::string& header)
	{
		if(header.size()==0) return;

		std::string innerToken;
		std::istringstream str_stream(header);
		data_titles_header_.clear();
		while(getline(str_stream, innerToken, ','))
		{
			data_titles_header_.push_back(innerToken);
		}
	}

	bool ReadSingleLine(std::vector<std::vector<std::string> >& line)
	{
		if(!p_file_->is_open() || p_file_->eof()) return false;

			std::string strLine, innerToken;
			line.clear();
			getline(*p_file_, strLine);
			std::istringstream str_stream(strLine);

			std::vector<std::string> obj_part;

			while(getline(str_stream, innerToken, ','))
			{
				obj_part.push_back(innerToken);
			}

			line.push_back(obj_part);
			return true;
	}

public:
	CSV_File_Reader_Base(const std::string& file_name)
	{
		p_file_ = new std::ifstream(file_name.c_str(), std::ios::in);
		if(!p_file_->is_open())
		{
		  printf("\n Can't Open CSV File !, %s", file_name.c_str());
		  return;
		}

		  p_file_->precision(16);

		  ReadHeaders();
	}

	virtual ~CSV_File_Reader_Base()
	{
		if(p_file_->is_open())
		{
			p_file_->close();
		}
	}

};

class SimpleReaderBase
{
private:
	std::vector<std::string> m_RawHeaders;
	std::vector<std::string> m_DataTitlesHeader;
	std::vector<std::vector<std::vector<std::string> > > m_AllData;
	int m_nHeders;
	int m_iDataTitles;
	int m_nVarPerObj;
	int m_nLineHeaders;
	std::string m_HeaderRepeatKey;
	char m_Separator;

	void ReadHeaders();
	void ParseDataTitles(const std::string& header);

public:
	/**
	 *
	 * @param fileName log file name
	 * @param nHeaders number of data headers
	 * @param iDataTitles which row contains the data titles
	 * @param nVariablesForOneObject 0 means each row represents one object
	 */
	SimpleReaderBase(const std::string& path, const int& nHeaders = 2, const std::string& csv_file_name = "", const char& separator = ',',
			const int& iDataTitles = 1, const int& nVariablesForOneObject = 0,
			const int& nLineHeaders = 0, const std::string& headerRepeatKey = "...");

	virtual ~SimpleReaderBase();
	virtual int ReadAllData() = 0;
	std::string header_;
	std::string file_name_;

protected:
	std::ifstream m_File;
	bool ReadSingleLine(std::vector<std::vector<std::string> >& line);

};

class GPSDataReader : public SimpleReaderBase
{
public:
	struct GPSBasicData
	{
		double lat;
		double lon;
		double alt;
		double dir;
		double distance;

	};

	public:
	GPSDataReader(const std::string& fileName) : SimpleReaderBase(fileName){}
	virtual ~GPSDataReader(){}

	bool ReadNextLine(GPSBasicData& data);
	int ReadAllData(std::vector<GPSBasicData>& data_list);
	int ReadAllData();
};

class SimulationFileReader : public SimpleReaderBase
{
public:
	struct SimulationPoint
	{
		double x;
		double y;
		double z;
		double a;
		double c;
		double v;
		std::string name;
	};

	struct SimulationData
	{
		SimulationPoint startPoint;
		SimulationPoint goalPoint;
		std::vector<SimulationPoint> simuCars;
	};

	SimulationFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1){}
	virtual ~SimulationFileReader(){}

	bool ReadNextLine(SimulationPoint& data);
	int ReadAllData(SimulationData& data_list);
	int ReadAllData();
};

class LocalizationPathReader : public SimpleReaderBase
{
public:
	struct LocalizationWayPoint
	{
		double x;
		double y;
		double z;
		double a;
		double v;
	};

	LocalizationPathReader(const std::string& fileName, const char& separator) : SimpleReaderBase(fileName, 1, "", separator){}
	virtual ~LocalizationPathReader(){}

	bool ReadNextLine(LocalizationWayPoint& data);
	int ReadAllData(std::vector<LocalizationWayPoint>& data_list);
	int ReadAllData();
};

class TimeStampedPathReader : public SimpleReaderBase
{
public:
	struct TimePoint
	{
		double t;
		double x;
		double y;
		double z;
		double a;
	};

	TimeStampedPathReader(const std::string& fileName, const char& separator) : SimpleReaderBase(fileName, 1, "", separator){}
	virtual ~TimeStampedPathReader(){}

	bool ReadNextLine(TimePoint& data);
	int ReadAllData(std::vector<TimePoint>& data_list);
	int ReadAllData();
};

class DatasetTrajectoryReader : public SimpleReaderBase
{
public:
	struct TimePoint
	{
		double t;
		double x;
		double y;
		double z;
		double roll;
		double pitch;
		double yaw;
	};

	DatasetTrajectoryReader(const std::string& fileName, const char& separator) : SimpleReaderBase(fileName, 1, "", separator)
	{
		header_ = "Time[s],Easting[m],Northing[m],Roll[deg],Pitch[deg],Yaw[deg],Height[m]";
	}
	virtual ~DatasetTrajectoryReader(){}

	bool ReadNextLine(TimePoint& data);
	int ReadAllData(std::vector<TimePoint>& data_list);
	int ReadAllData();

	friend std::ostream& operator<<(std::ostream& os, const TimePoint& obj)
	{
	    os << obj.t << ","
	    << obj.x << ","
	    << obj.y << ","
		<< obj.roll << ","
	    << obj.pitch << ","
	    << obj.yaw << ","
	    << obj.z ;
	    return os;
	}
};

class DestinationsDataFileReader : public SimpleReaderBase
{
public:
	struct DestinationData
	{
		int id;
		std::string name;
		double lon;
		double lat;
		double alt;
		double x;
		double y;
		double z;
		double angle;
		int hour; //0->24
		int minute; //0->59
	};

	DestinationsDataFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "destinations.csv")
	{
		header_ = "ID,Name,Longitude,latitude,Altitude,X,Y,Z,Angle,Hour,Minute";
	}

	DestinationsDataFileReader(const DestinationData& proj_data);
	virtual ~DestinationsDataFileReader(){}

	bool ReadNextLine(DestinationData& data);
	int ReadAllData(std::vector<DestinationData>& data_list);
	int ReadAllData();
	std::vector<DestinationData> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const DestinationData& obj)
	{
	    os << obj.id << ","
	    << obj.name << ","
	    << obj.lon << ","
		<< obj.lat << ","
	    << obj.alt << ","
	    << obj.x << ","
	    << obj.y << ","
	    << obj.z << ","
	    << obj.angle << ","
	    << obj.hour << ","
	    << obj.minute;
	    return os;
	}
};

class ProjectionDataFileReader : public SimpleReaderBase
{
public:
	struct ProjectionData
	{
		std::string proj_type;
		std::string proj_str;
		double lon;
		double lat;
		double alt;
		double x;
		double y;
		double z;
	};

	ProjectionDataFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "projection.csv")
	{
		header_ = "ProjType,ProjString,Longitude,Latitude,Altitude,X,Y,Z";
	}

	ProjectionDataFileReader(const ProjectionData& proj_data);
	virtual ~ProjectionDataFileReader(){}

	bool ReadNextLine(ProjectionData& data);
	int ReadAllData(std::vector<ProjectionData>& data_list);
	int ReadAllData();
	std::vector<ProjectionData> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const ProjectionData& obj)
	{
	    os << obj.proj_type << ","
		<< obj.proj_str << ","
	    << obj.lon << ","
	    << obj.lat << ","
	    << obj.alt << ","
	    << obj.x << ","
	    << obj.y << ","
	    << obj.z;
	    return os;
	}
};

class AisanPointsFileReader : public SimpleReaderBase
{
public:
	struct AisanPoints
	{
		int PID;
		double B;
		double L;
		double H;
		double Bx;
		double Ly;
		int Ref;
		int MCODE1;
		int MCODE2;
		int MCODE3;
	};

	AisanPointsFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "point.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "PID,B,L,H,Bx,Ly,Ref,MCODE1,MCODE2,MCODE3";
	}

	virtual ~AisanPointsFileReader(){}

	bool ReadNextLine(AisanPoints& data);
	int ReadAllData(std::vector<AisanPoints>& data_list);
	int ReadAllData();
	AisanPoints* GetDataRowById(int _pid);
	std::vector<AisanPoints> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanPoints& obj)
	{
	    os << obj.PID << ","
	    << obj.B << ","
	    << obj.L << ","
	    << obj.H << ","
	    << obj.Bx << ","
	    << obj.Ly << ","
	    << obj.Ref << ","
	    << obj.MCODE1 << ","
	    << obj.MCODE2 << ","
	    << obj.MCODE3;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanPoints*> m_data_map;
};

class AisanNodesFileReader : public SimpleReaderBase
{
public:

	struct AisanNode
	{
		int NID;
		int PID;
	};

	AisanNodesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "node.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "NID,PID";
	}

	virtual ~AisanNodesFileReader(){}

	bool ReadNextLine(AisanNode& data);
	int ReadAllData(std::vector<AisanNode>& data_list);
	int ReadAllData();
	AisanNode* GetDataRowById(int _nid);
	std::vector<AisanNode> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanNode& obj)
	{
	    os << obj.NID << ","
	    << obj.PID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanNode*> m_data_map;
};

class AisanLinesFileReader : public SimpleReaderBase
{
public:

	struct AisanLine
	{
		int LID;
		int BPID;
		int FPID;
		int BLID;
		int FLID;
	};

	AisanLinesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "line.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "LID,BPID,FPID,BLID,FLID";
	}
	virtual ~AisanLinesFileReader(){}

	bool ReadNextLine(AisanLine& data);
	int ReadAllData(std::vector<AisanLine>& data_list);
	int ReadAllData();
	AisanLine* GetDataRowById(int _lid);
	std::vector<AisanLine> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanLine& obj)
	{
	    os << obj.LID << ","
	    << obj.BPID << ","
	    << obj.FPID << ","
	    << obj.BLID << ","
	    << obj.FLID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanLine*> m_data_map;
};

class AisanCLinesFileReader : public SimpleReaderBase
{
public:

	struct AisanCLine
	{
		int ID;
		int LID;
		double width;
		char color;
		int type;
		int LinkID;
	};

	AisanCLinesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "cline.csv")
	{
		header_ = "ID,LID,width,color,type,LinkID";
	}
	virtual ~AisanCLinesFileReader(){}

	bool ReadNextLine(AisanCLine& data);
	int ReadAllData(std::vector<AisanCLine>& data_list);
	int ReadAllData();

	friend std::ostream& operator<<(std::ostream& os, const AisanCLine& obj)
	{
	    os << obj.ID << ","
	    << obj.LID << ","
	    << obj.width << ","
	    << obj.color << ","
	    << obj.type << ","
	    << obj.LinkID;
	    return os;
	}
};

class AisanCenterLinesFileReader : public SimpleReaderBase
{
public:

	struct AisanCenterLine
	{
		int 	DID;
		int 	Dist;
		int 	PID;
		double 	Dir;
		double 	Apara;
		double 	r;
		double 	slope;
		double 	cant;
		double 	LW;
		double 	RW;
	};

	AisanCenterLinesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "dtlane.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "DID,Dist,PID,Dir,Apara,r,slope,cant,LW,RW";
	}
	virtual ~AisanCenterLinesFileReader(){}

	bool ReadNextLine(AisanCenterLine& data);
	int ReadAllData(std::vector<AisanCenterLine>& data_list);
	int ReadAllData();
	AisanCenterLine* GetDataRowById(int _lnid);
	std::vector<AisanCenterLine> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanCenterLine& obj)
	{
	    os << obj.DID << ","
	    << obj.Dist << ","
	    << obj.PID << ","
	    << obj.Dir << ","
	    << obj.Apara << ","
	    << obj.r << ","
	    << obj.slope << ","
	    << obj.cant << ","
	    << obj.LW << ","
	    << obj.RW;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanCenterLine*> m_data_map;
};

class AisanAreasFileReader : public SimpleReaderBase
{
public:

	struct AisanArea
	{
		int 	AID;
		int 	SLID;
		int 	ELID;
	};

	AisanAreasFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "area.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "AID,SLID,ELID";
	}
	virtual ~AisanAreasFileReader(){}

	bool ReadNextLine(AisanArea& data);
	int ReadAllData(std::vector<AisanArea>& data_list);
	int ReadAllData();
	AisanArea* GetDataRowById(int _lnid);
	std::vector<AisanArea> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanArea& obj)
	{
	    os << obj.AID << ","
	    << obj.SLID << ","
	    << obj.ELID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanArea*> m_data_map;
};

class AisanIntersectionFileReader : public SimpleReaderBase
{
public:

	struct AisanIntersection
	{
		int 	ID;
		int 	AID;
		int 	LinkID;
	};

	AisanIntersectionFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "intersection.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,AID,LinkID";
	}
	virtual ~AisanIntersectionFileReader(){}

	bool ReadNextLine(AisanIntersection& data);
	int ReadAllData(std::vector<AisanIntersection>& data_list);
	int ReadAllData();
	AisanIntersection* GetDataRowById(int _lnid);
	std::vector<AisanIntersection> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanIntersection& obj)
	{
	    os << obj.ID << ","
	    << obj.AID << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanIntersection*> m_data_map;
};

class AisanLanesFileReader : public SimpleReaderBase
{
public:

	struct AisanLane
	{
		int LnID	;
		int DID		;
		int BLID	;
		int FLID	;
		int BNID	;
		int FNID	;
		int JCT		;
		int BLID2	;
		int BLID3	;
		int BLID4	;
		int FLID2	;
		int FLID3	;
		int FLID4	;
		int ClossID	;
	 	double Span	;
		int LCnt	;
		int Lno		;
		int LaneType;
		int LimitVel;
		int RefVel	;
		int RoadSecID;
		int LaneChgFG;
		int LinkWAID;
	    char LaneDir;
	    int  LeftLaneId;
		int RightLaneId;

		int originalMapID;
	};

	AisanLanesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "lane.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "LnID,DID,BLID,FLID,BNID,FNID,JCT,BLID2,BLID3,BLID4,FLID2,FLID3,"
					"FLID4,ClossID,Span,LCnt,Lno,LaneType,LimitVel,RefVel,RoadSecID,LaneChgFG,LinkWAID";
	}	virtual ~AisanLanesFileReader(){}

	bool ReadNextLine(AisanLane& data);
	int ReadAllData(std::vector<AisanLane>& data_list);
	int ReadAllData();
  AisanLane* GetDataRowById(int _lnid);
	std::vector<AisanLane> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanLane& obj)
	{
	    os << obj.LnID				<< ","
	       << obj.DID					<< ","
	       << obj.BLID				<< ","
	       << obj.FLID	 			<< ","
	       << obj.BNID	 			<< ","
	       << obj.FNID	 			<< ","
	       << obj.JCT		 			<< ","
	       << obj.BLID2	 			<< ","
	       << obj.BLID3	 			<< ","
	       << obj.BLID4	 			<< ","
	       << obj.FLID2	 			<< ","
	       << obj.FLID3	 			<< ","
	       << obj.FLID4	 			<< ","
	       << obj.ClossID	 			<< ","
	       << obj.Span	 			<< ","
	       << obj.LCnt	 			<< ","
	       << obj.Lno		 			<< ","
	       << obj.LaneType 			<< ","
	       << obj.LimitVel 			<< ","
	       << obj.RefVel	 			<< ","
	       << obj.RoadSecID 			<< ","
	       << obj.LaneChgFG 			<< ","
	       << obj.LinkWAID 			<< ","
	       << obj.LaneDir 			<< ","
	       << obj.LeftLaneId 			<< ","
	       << obj.RightLaneId;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanLane*> m_data_map;
};

class AisanStopLineFileReader : public SimpleReaderBase
{
public:

	struct AisanStopLine
	{
		int 	ID;
		int 	LID;
		int 	TLID;
		int 	SignID;
		int 	LinkID;
	};

	AisanStopLineFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "stopline.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,LID,TLID,SignID,LinkID";
	}
	virtual ~AisanStopLineFileReader(){}

	bool ReadNextLine(AisanStopLine& data);
	int ReadAllData(std::vector<AisanStopLine>& data_list);
	int ReadAllData();
	AisanStopLine* GetDataRowById(int _lnid);
	std::vector<AisanStopLine> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanStopLine& obj)
	{
	    os << obj.ID << ","
	    << obj.LID << ","
	    << obj.TLID << ","
	    << obj.SignID << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanStopLine*> m_data_map;
};

class AisanRoadSignFileReader : public SimpleReaderBase
{
public:

	struct AisanRoadSign
	{
		int 	ID;
		int 	VID;
		int 	PLID;
		int 	Type;
		int 	LinkID;
	};

	AisanRoadSignFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "roadsign.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,VID,PLID,Type,LinkID";
	}
	virtual ~AisanRoadSignFileReader(){}

	bool ReadNextLine(AisanRoadSign& data);
	int ReadAllData(std::vector<AisanRoadSign>& data_list);
	int ReadAllData();
	AisanRoadSign* GetDataRowById(int _lnid);
	std::vector<AisanRoadSign> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanRoadSign& obj)
	{
	    os << obj.ID << ","
	    << obj.VID << ","
	    << obj.PLID << ","
	    << obj.Type << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanRoadSign*> m_data_map;
};

class AisanSignalFileReader : public SimpleReaderBase
{
public:

	struct AisanSignal
	{
		int 	ID;
		int 	VID;
		int 	PLID;
		int 	Type;
		int 	LinkID;
	};

	AisanSignalFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "signaldata.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,VID,PLID,Type,LinkID";
	}
	virtual ~AisanSignalFileReader(){}

	bool ReadNextLine(AisanSignal& data);
	int ReadAllData(std::vector<AisanSignal>& data_list);
	int ReadAllData();
	AisanSignal* GetDataRowById(int _lnid);
	std::vector<AisanSignal> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanSignal& obj)
	{
	    os << obj.ID << ","
	    << obj.VID << ","
	    << obj.PLID << ","
	    << obj.Type << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanSignal*> m_data_map;
};

class AisanVectorFileReader : public SimpleReaderBase
{
public:

	struct AisanVector
	{
		int 	VID;
		int 	PID;
		double 	Hang;
		double 	Vang;
	};

	AisanVectorFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "vector.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "VID,PID,hang,Vang";
	}
	virtual ~AisanVectorFileReader(){}

	bool ReadNextLine(AisanVector& data);
	int ReadAllData(std::vector<AisanVector>& data_list);
	int ReadAllData();
	AisanVector* GetDataRowById(int _lnid);
	std::vector<AisanVector> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanVector& obj)
	{
	    os << obj.VID << ","
	    << obj.PID << ","
	    << obj.Hang << ","
	    << obj.Vang;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanVector*> m_data_map;
};

class AisanCurbFileReader : public SimpleReaderBase
{
public:

	struct AisanCurb
	{
		int 	ID;
		int 	LID;
		double 	Height;
		double 	Width;
		int 	dir;
		int 	LinkID;
	};

	AisanCurbFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "curb.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,LID,Height,Width,dir,LinkID";
	}
	virtual ~AisanCurbFileReader(){}

	bool ReadNextLine(AisanCurb& data);
	int ReadAllData(std::vector<AisanCurb>& data_list);
	int ReadAllData();
	AisanCurb* GetDataRowById(int _lnid);
	std::vector<AisanCurb> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanCurb& obj)
	{
	    os << obj.ID << ","
	    << obj.LID << ","
	    << obj.Height << ","
	    << obj.Width << ","
	    << obj.dir << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanCurb*> m_data_map;
};

class AisanCrossWalkFileReader : public SimpleReaderBase
{
public:

	struct AisanCrossWalk
	{
		int 	ID;
		int 	AID;
		int 	Type;
		int		BdID;
		int 	LinkID;
	};

	AisanCrossWalkFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "crosswalk.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,AID,Type,BdID,LinkID";
	}
	virtual ~AisanCrossWalkFileReader(){}

	bool ReadNextLine(AisanCrossWalk& data);
	int ReadAllData(std::vector<AisanCrossWalk>& data_list);
	int ReadAllData();
	AisanCrossWalk* GetDataRowById(int _lnid);
	std::vector<AisanCrossWalk> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanCrossWalk& obj)
	{
	    os << obj.ID << ","
	    << obj.AID << ","
	    << obj.Type << ","
	    << obj.BdID << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanCrossWalk*> m_data_map;
};

class AisanWayareaFileReader : public SimpleReaderBase
{
public:

	struct AisanWayarea
	{
		int 	ID;
		int 	AID;
		int 	LinkID;
	};

	AisanWayareaFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "wayarea.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,AID,LinkID";
	}
	virtual ~AisanWayareaFileReader(){}

	bool ReadNextLine(AisanWayarea& data);
	int ReadAllData(std::vector<AisanWayarea>& data_list);
	int ReadAllData();
	AisanWayarea* GetDataRowById(int _lnid);
	std::vector<AisanWayarea> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanWayarea& obj)
	{
	    os << obj.ID << ","
	    << obj.AID << ","
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanWayarea*> m_data_map;
};

class AisanWhitelinesFileReader : public SimpleReaderBase
{
public:

	struct AisanWhiteline
	{
		int ID;
		int LID;
		double Width;
		std::string Color;
		int type;
		int LinkID;
	};

	AisanWhitelinesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "whiteline.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,LID,Width,Color,type,LinkID";
	}

	virtual ~AisanWhitelinesFileReader(){}

	bool ReadNextLine(AisanWhiteline& data);
	int ReadAllData(std::vector<AisanWhiteline>& data_list);
	int ReadAllData();
	AisanWhiteline* GetDataRowById(int _wlid);
	std::vector<AisanWhiteline> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanWhiteline& obj)
	{
	    os << obj.ID << ","
	    << obj.LID
	    << obj.Width
	    << obj.Color
	    << obj.type
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanWhiteline*> m_data_map;
};

class AisanGutterFileReader : public SimpleReaderBase
{
public:

	struct AisanGutter
	{
		int ID;
		int AID;
		int Type;
		int LinkID;
	};

	AisanGutterFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "gutter.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,AID,Type,LinkID";
	}

	virtual ~AisanGutterFileReader(){}

	bool ReadNextLine(AisanGutter& data);
	int ReadAllData(std::vector<AisanGutter>& data_list);
	int ReadAllData();
	AisanGutter* GetDataRowById(int _wlid);
	std::vector<AisanGutter> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanGutter& obj)
	{
	    os << obj.ID << ","
	    << obj.AID
	    << obj.Type
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanGutter*> m_data_map;
};

class AisanIdxFileReader : public SimpleReaderBase
{
public:

	struct AisanIdx
	{
		int ID;
		std::string KIND;
		std::string fname;
	};

	AisanIdxFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "idx.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,KIND,fname";
	}


	virtual ~AisanIdxFileReader(){}

	bool ReadNextLine(AisanIdx& data);
	int ReadAllData(std::vector<AisanIdx>& data_list);
	int ReadAllData();
	//void ParseNextLine(const vector_map_msgs::WhiteLine& _rec, AisanIdx& data);
	AisanIdx* GetDataRowById(int _wlid);
	std::vector<AisanIdx> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanIdx& obj)
	{
	    os << obj.ID << ","
	    << obj.KIND
	    << obj.fname;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanIdx*> m_data_map;
};

class AisanPoleFileReader : public SimpleReaderBase
{
public:

	struct AisanPole
	{
		int PLID;
		int VID;
		double Length;
		double Dim;
	};

	AisanPoleFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "pole.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "PLID,VID,Length,Dim";
	}

	virtual ~AisanPoleFileReader(){}

	bool ReadNextLine(AisanPole& data);
	int ReadAllData(std::vector<AisanPole>& data_list);
	int ReadAllData();
	AisanPole* GetDataRowById(int _wlid);
	std::vector<AisanPole> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanPole& obj)
	{
	    os << obj.PLID << ","
	    << obj.VID
	    << obj.Length
	    << obj.Dim;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanPole*> m_data_map;
};

class AisanPoledataFileReader : public SimpleReaderBase
{
public:

	struct AisanPoledata
	{
		int ID;
		int PLID;
		int LinkID;
	};

	AisanPoledataFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "poledata.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,PLID,LinkID";
	}

	//AisanPoledataFileReader(const vector_map_msgs::Poledata& _Poledata);
	virtual ~AisanPoledataFileReader(){}

	bool ReadNextLine(AisanPoledata& data);
	int ReadAllData(std::vector<AisanPoledata>& data_list);
	int ReadAllData();
	//void ParseNextLine(const vector_map_msgs::Poledata& _rec, AisanPoledata& data);
	AisanPoledata* GetDataRowById(int _wlid);
	std::vector<AisanPoledata> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanPoledata& obj)
	{
	    os << obj.ID << ","
	    << obj.PLID
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanPoledata*> m_data_map;
};

class AisanRoadEdgeFileReader : public SimpleReaderBase
{
public:

	struct AisanRoadEdge
	{
		int ID;
		int LID;
		int LinkID;
	};

	AisanRoadEdgeFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "roadedge.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,LID,LinkID";
	}

	virtual ~AisanRoadEdgeFileReader(){}

	bool ReadNextLine(AisanRoadEdge& data);
	int ReadAllData(std::vector<AisanRoadEdge>& data_list);
	int ReadAllData();
  AisanRoadEdge* GetDataRowById(int _wlid);
	std::vector<AisanRoadEdge> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanRoadEdge& obj)
	{
	    os << obj.ID << ","
	    << obj.LID
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanRoadEdge*> m_data_map;
};

class AisanSurfacemarkFileReader : public SimpleReaderBase
{
public:

	struct AisanSurfacemark
	{
		int ID;
		int AID;
		int Type;
		int LinkID;
	};

	AisanSurfacemarkFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "road_surface_mark.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,AID,Type,LinkID";
	}

	//AisanSurfacemarkFileReader(const vector_map_msgs::& _Surfacemark);
	virtual ~AisanSurfacemarkFileReader(){}

	bool ReadNextLine(AisanSurfacemark& data);
	int ReadAllData(std::vector<AisanSurfacemark>& data_list);
	int ReadAllData();
	//void ParseNextLine(const vector_map_msgs::Surfacemark& _rec, AisanSurfacemark& data);
	AisanSurfacemark* GetDataRowById(int _wlid);
	std::vector<AisanSurfacemark> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanSurfacemark& obj)
	{
	    os << obj.ID << ","
	    << obj.AID
	    << obj.Type
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanSurfacemark*> m_data_map;
};

class AisanStreetlightFileReader : public SimpleReaderBase
{
public:

	struct AisanStreetlight
	{
		int ID;
		int LID;
		int PLID;
		int LinkID;
	};

	AisanStreetlightFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "streetlight.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,LID,PLID,LinkID";
	}

	virtual ~AisanStreetlightFileReader(){}

	bool ReadNextLine(AisanStreetlight& data);
	int ReadAllData(std::vector<AisanStreetlight>& data_list);
	int ReadAllData();
	AisanStreetlight* GetDataRowById(int _wlid);
	std::vector<AisanStreetlight> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanStreetlight& obj)
	{
	    os << obj.ID << ","
	    << obj.LID
	    << obj.PLID
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanStreetlight*> m_data_map;
};

class AisanUtilitypoleFileReader : public SimpleReaderBase
{
public:

	struct AisanUtilitypole
	{
		int ID;
		int PLID;
		int LinkID;
	};

	AisanUtilitypoleFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "utilitypole.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,PLID,LinkID";
	}

	virtual ~AisanUtilitypoleFileReader(){}

	bool ReadNextLine(AisanUtilitypole& data);
	int ReadAllData(std::vector<AisanUtilitypole>& data_list);
	int ReadAllData();
	AisanUtilitypole* GetDataRowById(int _wlid);
	std::vector<AisanUtilitypole> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanUtilitypole& obj)
	{
	    os << obj.ID << ","
	    << obj.PLID
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanUtilitypole*> m_data_map;
};

class AisanZebrazoneFileReader : public SimpleReaderBase
{
public:

	struct AisanZebrazone
	{
		int ID;
		int AID;
		int LinkID;
	};

	AisanZebrazoneFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1, "zebrazone.csv")
	{
		m_min_id = std::numeric_limits<int>::max();
		header_ = "ID,AID,LinkID";
	}

	virtual ~AisanZebrazoneFileReader(){}

	bool ReadNextLine(AisanZebrazone& data);
	int ReadAllData(std::vector<AisanZebrazone>& data_list);
	int ReadAllData();
	AisanZebrazone* GetDataRowById(int _wlid);
	std::vector<AisanZebrazone> m_data_list;

	friend std::ostream& operator<<(std::ostream& os, const AisanZebrazone& obj)
	{
	    os << obj.ID << ","
	    << obj.AID
	    << obj.LinkID;
	    return os;
	}

private:
	int m_min_id;
	std::vector<AisanZebrazone*> m_data_map;
};

class AisanDataConnFileReader : public SimpleReaderBase
{
public:

	struct DataConn
	{
		int 	LID; // lane id
		int 	SLID; // stop line id
		int 	SID; // signal id
		int 	SSID; // stop sign id
	};

	AisanDataConnFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
	{
		header_ = "LID,SLID,SID,SSID";
	}

	virtual ~AisanDataConnFileReader(){}

	bool ReadNextLine(DataConn& data);
	int ReadAllData(std::vector<DataConn>& data_list);
	int ReadAllData();

	friend std::ostream& operator<<(std::ostream& os, const DataConn& obj)
	{
	    os << obj.LID << ","
	    << obj.SLID << ","
	    << obj.SID << ","
	    << obj.SSID;
	    return os;
	}
};

class MapRaw
{
public:
	AisanLanesFileReader* pLanes;
	AisanPointsFileReader* pPoints;
	AisanCenterLinesFileReader* pCenterLines;
	AisanIntersectionFileReader* pIntersections;
	AisanAreasFileReader* pAreas;
	AisanLinesFileReader* pLines;
	AisanStopLineFileReader* pStopLines;
	AisanSignalFileReader* pSignals;
	AisanVectorFileReader* pVectors;
	AisanCurbFileReader* pCurbs;
	AisanRoadEdgeFileReader* pRoadedges;
	AisanWayareaFileReader* pWayAreas;
	AisanCrossWalkFileReader* pCrossWalks;
	AisanNodesFileReader* pNodes;
	AisanWhitelinesFileReader* pWhitelines;
	AisanGutterFileReader* pGutter;
	AisanIdxFileReader* pIdx;
	AisanPoleFileReader* pPole;
	AisanPoledataFileReader* pPoledata;
	AisanSurfacemarkFileReader* pSurfacemark;
	AisanStreetlightFileReader* pStreetLight;
	AisanUtilitypoleFileReader* pUtilitypole;
	AisanZebrazoneFileReader* pZebrazore;

	std::string map_path;

	struct timespec _time_out;

	MapRaw()
	{
		pLanes = nullptr;
		pPoints = nullptr;
		pCenterLines = nullptr;
		pIntersections = nullptr;
		pAreas = nullptr;
		pLines = nullptr;
		pStopLines = nullptr;
		pSignals = nullptr;
		pVectors = nullptr;
		pCurbs = nullptr;
		pRoadedges = nullptr;
		pWayAreas = nullptr;
		pCrossWalks = nullptr;
		pNodes = nullptr;
		pWhitelines = nullptr;
		pGutter = nullptr;
		pIdx = nullptr;
		pPole = nullptr;
		pPoledata = nullptr;
		pSurfacemark = nullptr;
		pStreetLight = nullptr;
		pUtilitypole = nullptr;
		pZebrazore = nullptr;

		Time::GetTickCount(_time_out);
	}

	virtual ~MapRaw()
	{
		ClearRawMap();
	}

	void ClearRawMap()
	{
		if(pLanes != nullptr)
		{
			delete pLanes;
			pLanes = nullptr;
		}

		if(pPoints != nullptr)
		{
			delete pPoints;
			pPoints = nullptr;
		}

		if(pCenterLines != nullptr)
		{
			delete pCenterLines;
			pCenterLines = nullptr;
		}

		if(pIntersections != nullptr)
		{
			delete pIntersections;
			pIntersections = nullptr;
		}

		if(pAreas != nullptr)
		{
			delete pAreas;
			pAreas = nullptr;
		}

		if(pLines != nullptr)
		{
			delete pLines;
			pLines = nullptr;
		}

		if(pStopLines != nullptr)
		{
			delete pStopLines;
			pStopLines = nullptr;
		}

		if(pSignals != nullptr)
		{
			delete pSignals;
			pSignals = nullptr;
		}

		if(pVectors != nullptr)
		{
			delete pVectors;
			pVectors = nullptr;
		}

		if(pCurbs != nullptr)
		{
			delete pCurbs;
			pCurbs = nullptr;
		}

		if(pRoadedges != nullptr)
		{
			delete pRoadedges;
			pRoadedges = nullptr;
		}

		if(pWayAreas != nullptr)
		{
			delete pWayAreas;
			pWayAreas = nullptr;
		}

		if(pCrossWalks != nullptr)
		{
			delete pCrossWalks;
			pCrossWalks = nullptr;
		}

		if(pNodes != nullptr)
		{
			delete pNodes;
			pNodes = nullptr;
		}

		if(pWhitelines != nullptr)
		{
			delete pWhitelines;
			pWhitelines = nullptr;
		}

		if(pUtilitypole != nullptr)
		{
			delete pUtilitypole;
			pUtilitypole = nullptr;
		}

		if(pGutter != nullptr)
		{
			delete pGutter;
			pGutter = nullptr;
		}

		if(pIdx != nullptr)
		{
			delete pIdx;
			pIdx = nullptr;
		}

		if(pPole != nullptr)
		{
			delete pPole;
			pPole = nullptr;
		}

		if(pPoledata != nullptr)
		{
			delete pPoledata;
			pPoledata = nullptr;
		}

		if(pSurfacemark != nullptr)
		{
			delete pSurfacemark;
			pSurfacemark =  nullptr;
		}

		if(pStreetLight != nullptr)
		{
			delete pStreetLight;
			pStreetLight = nullptr;
		}

		if(pZebrazore != nullptr)
		{
			delete pZebrazore;
			pZebrazore = nullptr;
		}
	}

	void LoadFromFolder(std::string vector_map_folder)
	{
		ClearRawMap();

		this->pAreas = new AisanAreasFileReader(vector_map_folder);
		this->pAreas->ReadAllData();

		this->pCenterLines = new AisanCenterLinesFileReader(vector_map_folder);
		this->pCenterLines->ReadAllData();

		this->pCrossWalks = new AisanCrossWalkFileReader(vector_map_folder);
		this->pCrossWalks->ReadAllData();

		this->pCurbs = new AisanCurbFileReader(vector_map_folder);
		this->pCurbs->ReadAllData();

		this->pGutter = new AisanGutterFileReader(vector_map_folder);
		this->pGutter->ReadAllData();

		this->pIdx = new AisanIdxFileReader(vector_map_folder);
		this->pIdx->ReadAllData();

		this->pIntersections = new AisanIntersectionFileReader(vector_map_folder);
		this->pIntersections->ReadAllData();

		this->pLanes = new AisanLanesFileReader(vector_map_folder);
		this->pLanes->ReadAllData();

		this->pLines = new AisanLinesFileReader(vector_map_folder);
		this->pLines->ReadAllData();

		this->pNodes = new AisanNodesFileReader(vector_map_folder);
		this->pNodes->ReadAllData();

		this->pPoints = new AisanPointsFileReader(vector_map_folder);
		this->pPoints->ReadAllData();

		this->pPole = new AisanPoleFileReader(vector_map_folder);
		this->pPole->ReadAllData();

		this->pPoledata = new AisanPoledataFileReader(vector_map_folder);
		this->pPoledata->ReadAllData();

		this->pRoadedges = new AisanRoadEdgeFileReader(vector_map_folder);
		this->pRoadedges->ReadAllData();

		this->pSignals = new AisanSignalFileReader(vector_map_folder);
		this->pSignals->ReadAllData();

		this->pStopLines = new AisanStopLineFileReader(vector_map_folder);
		this->pStopLines->ReadAllData();

		this->pStreetLight = new AisanStreetlightFileReader(vector_map_folder);
		this->pStreetLight->ReadAllData();

		this->pSurfacemark = new AisanSurfacemarkFileReader(vector_map_folder);
		this->pSurfacemark->ReadAllData();

		this->pUtilitypole = new AisanUtilitypoleFileReader(vector_map_folder);
		this->pUtilitypole->ReadAllData();

		this->pVectors = new AisanVectorFileReader(vector_map_folder);
		this->pVectors->ReadAllData();

		this->pWayAreas = new AisanWayareaFileReader(vector_map_folder);
		this->pWayAreas->ReadAllData();

		this->pWhitelines = new AisanWhitelinesFileReader(vector_map_folder);
		this->pWhitelines->ReadAllData();

		this->pZebrazore = new AisanZebrazoneFileReader(vector_map_folder);
		this->pZebrazore->ReadAllData();
	}

	int GetVersion()
	{
		bool bTimeOut = Time::GetTimeDiffNow(_time_out) > 2.0;
		bool bLoaded =  pLanes != nullptr && pPoints != nullptr && pCenterLines  != nullptr && pNodes  != nullptr;
		int iVersion = 0;
		if(bLoaded && bTimeOut)
		{
			iVersion = 2;
			if(pNodes->m_data_list.size() == 0)
			{
				iVersion = 1;
			}
		}
//		else
//		{
//			bLoaded =  pLanes != nullptr && pPoints != nullptr && pCenterLines  != nullptr;
//			if(bLoaded && bTimeOut)
//			{
//				iVersion = 1;
//				if(pNodes  == nullptr)
//					pNodes = new AisanNodesFileReader(vector_map_msgs::NodeArray());
//			}
//		}

		return iVersion;
	}
};

} /* namespace utilities */

} /* namespace op */

#endif /* data_rw */
