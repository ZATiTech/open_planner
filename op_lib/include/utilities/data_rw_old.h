
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


  static void WriteKMLFile(const std::string& fileName, const std::vector<std::string>& gps_list);
  static void WriteKMLFile(const std::string& fileName, const std::vector<std::vector<std::string> >& gps_list);
  static void WriteLogData(const std::string& logFolder, const std::string& logTitle, const std::string& header, const std::vector<std::string>& logData);
  static void CreateLoggingFolder();
};

class SimpleReaderBase
{
private:
  std::ifstream* m_pFile;
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
  SimpleReaderBase(const std::string& fileName, const int& nHeaders = 2, const char& separator = ',',
      const int& iDataTitles = 1, const int& nVariablesForOneObject = 0,
      const int& nLineHeaders = 0, const std::string& headerRepeatKey = "...");

  ~SimpleReaderBase();

protected:
  int ReadAllData();
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
  ~GPSDataReader(){}

  bool ReadNextLine(GPSBasicData& data);
  int ReadAllData(std::vector<GPSBasicData>& data_list);
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
  ~SimulationFileReader(){}

  bool ReadNextLine(SimulationPoint& data);
  int ReadAllData(SimulationData& data_list);
};

class LocalizationPathReader : public SimpleReaderBase
{
public:
  struct LocalizationWayPoint
  {
    double t;
    double x;
    double y;
    double z;
    double a;
    double v;
  };

  LocalizationPathReader(const std::string& fileName, const char& separator) : SimpleReaderBase(fileName, 1, separator){}
  ~LocalizationPathReader(){}

  bool ReadNextLine(LocalizationWayPoint& data);
  int ReadAllData(std::vector<LocalizationWayPoint>& data_list);
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

  AisanPointsFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }

  ~AisanPointsFileReader(){}

  bool ReadNextLine(AisanPoints& data);
  int ReadAllData(std::vector<AisanPoints>& data_list);
  AisanPoints* GetDataRowById(int _pid);
  std::vector<AisanPoints> m_data_list;

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

  AisanNodesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }

  ~AisanNodesFileReader(){}

  bool ReadNextLine(AisanNode& data);
  int ReadAllData(std::vector<AisanNode>& data_list);
  AisanNode* GetDataRowById(int _nid);
  std::vector<AisanNode> m_data_list;

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

  AisanLinesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }
  
  ~AisanLinesFileReader(){}

  bool ReadNextLine(AisanLine& data);
  int ReadAllData(std::vector<AisanLine>& data_list);
  
  AisanLine* GetDataRowById(int _lid);
  std::vector<AisanLine> m_data_list;

private:
  int m_min_id;
  std::vector<AisanLine*> m_data_map;
};

class AisanCenterLinesFileReader : public SimpleReaderBase
{
public:

  struct AisanCenterLine
  {
    int   DID;
    int   Dist;
    int   PID;
    double   Dir;
    double   Apara;
    double   r;
    double   slope;
    double   cant;
    double   LW;
    double   RW;
  };

  AisanCenterLinesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }
  
  ~AisanCenterLinesFileReader(){}

  bool ReadNextLine(AisanCenterLine& data);
  int ReadAllData(std::vector<AisanCenterLine>& data_list);
  
  AisanCenterLine* GetDataRowById(int _lnid);
  std::vector<AisanCenterLine> m_data_list;

private:
  int m_min_id;
  std::vector<AisanCenterLine*> m_data_map;
};

class AisanAreasFileReader : public SimpleReaderBase
{
public:

  struct AisanArea
  {
    int   AID;
    int   SLID;
    int   ELID;
  };

  AisanAreasFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }
  
  ~AisanAreasFileReader(){}

  bool ReadNextLine(AisanArea& data);
  int ReadAllData(std::vector<AisanArea>& data_list);
  
  AisanArea* GetDataRowById(int _lnid);
  std::vector<AisanArea> m_data_list;

private:
  int m_min_id;
  std::vector<AisanArea*> m_data_map;
};

class AisanIntersectionFileReader : public SimpleReaderBase
{
public:

  struct AisanIntersection
  {
    int   ID;
    int   AID;
    int   LinkID;
  };

  AisanIntersectionFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }
  
  ~AisanIntersectionFileReader(){}

  bool ReadNextLine(AisanIntersection& data);
  int ReadAllData(std::vector<AisanIntersection>& data_list);
  
  AisanIntersection* GetDataRowById(int _lnid);
  std::vector<AisanIntersection> m_data_list;

private:
  int m_min_id;
  std::vector<AisanIntersection*> m_data_map;
};

class AisanLanesFileReader : public SimpleReaderBase
{
public:

  struct AisanLane
  {
    int LnID  ;
    int DID    ;
    int BLID  ;
    int FLID  ;
    int BNID  ;
    int FNID  ;
    int JCT    ;
    int BLID2  ;
    int BLID3  ;
    int BLID4  ;
    int FLID2  ;
    int FLID3  ;
    int FLID4  ;
    int ClossID  ;
    double Span  ;
    int LCnt  ;
    int Lno    ;
    int LaneType;
    int LimitVel;
    int RefVel  ;
    int RoadSecID;
    int LaneChgFG;
    int LinkWAID;
    char LaneDir;
    int  LeftLaneId;
    int RightLaneId;

    int originalMapID;
  };

  AisanLanesFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }
  
  ~AisanLanesFileReader(){}

  bool ReadNextLine(AisanLane& data);
  int ReadAllData(std::vector<AisanLane>& data_list);
  
  AisanLane* GetDataRowById(int _lnid);
  std::vector<AisanLane> m_data_list;

private:
  int m_min_id;
  std::vector<AisanLane*> m_data_map;
};

class AisanStopLineFileReader : public SimpleReaderBase
{
public:

  struct AisanStopLine
  {
    int   ID;
    int   LID;
    int   TLID;
    int   SignID;
    int   LinkID;
  };

  AisanStopLineFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }
  
  ~AisanStopLineFileReader(){}

  bool ReadNextLine(AisanStopLine& data);
  int ReadAllData(std::vector<AisanStopLine>& data_list);
  
  AisanStopLine* GetDataRowById(int _lnid);
  std::vector<AisanStopLine> m_data_list;

private:
  int m_min_id;
  std::vector<AisanStopLine*> m_data_map;
};

class AisanRoadSignFileReader : public SimpleReaderBase
{
public:

  struct AisanRoadSign
  {
    int   ID;
    int   VID;
    int   PLID;
    int   Type;
    int   LinkID;
  };

  AisanRoadSignFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }
  
  ~AisanRoadSignFileReader(){}

  bool ReadNextLine(AisanRoadSign& data);
  int ReadAllData(std::vector<AisanRoadSign>& data_list);
  
  AisanRoadSign* GetDataRowById(int _lnid);
  std::vector<AisanRoadSign> m_data_list;

private:
  int m_min_id;
  std::vector<AisanRoadSign*> m_data_map;
};

class AisanSignalFileReader : public SimpleReaderBase
{
public:

  struct AisanSignal
  {
    int   ID;
    int   VID;
    int   PLID;
    int   Type;
    int   LinkID;
  };

  AisanSignalFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }
  
  ~AisanSignalFileReader(){}

  bool ReadNextLine(AisanSignal& data);
  int ReadAllData(std::vector<AisanSignal>& data_list);
  
  AisanSignal* GetDataRowById(int _lnid);
  std::vector<AisanSignal> m_data_list;

private:
  int m_min_id;
  std::vector<AisanSignal*> m_data_map;
};

class AisanVectorFileReader : public SimpleReaderBase
{
public:

  struct AisanVector
  {
    int   VID;
    int   PID;
    double   Hang;
    double   Vang;
  };

  AisanVectorFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }
  
  ~AisanVectorFileReader(){}

  bool ReadNextLine(AisanVector& data);
  int ReadAllData(std::vector<AisanVector>& data_list);
  
  AisanVector* GetDataRowById(int _lnid);
  std::vector<AisanVector> m_data_list;

private:
  int m_min_id;
  std::vector<AisanVector*> m_data_map;
};

class AisanCurbFileReader : public SimpleReaderBase
{
public:

  struct AisanCurb
  {
    int   ID;
    int   LID;
    double   Height;
    double   Width;
    int   dir;
    int   LinkID;
  };

  AisanCurbFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }
  
  ~AisanCurbFileReader(){}

  bool ReadNextLine(AisanCurb& data);
  int ReadAllData(std::vector<AisanCurb>& data_list);
  
  AisanCurb* GetDataRowById(int _lnid);
  std::vector<AisanCurb> m_data_list;

private:
  int m_min_id;
  std::vector<AisanCurb*> m_data_map;
};

class AisanRoadEdgeFileReader : public SimpleReaderBase
{
public:

  struct AisanRoadEdge
  {
    int   ID;
    int   LID;
    int   LinkID;
  };

  AisanRoadEdgeFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }
  
  ~AisanRoadEdgeFileReader(){}

  bool ReadNextLine(AisanRoadEdge& data);
  int ReadAllData(std::vector<AisanRoadEdge>& data_list);
  
  AisanRoadEdge* GetDataRowById(int _lnid);
  std::vector<AisanRoadEdge> m_data_list;

private:
  int m_min_id;
  std::vector<AisanRoadEdge*> m_data_map;
};

class AisanCrossWalkFileReader : public SimpleReaderBase
{
public:

  struct AisanCrossWalk
  {
    int   ID;
    int   AID;
    int   Type;
    int    BdID;
    int   LinkID;
  };

  AisanCrossWalkFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }
  
  ~AisanCrossWalkFileReader(){}

  bool ReadNextLine(AisanCrossWalk& data);
  int ReadAllData(std::vector<AisanCrossWalk>& data_list);
  
  AisanCrossWalk* GetDataRowById(int _lnid);
  std::vector<AisanCrossWalk> m_data_list;

private:
  int m_min_id;
  std::vector<AisanCrossWalk*> m_data_map;
};

class AisanWayareaFileReader : public SimpleReaderBase
{
public:

  struct AisanWayarea
  {
    int   ID;
    int   AID;
    int   LinkID;
  };

  AisanWayareaFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1)
  {
    m_min_id = std::numeric_limits<int>::max();
  }
  
  ~AisanWayareaFileReader(){}

  bool ReadNextLine(AisanWayarea& data);
  int ReadAllData(std::vector<AisanWayarea>& data_list);
  
  AisanWayarea* GetDataRowById(int _lnid);
  std::vector<AisanWayarea> m_data_list;

private:
  int m_min_id;
  std::vector<AisanWayarea*> m_data_map;
};

class AisanDataConnFileReader : public SimpleReaderBase
{
public:

  struct DataConn
  {
    int   LID; // lane id
    int   SLID; // stop line id
    int   SID; // signal id
    int   SSID; // stop sign id
  };

  AisanDataConnFileReader(const std::string& fileName) : SimpleReaderBase(fileName, 1){}
  ~AisanDataConnFileReader(){}

  bool ReadNextLine(DataConn& data);
  int ReadAllData(std::vector<DataConn>& data_list);
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

    Time::GetTickCount(_time_out);
  }

  ~MapRaw()
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
//    else
//    {
//      bLoaded =  pLanes != nullptr && pPoints != nullptr && pCenterLines  != nullptr;
//      if(bLoaded && bTimeOut)
//      {
//        iVersion = 1;
//        if(pNodes  == nullptr)
//          pNodes = new AisanNodesFileReader(vector_map_msgs::NodeArray());
//      }
//    }

    return iVersion;
  }
};

} /* namespace utilities */
} /* namespace op */

#endif /* DATARW_H_ */
