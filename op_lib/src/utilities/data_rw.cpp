
/// \file DataRW.cpp
/// \brief File operations for loading vector map files, loading kml map files and writing log .csv files
/// \author Hatem Darweesh
/// \date Jun 23, 2016

#include <utilities/data_rw.h>
#include <stdlib.h>
#include <tinyxml.h>
#include <sys/stat.h>

namespace op
{
  namespace utilities
  {

using namespace std;

std::string DataRW::LoggingMainfolderName = "/autoware_openplanner_logs/";
std::string DataRW::ControlLogFolderName = "ControlLogs/";
std::string DataRW::GlobalPathLogFolderName = "GlobalPathLogs/";
std::string DataRW::PathLogFolderName = "LocalTrajectoriesLogs/";
std::string DataRW::StatesLogFolderName = "BehaviorsLogs/";
std::string DataRW::SimulationFolderName = "SimulationData/";
std::string DataRW::KmlMapsFolderName = "KmlMaps/";
std::string DataRW::PredictionFolderName = "PredictionLogs/";
std::string DataRW::TrackingFolderName = "TrackingLogs/";
std::string DataRW::ExperimentsFolderName = "Experiments/";


DataRW::DataRW()
{
}

DataRW::~DataRW()
{
}

void DataRW::CreateLoggingMainFolder()
{
	std::string main_folder = System::GetHomeDirectory() + DataRW::LoggingMainfolderName;
	mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	
	CreateLoggingFolders(main_folder);

	main_folder = main_folder + DataRW::ExperimentsFolderName;
	mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}

void DataRW::CreateLoggingFolders(const std::string& mainFolderName)
{
	std::string main_folder = mainFolderName + DataRW::ControlLogFolderName;
	mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	main_folder = mainFolderName + DataRW::GlobalPathLogFolderName;
	mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	main_folder = mainFolderName + DataRW::PathLogFolderName;
	mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	main_folder = mainFolderName + DataRW::StatesLogFolderName;
	mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	main_folder = mainFolderName + DataRW::SimulationFolderName;
	mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	main_folder = mainFolderName + DataRW::PredictionFolderName;
	mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	main_folder = mainFolderName + DataRW::TrackingFolderName;
	mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
}

void DataRW::CreateExperimentFolder(const std::string& folderName)
{
	std::string main_folder = System::GetHomeDirectory() + DataRW::LoggingMainfolderName + DataRW::ExperimentsFolderName + folderName;
	mkdir(main_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	CreateLoggingFolders(main_folder);
}

void DataRW::WriteLogData(const std::string& logFolder, const std::string& logTitle, const std::string& header, const std::vector<std::string>& logData)
{
	if(logData.size() < 2)
		return;

	ostringstream fileName;
	fileName << logFolder;
	fileName << logTitle;
	fileName << Time::GetFilePrefixHourMinuteSeconds();
	fileName << ".csv";

	std::ofstream f(fileName.str().c_str());

	if(f.is_open())
	{
		if(header.size() > 0)
			f << header << "\r\n";
		for(unsigned int i = 0 ; i < logData.size(); i++)
			f << logData.at(i) << "\r\n";
	}

	f.close();
}

void DataRW::WriteKMLFile(const string& fileName, const vector<string>& gps_list)
{
	TiXmlDocument kmldoc(System::GetHomeDirectory()+DataRW::KmlMapsFolderName + "KmlTemplate.kml");

	assert(kmldoc.LoadFile() == true);

	TiXmlElement* pElem = kmldoc.FirstChildElement();

	if(!pElem)
	{
		printf("\n Empty KML File !");
		return;
	}

	TiXmlElement* pV=0;
	TiXmlHandle hKmlFile(pElem);

	//pV = hKmlFile.FirstChild("Folder").FirstChild("Folder").FirstChild("Document").FirstChild("Placemark").FirstChild("LineString").FirstChild("coordinates").Element();
	pV = hKmlFile.FirstChild("Folder").FirstChild("Document").FirstChild("Placemark").FirstChild("LineString").FirstChild("coordinates").Element();
	if(!pV)
		pV = hKmlFile.FirstChild( "Placemark" ).FirstChild("LineString").FirstChild("coordinates").Element();

	if(pV)
	{
			ostringstream val;
			val.precision(18);

			for(unsigned int i =0; i < gps_list.size(); i++)
			{
				val << gps_list[i] <<  " ";
			}

			TiXmlText * text = new TiXmlText( val.str() );
			pV->LinkEndChild(text);
	}

	kmldoc.SaveFile(fileName);
}

void DataRW::WriteKMLFile(const string& fileName, const vector<vector<string> >& gps_list)
  {
	  TiXmlDocument kmldoc(System::GetHomeDirectory()+DataRW::KmlMapsFolderName + "KmlTemplate.kml");

	  	assert(kmldoc.LoadFile() == true);

	  	TiXmlElement* pElem = kmldoc.FirstChildElement();

	  	if(!pElem)
	  	{
	  		printf("\n Empty KML File !");
	  		return;
	  	}

	  	TiXmlNode* pV=0;
	  	TiXmlNode* pPlaceMarkNode=0;
	  	TiXmlElement* pDocument=0;
	  	TiXmlHandle hKmlFile(pElem);

	  	//pV = hKmlFile.FirstChild("Folder").FirstChild("Folder").FirstChild("Document").FirstChild("Placemark").FirstChild("LineString").FirstChild("coordinates").Element();

	  	pDocument = hKmlFile.FirstChild("Folder").FirstChild("Document").Element();
	  	pPlaceMarkNode = hKmlFile.FirstChild("Folder").FirstChild("Document").FirstChild("Placemark").Node();

	  	if(!pDocument)
	  	{
	  		pDocument = hKmlFile.Element();
	  		pPlaceMarkNode = hKmlFile.FirstChild( "Placemark" ).Node();
	  	}



//	  	pV = hKmlFile.FirstChild("Folder").FirstChild("Document").FirstChild("Placemark").FirstChild("LineString").FirstChild("coordinates").Element();
//	  	if(!pV)
//	  		pV = hKmlFile.FirstChild( "Placemark" ).FirstChild("LineString").FirstChild("coordinates").Element();


	  	if(pDocument)
	  	{
	  		for(unsigned int l = 0; l < gps_list.size(); l++)
	  		{

	  			pV = pPlaceMarkNode->Clone();
	  			TiXmlElement* pElement = pV->FirstChild("LineString")->FirstChild("coordinates")->ToElement();

	  			ostringstream val;
				val.precision(18);

				for(unsigned int i =0; i < gps_list[l].size(); i++)
				{
					val << gps_list[l][i] <<  " ";
				}

				TiXmlText * text = new TiXmlText( val.str() );
				pElement->LinkEndChild(text);

				pDocument->InsertEndChild(*pV);

	  		}

	  	}

	  	kmldoc.SaveFile(fileName);
  }

void DataRW::writeCSVFile(const std::string& folder, const std::string& title, const std::string& header,
		const std::vector<std::string>& data_list)
{

	if(data_list.size() < 2)
			return;

	std::ostringstream file_name;
	file_name << folder;
	file_name << title;
	file_name << ".csv";

	std::ofstream f(file_name.str().c_str());

	if(f.is_open())
	{
		if(header.size() > 0)
			f << header << "\r\n";
		for(unsigned int i = 0 ; i < data_list.size(); i++)
			f << data_list.at(i) << "\r\n";
	}

	f.close();
}

SimpleReaderBase::SimpleReaderBase(const string& path, const int& nHeaders, const std::string& csv_file_name,
		const char& separator, const int& iDataTitles, const int& nVariablesForOneObject ,
		  const int& nLineHeaders, const string& headerRepeatKey)
{
	file_name_ = csv_file_name;

	if(path.empty()) return;
	if(path.compare("d") == 0) return;

	//check if fileName ends with '/' if yes append file_name_ , if not  , use it to open the file
	//then change the file_name_ for all classes to match file name .csv

	file_name_ = path;
	if(file_name_.at(file_name_.size()-1) == '/')
	{
		file_name_ += csv_file_name;
	}

	m_File = ifstream(file_name_.c_str(), ios::in);
	if(!m_File.is_open())
	{
		printf("\nCan't Open File!, %s \n", file_name_.c_str());
		return;
	}

	m_nHeders = nHeaders;
	m_iDataTitles = iDataTitles;
	m_nVarPerObj = nVariablesForOneObject;
	m_HeaderRepeatKey = headerRepeatKey;
	m_nLineHeaders = nLineHeaders;
	m_Separator = separator;
	m_File.precision(16);

	ReadHeaders();
}

SimpleReaderBase::~SimpleReaderBase()
{
	if(m_File.is_open())
		m_File.close();
}

bool SimpleReaderBase::ReadSingleLine(vector<vector<string> >& line)
{
	if(!m_File.is_open() || m_File.eof()) return false;

	string strLine, innerToken;
	line.clear();
	getline(m_File, strLine);
	istringstream str_stream(strLine);

	vector<string> header;
	vector<string> obj_part;

	if(m_nVarPerObj == 0)
	{
		while(getline(str_stream, innerToken, m_Separator))
		{
			obj_part.push_back(innerToken);
		}

		line.push_back(obj_part);
		return true;
	}
	else
	{
		int iCounter = 0;
		while(iCounter < m_nLineHeaders && getline(str_stream, innerToken, m_Separator))
		{
			header.push_back(innerToken);
			iCounter++;
		}
		obj_part.insert(obj_part.begin(), header.begin(), header.end());

		iCounter = 1;

		while(getline(str_stream, innerToken, m_Separator))
		{
			obj_part.push_back(innerToken);
			if(iCounter == m_nVarPerObj)
			{
				line.push_back(obj_part);
				obj_part.clear();

				iCounter = 0;
				obj_part.insert(obj_part.begin(), header.begin(), header.end());

			}
			iCounter++;
		}
	}

	return true;
}

//int SimpleReaderBase::ReadAllData()X
//{
//	if(!m_File.is_open()) return 0;
//
//	m_AllData.clear();
//	vector<vector<string> > singleLine;
//	while(!m_File.eof())
//	{
//		ReadSingleLine(singleLine);
//		m_AllData.push_back(singleLine);
//	}
//
//	return m_AllData.size();
//}

void SimpleReaderBase::ReadHeaders()
{
	if(!m_File.is_open()) return;

	string strLine;
	int iCounter = 0;
	m_RawHeaders.clear();
	while(!m_File.eof() && iCounter < m_nHeders)
	{
		getline(m_File, strLine);
		m_RawHeaders.push_back(strLine);
		if(iCounter == m_iDataTitles)
			ParseDataTitles(strLine);
		iCounter++;
	}
}

void SimpleReaderBase::ParseDataTitles(const string& header)
{
	if(header.size()==0) return;

	string innerToken;
	istringstream str_stream(header);
	m_DataTitlesHeader.clear();
	while(getline(str_stream, innerToken, m_Separator))
	{
		if(innerToken.compare(m_HeaderRepeatKey)!=0)
			m_DataTitlesHeader.push_back(innerToken);
	}
}

bool GPSDataReader::ReadNextLine(GPSBasicData& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 5) return false;

		data.lat = strtod(lineData.at(0).at(2).c_str(), NULL);
		data.lon = strtod(lineData.at(0).at(3).c_str(), NULL);
		data.alt = strtod(lineData.at(0).at(4).c_str(), NULL);
		data.distance = strtod(lineData.at(0).at(5).c_str(), NULL);

		return true;

	}
	else
		return false;
}

int GPSDataReader::ReadAllData()
{
	return 0;
}

int GPSDataReader::ReadAllData(vector<GPSBasicData>& data_list)
{
	if(!m_File.is_open()) return 0;

	data_list.clear();
	GPSBasicData data;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

bool SimulationFileReader::ReadNextLine(SimulationPoint& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 6) return false;

		data.x = strtod(lineData.at(0).at(0).c_str(), NULL);
		data.y = strtod(lineData.at(0).at(1).c_str(), NULL);
		data.z = strtod(lineData.at(0).at(2).c_str(), NULL);
		data.a = strtod(lineData.at(0).at(3).c_str(), NULL);
		data.c = strtod(lineData.at(0).at(4).c_str(), NULL);
		data.v = strtod(lineData.at(0).at(5).c_str(), NULL);
		data.name = lineData.at(0).at(6);

		return true;

	}
	else
		return false;
}

int SimulationFileReader::ReadAllData()
{
	return 0;
}

int SimulationFileReader::ReadAllData(SimulationData& data_list)
{
	if(!m_File.is_open()) return 0;
	data_list.simuCars.clear();
	SimulationPoint data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		if(count == 0)
			data_list.startPoint = data;
		else if(count == 1)
			data_list.goalPoint = data;
		else
			data_list.simuCars.push_back(data);

		count++;
	}

	return count;
}

bool LocalizationPathReader::ReadNextLine(LocalizationWayPoint& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 3) return false;

		//data.t = strtod(lineData.at(0).at(0).c_str(), NULL);
		data.x = strtod(lineData.at(0).at(0).c_str(), NULL);
		data.y = strtod(lineData.at(0).at(1).c_str(), NULL);
		data.z = strtod(lineData.at(0).at(2).c_str(), NULL);

		if(lineData.at(0).size() > 3)
			data.a = strtod(lineData.at(0).at(3).c_str(), NULL);
		if(lineData.at(0).size() > 4)
			data.v = strtod(lineData.at(0).at(4).c_str(), NULL);

		return true;

	}
	else
		return false;
}

int LocalizationPathReader::ReadAllData()
{
	return 0;
}

int LocalizationPathReader::ReadAllData(vector<LocalizationWayPoint>& data_list)
{
	if(!m_File.is_open()) return 0;
	data_list.clear();
	LocalizationWayPoint data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

//TimeStampedPath data

bool TimeStampedPathReader::ReadNextLine(TimePoint& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 3) return false;

		data.t = strtod(lineData.at(0).at(0).c_str(), NULL);
		data.x = strtod(lineData.at(0).at(1).c_str(), NULL);
		data.y = strtod(lineData.at(0).at(2).c_str(), NULL);
		data.z = strtod(lineData.at(0).at(3).c_str(), NULL);

		if(lineData.at(0).size() > 3)
		{
			data.a = strtod(lineData.at(0).at(3).c_str(), NULL);
		}

		return true;

	}
	else
		return false;
}

int TimeStampedPathReader::ReadAllData()
{
	return 0;
}

int TimeStampedPathReader::ReadAllData(vector<TimePoint>& data_list)
{
	if(!m_File.is_open()) return 0;
	data_list.clear();
	TimePoint data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

//NCKU Dataset Trajectory Reader

bool DatasetTrajectoryReader::ReadNextLine(TimePoint& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 6) return false;

		data.t = strtod(lineData.at(0).at(0).c_str(), NULL);
		data.x = strtod(lineData.at(0).at(1).c_str(), NULL);
		data.y = strtod(lineData.at(0).at(2).c_str(), NULL);
		data.roll = strtod(lineData.at(0).at(3).c_str(), NULL);
		data.pitch = strtod(lineData.at(0).at(4).c_str(), NULL);
		data.yaw = strtod(lineData.at(0).at(5).c_str(), NULL);
		data.z = strtod(lineData.at(0).at(6).c_str(), NULL);

		return true;

	}
	else
		return false;
}

int DatasetTrajectoryReader::ReadAllData()
{
	return 0;
}

int DatasetTrajectoryReader::ReadAllData(vector<TimePoint>& data_list)
{
	if(!m_File.is_open()) return 0;
	data_list.clear();
	TimePoint data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}


// Destinations data

DestinationsDataFileReader::DestinationsDataFileReader(const DestinationData& proj_data) : SimpleReaderBase("d", 1)
{
	header_ = "ID,Name,Longitude,latitude,Altitude,X,Y,Z,Angle,Hour,Minute";
	m_data_list.clear();
	m_data_list.push_back(proj_data);
}

bool DestinationsDataFileReader::ReadNextLine(DestinationData& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 11) return false;

		data.id = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.name = lineData.at(0).at(1);
		data.lon = strtod(lineData.at(0).at(2).c_str(), NULL);
		data.lat = strtod(lineData.at(0).at(3).c_str(), NULL);
		data.alt = strtod(lineData.at(0).at(4).c_str(), NULL);
		data.x = strtod(lineData.at(0).at(5).c_str(), NULL);
		data.y = strtod(lineData.at(0).at(6).c_str(), NULL);
		data.z = strtod(lineData.at(0).at(7).c_str(), NULL);
		data.angle = strtod(lineData.at(0).at(8).c_str(), NULL);
		data.hour = strtol(lineData.at(0).at(9).c_str(), NULL, 10);
		data.minute = strtol(lineData.at(0).at(10).c_str(), NULL, 10);

		return true;
	}
	else
		return false;
}

int DestinationsDataFileReader::ReadAllData(vector<DestinationData>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int DestinationsDataFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;

	m_data_list.clear();
	DestinationData data;
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
	}
	return m_data_list.size();
}

// Projection data

ProjectionDataFileReader::ProjectionDataFileReader(const ProjectionData& proj_data) : SimpleReaderBase("d", 1)
{
	header_ = "ProjType,ProjString,Longitude,Latitude,Altitude,X,Y,Z";
	m_data_list.clear();
	m_data_list.push_back(proj_data);
}

bool ProjectionDataFileReader::ReadNextLine(ProjectionData& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 8) return false;

		data.proj_type = lineData.at(0).at(0);
		data.proj_str = lineData.at(0).at(1);
		data.lon = strtod(lineData.at(0).at(2).c_str(), NULL);
		data.lat = strtod(lineData.at(0).at(3).c_str(), NULL);
		data.alt = strtod(lineData.at(0).at(4).c_str(), NULL);
		data.x = strtod(lineData.at(0).at(5).c_str(), NULL);
		data.y = strtod(lineData.at(0).at(6).c_str(), NULL);
		data.z = strtod(lineData.at(0).at(7).c_str(), NULL);
		return true;
	}
	else
		return false;
}

int ProjectionDataFileReader::ReadAllData(vector<ProjectionData>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int ProjectionDataFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;

	m_data_list.clear();
	ProjectionData data;
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
	}
	return m_data_list.size();
}

//Nodes

AisanNodesFileReader::AisanNode* AisanNodesFileReader::GetDataRowById(int _nid)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _nid-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).NID == _nid)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanNodesFileReader::ReadNextLine(AisanNode& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 2) return false;

		data.NID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.PID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanNodesFileReader::ReadAllData(vector<AisanNode>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanNodesFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanNode data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.NID < m_min_id)
			m_min_id = data.NID;

		if(data.NID > max_id)
			max_id = data.NID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).NID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Points

AisanPointsFileReader::AisanPoints* AisanPointsFileReader::GetDataRowById(int _pid)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _pid-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).PID == _pid)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanPointsFileReader::ReadNextLine(AisanPoints& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 10) return false;

		data.PID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.B = strtod(lineData.at(0)[1].c_str(), NULL);
		data.L = strtod(lineData.at(0)[2].c_str(), NULL);
		data.H = strtod(lineData.at(0)[3].c_str(), NULL);

		data.Bx = strtod(lineData.at(0)[4].c_str(), NULL);
		data.Ly = strtod(lineData.at(0)[5].c_str(), NULL);
		data.Ref = strtol(lineData.at(0).at(6).c_str(), NULL, 10);
		data.MCODE1 = strtol(lineData.at(0).at(7).c_str(), NULL, 10);
		data.MCODE2 = strtol(lineData.at(0).at(8).c_str(), NULL, 10);
		data.MCODE3 = strtol(lineData.at(0).at(9).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanPointsFileReader::ReadAllData(vector<AisanPoints>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanPointsFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanPoints data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.PID < m_min_id)
			m_min_id = data.PID;

		if(data.PID > max_id)
			max_id = data.PID;
	}

	m_data_map.resize(max_id-m_min_id+2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).PID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

// Lines

AisanLinesFileReader::AisanLine* AisanLinesFileReader::GetDataRowById(int _lid)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _lid-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).LID == _lid)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanLinesFileReader::ReadNextLine(AisanLine& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 5) return false;

		data.LID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.BPID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.FPID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.BLID = strtol(lineData.at(0).at(3).c_str(), NULL, 10);
		data.FLID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

		return true;
	}
	else
		return false;
}

int AisanLinesFileReader::ReadAllData(vector<AisanLine>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanLinesFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanLine data;
	//double logTime = 0;

	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.LID < m_min_id)
			m_min_id = data.LID;

		if(data.LID > max_id)
			max_id = data.LID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).LID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

// Whitelines

AisanWhitelinesFileReader::AisanWhiteline* AisanWhitelinesFileReader::GetDataRowById(int _wlid)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _wlid-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _wlid)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanWhitelinesFileReader::ReadNextLine(AisanWhiteline& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 6) return false;

		data.ID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.LID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.Width = strtod(lineData.at(0).at(2).c_str(), NULL);
		data.Color = lineData.at(0).at(3);
		data.type = strtol(lineData.at(0).at(4).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(5).c_str(), NULL, 10);

		return true;
	}
	else
		return false;
}

int AisanWhitelinesFileReader::ReadAllData(vector<AisanWhiteline>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanWhitelinesFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanWhiteline data;

	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.ID < m_min_id)
			m_min_id = data.ID;

		if(data.ID > max_id)
			max_id = data.ID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).ID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//dt Lanes (center lines)

AisanCenterLinesFileReader::AisanCenterLine* AisanCenterLinesFileReader::GetDataRowById(int _did)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _did-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).DID == _did)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanCenterLinesFileReader::ReadNextLine(AisanCenterLine& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 10) return false;

		data.DID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.Dist 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.PID 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);

		data.Dir 	= strtod(lineData.at(0)[3].c_str(), NULL);
		data.Apara 	= strtod(lineData.at(0)[4].c_str(), NULL);
		data.r 		= strtod(lineData.at(0)[5].c_str(), NULL);
		data.slope 	= strtod(lineData.at(0)[6].c_str(), NULL);
		data.cant 	= strtod(lineData.at(0)[7].c_str(), NULL);
		data.LW 	= strtod(lineData.at(0)[8].c_str(), NULL);
		data.RW 	= strtod(lineData.at(0)[9].c_str(), NULL);

		return true;
	}
	else
		return false;
}

int AisanCenterLinesFileReader::ReadAllData(vector<AisanCenterLine>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanCenterLinesFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanCenterLine data;
	int max_id = std::numeric_limits<int>::min();

	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.DID < m_min_id)
			m_min_id = data.DID;

		if(data.DID > max_id)
			max_id = data.DID;
	}

	m_data_map.resize(max_id-m_min_id+2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).DID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Lane

AisanLanesFileReader::AisanLane* AisanLanesFileReader::GetDataRowById(int _lnid)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _lnid-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).LnID == _lnid)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanLanesFileReader::ReadNextLine(AisanLane& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size() == 0) return false;
		if(lineData.at(0).size() < 17) return false;

		data.LnID		= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.DID		= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.BLID		= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.FLID		= strtol(lineData.at(0).at(3).c_str(), NULL, 10);
		data.BNID	 	= strtol(lineData.at(0).at(4).c_str(), NULL, 10);
		data.FNID		= strtol(lineData.at(0).at(5).c_str(), NULL, 10);
		data.JCT		= strtol(lineData.at(0).at(6).c_str(), NULL, 10);
		data.BLID2	 	= strtol(lineData.at(0).at(7).c_str(), NULL, 10);
		data.BLID3		= strtol(lineData.at(0).at(8).c_str(), NULL, 10);
		data.BLID4		= strtol(lineData.at(0).at(9).c_str(), NULL, 10);
		data.FLID2	 	= strtol(lineData.at(0).at(10).c_str(), NULL, 10);
		data.FLID3		= strtol(lineData.at(0).at(11).c_str(), NULL, 10);
		data.FLID4		= strtol(lineData.at(0).at(12).c_str(), NULL, 10);
		data.ClossID 	= strtol(lineData.at(0).at(13).c_str(), NULL, 10);
		data.Span 		= strtod(lineData.at(0).at(14).c_str(), NULL);
		data.LCnt	 	= strtol(lineData.at(0).at(15).c_str(), NULL, 10);
		data.Lno	  	= strtol(lineData.at(0).at(16).c_str(), NULL, 10);


		if(lineData.at(0).size() > 17)
			data.LaneType	= strtol(lineData.at(0).at(17).c_str(), NULL, 10);
		if(lineData.at(0).size() > 18)
			data.LimitVel	= strtol(lineData.at(0).at(18).c_str(), NULL, 10);
		if(lineData.at(0).size() > 19)
			data.RefVel	 	= strtol(lineData.at(0).at(19).c_str(), NULL, 10);
		if(lineData.at(0).size() > 20)
			data.RoadSecID	= strtol(lineData.at(0).at(20).c_str(), NULL, 10);
		if(lineData.at(0).size() > 21)
			data.LaneChgFG 	= strtol(lineData.at(0).at(21).c_str(), NULL, 10);
		if(lineData.at(0).size() > 22)
			data.LinkWAID	= strtol(lineData.at(0).at(22).c_str(), NULL, 10);

		if(lineData.at(0).size() > 23)
		{
			string str_dir = lineData.at(0).at(23);
			if(str_dir.size() > 0)
				data.LaneDir 	= str_dir.at(0);
			else
				data.LaneDir  	= 'F';
		}

//		data.LeftLaneId  = 0;
//		data.RightLaneId = 0;
//		data.LeftLaneId 	= strtol(lineData.at(0).at(24).c_str(), NULL, 10);
//		data.RightLaneId 	= strtol(lineData.at(0).at(25).c_str(), NULL, 10);


		return true;
	}
	else
		return false;
}

int AisanLanesFileReader::ReadAllData(vector<AisanLane>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanLanesFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanLane data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();

	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.LnID < m_min_id)
			m_min_id = data.LnID;

		if(data.LnID > max_id)
			max_id = data.LnID;
	}

	m_data_map.resize(max_id-m_min_id+2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).LnID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Area

AisanAreasFileReader::AisanArea* AisanAreasFileReader::GetDataRowById(int _aid)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _aid-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).AID == _aid)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanAreasFileReader::ReadNextLine(AisanArea& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 3) return false;

		data.AID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.SLID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.ELID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanAreasFileReader::ReadAllData(vector<AisanArea>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanAreasFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanArea data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.AID < m_min_id)
			m_min_id = data.AID;

		if(data.AID > max_id)
			max_id = data.AID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).AID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Intersection

AisanIntersectionFileReader::AisanIntersection* AisanIntersectionFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanIntersectionFileReader::ReadNextLine(AisanIntersection& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 3) return false;

		data.ID = strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.AID = strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanIntersectionFileReader::ReadAllData(vector<AisanIntersection>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanIntersectionFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanIntersection data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.ID < m_min_id)
			m_min_id = data.ID;

		if(data.ID > max_id)
			max_id = data.ID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).ID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//StopLine

AisanStopLineFileReader::AisanStopLine* AisanStopLineFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanStopLineFileReader::ReadNextLine(AisanStopLine& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 5) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.LID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.TLID 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.SignID = strtol(lineData.at(0).at(3).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanStopLineFileReader::ReadAllData(vector<AisanStopLine>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanStopLineFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanStopLine data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.ID < m_min_id)
			m_min_id = data.ID;

		if(data.ID > max_id)
			max_id = data.ID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).ID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//RoadSign

AisanRoadSignFileReader::AisanRoadSign* AisanRoadSignFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanRoadSignFileReader::ReadNextLine(AisanRoadSign& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 5) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.VID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.PLID 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.Type 	= strtol(lineData.at(0).at(3).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanRoadSignFileReader::ReadAllData(vector<AisanRoadSign>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanRoadSignFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanRoadSign data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.ID < m_min_id)
			m_min_id = data.ID;

		if(data.ID > max_id)
			max_id = data.ID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).ID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Signal

AisanSignalFileReader::AisanSignal* AisanSignalFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanSignalFileReader::ReadNextLine(AisanSignal& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 5) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.VID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.PLID 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.Type 	= strtol(lineData.at(0).at(3).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanSignalFileReader::ReadAllData(vector<AisanSignal>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanSignalFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanSignal data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.ID < m_min_id)
			m_min_id = data.ID;

		if(data.ID > max_id)
			max_id = data.ID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).ID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Vector

AisanVectorFileReader::AisanVector* AisanVectorFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).VID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanVectorFileReader::ReadNextLine(AisanVector& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 4) return false;

		data.VID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.PID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.Hang 	= strtod(lineData.at(0).at(2).c_str(), NULL);
		data.Vang 	= strtod(lineData.at(0).at(3).c_str(), NULL);

		return true;

	}
	else
		return false;
}

int AisanVectorFileReader::ReadAllData(vector<AisanVector>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanVectorFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanVector data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.VID < m_min_id)
			m_min_id = data.VID;

		if(data.VID > max_id)
			max_id = data.VID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).VID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Curb

AisanCurbFileReader::AisanCurb* AisanCurbFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanCurbFileReader::ReadNextLine(AisanCurb& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 6) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.LID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.Height = strtod(lineData.at(0).at(2).c_str(), NULL);
		data.Width 	= strtod(lineData.at(0).at(3).c_str(), NULL);
		data.dir 	= strtol(lineData.at(0).at(4).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(5).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanCurbFileReader::ReadAllData(vector<AisanCurb>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanCurbFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanCurb data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.ID < m_min_id)
			m_min_id = data.ID;

		if(data.ID > max_id)
			max_id = data.ID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).ID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//CrossWalk

AisanCrossWalkFileReader::AisanCrossWalk* AisanCrossWalkFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanCrossWalkFileReader::ReadNextLine(AisanCrossWalk& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 5) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.AID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.Type 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.BdID 	= strtol(lineData.at(0).at(3).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(4).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanCrossWalkFileReader::ReadAllData(vector<AisanCrossWalk>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanCrossWalkFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanCrossWalk data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.ID < m_min_id)
			m_min_id = data.ID;

		if(data.ID > max_id)
			max_id = data.ID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).ID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//WayArea

AisanWayareaFileReader::AisanWayarea* AisanWayareaFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanWayareaFileReader::ReadNextLine(AisanWayarea& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 3) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.AID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanWayareaFileReader::ReadAllData(vector<AisanWayarea>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanWayareaFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanWayarea data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.ID < m_min_id)
			m_min_id = data.ID;

		if(data.ID > max_id)
			max_id = data.ID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).ID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Gutter

AisanGutterFileReader::AisanGutter* AisanGutterFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanGutterFileReader::ReadNextLine(AisanGutter& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 4) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.AID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.Type 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(3).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanGutterFileReader::ReadAllData(vector<AisanGutter>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanGutterFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanGutter data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.ID < m_min_id)
			m_min_id = data.ID;

		if(data.ID > max_id)
			max_id = data.ID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).ID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Idx

AisanIdxFileReader::AisanIdx* AisanIdxFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanIdxFileReader::ReadNextLine(AisanIdx& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 3) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.KIND 	= lineData.at(0).at(1).c_str();
		data.fname 	= lineData.at(0).at(2).c_str();

		return true;

	}
	else
		return false;
}

int AisanIdxFileReader::ReadAllData(vector<AisanIdx>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanIdxFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanIdx data;
	//double logTime = 0;
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
	}
	return m_data_list.size();
}

//Pole

AisanPoleFileReader::AisanPole* AisanPoleFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).PLID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanPoleFileReader::ReadNextLine(AisanPole& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 4) return false;

		data.PLID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.VID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.Length = strtod(lineData.at(0).at(2).c_str(), NULL);
		data.Dim 	= strtod(lineData.at(0).at(3).c_str(), NULL);

		return true;

	}
	else
		return false;
}

int AisanPoleFileReader::ReadAllData(vector<AisanPole>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanPoleFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanPole data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.PLID < m_min_id)
			m_min_id = data.PLID;

		if(data.PLID > max_id)
			max_id = data.PLID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).PLID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Poledata

AisanPoledataFileReader::AisanPoledata* AisanPoledataFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).PLID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanPoledataFileReader::ReadNextLine(AisanPoledata& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 3) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.PLID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		return true;

	}
	else
		return false;
}

int AisanPoledataFileReader::ReadAllData(vector<AisanPoledata>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanPoledataFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanPoledata data;
	//double logTime = 0;
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
	}
	return m_data_list.size();
}

//Roadedge

AisanRoadEdgeFileReader::AisanRoadEdge* AisanRoadEdgeFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanRoadEdgeFileReader::ReadNextLine(AisanRoadEdge& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 3) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.LID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanRoadEdgeFileReader::ReadAllData(vector<AisanRoadEdge>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanRoadEdgeFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanRoadEdge data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.ID < m_min_id)
			m_min_id = data.ID;

		if(data.ID > max_id)
			max_id = data.ID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).ID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Surfacemark

AisanSurfacemarkFileReader::AisanSurfacemark* AisanSurfacemarkFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanSurfacemarkFileReader::ReadNextLine(AisanSurfacemark& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 4) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.AID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.Type 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(3).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanSurfacemarkFileReader::ReadAllData(vector<AisanSurfacemark>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanSurfacemarkFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanSurfacemark data;
	//double logTime = 0;
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
	}
	return m_data_list.size();
}

//Streetlight

AisanStreetlightFileReader::AisanStreetlight* AisanStreetlightFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanStreetlightFileReader::ReadNextLine(AisanStreetlight& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 4) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.LID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.PLID 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(3).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanStreetlightFileReader::ReadAllData(vector<AisanStreetlight>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanStreetlightFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanStreetlight data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.ID < m_min_id)
			m_min_id = data.ID;

		if(data.ID > max_id)
			max_id = data.ID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).ID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Utilitypole

AisanUtilitypoleFileReader::AisanUtilitypole* AisanUtilitypoleFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanUtilitypoleFileReader::ReadNextLine(AisanUtilitypole& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 3) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.PLID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanUtilitypoleFileReader::ReadAllData(vector<AisanUtilitypole>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanUtilitypoleFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanUtilitypole data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.ID < m_min_id)
			m_min_id = data.ID;

		if(data.ID > max_id)
			max_id = data.ID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).ID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Zebrazone

AisanZebrazoneFileReader::AisanZebrazone* AisanZebrazoneFileReader::GetDataRowById(int _id)
{
	if(m_data_map.size()==0) return nullptr;

	int index = _id-m_min_id;
	if(index >= 0 && index < (int)m_data_map.size())
	{
		return m_data_map.at(index);
	}
	else
	{
		for(unsigned int i=0; i < m_data_list.size(); i++)
		{
			if(m_data_list.at(i).ID == _id)
			{
				return &m_data_list.at(i);
			}
		}
	}

	return nullptr;
}

bool AisanZebrazoneFileReader::ReadNextLine(AisanZebrazone& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 3) return false;

		data.ID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.AID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.LinkID = strtol(lineData.at(0).at(2).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanZebrazoneFileReader::ReadAllData(vector<AisanZebrazone>& data_list)
{
	ReadAllData();
	data_list = m_data_list;
	return m_data_list.size();
}

int AisanZebrazoneFileReader::ReadAllData()
{
	if(!m_File.is_open()) return 0;
	m_data_list.clear();
	AisanZebrazone data;
	//double logTime = 0;
	int max_id = std::numeric_limits<int>::min();
	while(ReadNextLine(data))
	{
		m_data_list.push_back(data);
		if(data.ID < m_min_id)
			m_min_id = data.ID;

		if(data.ID > max_id)
			max_id = data.ID;
	}

	m_data_map.resize(max_id - m_min_id + 2);
	for(unsigned int i=0; i < m_data_list.size(); i++)
	{
		m_data_map.at(m_data_list.at(i).ID-m_min_id) = &m_data_list.at(i);
	}
	return m_data_list.size();
}

//Data Conn
bool AisanDataConnFileReader::ReadNextLine(DataConn& data)
{
	vector<vector<string> > lineData;
	if(ReadSingleLine(lineData))
	{
		if(lineData.size()==0) return false;
		if(lineData.at(0).size() < 4) return false;

		data.LID 	= strtol(lineData.at(0).at(0).c_str(), NULL, 10);
		data.SLID 	= strtol(lineData.at(0).at(1).c_str(), NULL, 10);
		data.SID 	= strtol(lineData.at(0).at(2).c_str(), NULL, 10);
		data.SSID 	= strtol(lineData.at(0).at(3).c_str(), NULL, 10);

		return true;

	}
	else
		return false;
}

int AisanDataConnFileReader::ReadAllData()
{
	return 0;
}

int AisanDataConnFileReader::ReadAllData(vector<DataConn>& data_list)
{
	if(!m_File.is_open()) return 0;
	data_list.clear();
	DataConn data;
	//double logTime = 0;
	int count = 0;
	while(ReadNextLine(data))
	{
		data_list.push_back(data);
		count++;
	}
	return count;
}

} /* namespace utilities */
} /* namespace op */
