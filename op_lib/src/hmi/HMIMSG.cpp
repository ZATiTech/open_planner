/*
 * HMIMSG.cpp
 *
 *  Created on: Apr 18, 2020
 *      Author: hatem
 */

#include "hmi/HMIMSG.h"
#include <sstream>

namespace op
{
	namespace hmi
	{

HMI_MSG::HMI_MSG()
{
	type = UNKNOWN_MSG;
	msg_id = -1;
	current_action = MSG_FORWARD_ACTION;
	next_destination_id = -1;
	curr_destination_id = -1;
	bErr = false;
}

HMI_MSG::~HMI_MSG()
{
}

HMI_MSG HMI_MSG::FromString(const std::string& msg)
{
	//command,msg_id,action1;action2;action3;,currAction,next_destination,curr_destination,dest1|name1|departure1;dest2|name2|departure2;,has_error,err_msg,
	HMI_MSG recieved_msg;
	std::vector<std::string> sections = SplitString(msg, ",");
	if (sections.size() > 0)
	{
		int type_item = strtol(sections.at(0).c_str(), NULL, 10);
		switch (type_item)
		{
		case 0:
			recieved_msg.type = COMMAND_MSG;
			break;
		case 1:
			recieved_msg.type = CONFIRM_MSG;
			break;
		case 2:
			recieved_msg.type = OPTIONS_MSG;
			break;
		case 3:
			recieved_msg.type = DESTINATIONS_MSG;
			break;
		case 4:
			recieved_msg.type = ERROR_MSG;
			break;
		default:
			recieved_msg.type = UNKNOWN_MSG;
			break;
		}
	}

	if (sections.size() > 1)
	{
		recieved_msg.msg_id = strtol(sections.at(1).c_str(), NULL, 10);
	}

	if (sections.size() > 2)
	{
		std::vector<std::string> actions_str = SplitString(sections.at(2), ";");
		for (auto& x : actions_str)
		{
			int actid = strtol(x.c_str(), NULL, 10);
			recieved_msg.available_actions.push_back(GetActionType(actid));
		}
	}

	if (sections.size() > 3)
	{
		int action_item = strtol(sections.at(3).c_str(), NULL, 10);
		recieved_msg.current_action = GetActionType(action_item);
	}

	if (sections.size() > 4)
	{
		recieved_msg.next_destination_id = strtol(sections.at(4).c_str(), NULL, 10);
	}

	if (sections.size() > 5)
	{
		recieved_msg.curr_destination_id = strtol(sections.at(5).c_str(), NULL, 10);
	}

	if (sections.size() > 6)
	{
		std::vector<std::string> destinations_strs = SplitString(sections.at(6), ";");
		for(auto& dest: destinations_strs)
		{
			DESTINATION d;
			std::vector<std::string> items_strs = SplitString(dest, "|");
			if(items_strs.size() >= 4)
			{
				d.id = strtol(items_strs.at(0).c_str(), NULL, 10);
				d.name = items_strs.at(1);
				d.hour = strtol(items_strs.at(2).c_str(), NULL, 10);
				d.minute = strtol(items_strs.at(3).c_str(), NULL, 10);
			}
			recieved_msg.destinations.push_back(d);
		}
	}

	if (sections.size() > 7)
	{
		recieved_msg.bErr = strtol(sections.at(7).c_str(), NULL, 10);
	}

	if (sections.size() > 8)
	{
		recieved_msg.err_msg = sections.at(8);
	}

	return recieved_msg;
}

std::string HMI_MSG::CreateStringMessage()
{
	//command,msg_id,action1;action2;action3;,currAction,next_destination,curr_destination,dest1|name1|hour1|minute1|;dest2|name2|hour2|minut2|;,has_error,err_msg,
	std::ostringstream oss;
	oss << this->type << ",";
	oss << this->msg_id << ",";

	for(auto& x: this->available_actions)
	{
		oss << x << ";";
	}

	oss << "," << this->current_action;
	oss << "," << this->next_destination_id;
	oss << "," << this->curr_destination_id;

	oss << ",";
	for(auto& x: this->destinations)
	{
		oss << x.id << "|";
		oss << x.name << "|";
		oss << x.hour << "|";
		oss << x.minute << "|";
		oss << ";";
	}
	oss << ",";

	oss << this->bErr << ",";
	oss << this->err_msg << ",";
	oss << ",";

	  return oss.str();
}

std::vector<std::string> HMI_MSG::SplitString(const std::string& str, const std::string& token)
{
	std::vector<std::string> str_parts;
	int iFirstPart = 0;
	int iSecondPart = str.find(token, iFirstPart);

	while (iSecondPart > 0 && iSecondPart < (int)str.size())
	{
		str_parts.push_back(str.substr(iFirstPart, iSecondPart - iFirstPart));
		iFirstPart = iSecondPart+1;
		iSecondPart = str.find(token, iFirstPart);
	}

	return str_parts;
}

MSG_ACTION HMI_MSG::GetActionType(int id)
{
	MSG_ACTION action_type = MSG_FORWARD_ACTION;
	switch(id)
	{
	case 0:
		action_type = MSG_FORWARD_ACTION;
		break;
	case 1:
		action_type = MSG_BACKWARD_ACTION;
		break;
	case 2:
		action_type = MSG_STOP_ACTION;
		break;
	case 3:
		action_type = MSG_LEFT_TURN_ACTION;
		break;
	case 4:
		action_type = MSG_RIGHT_TURN_ACTION;
		break;
	case 5:
		action_type = MSG_U_TURN_ACTION;
		break;
	case 6:
		action_type = MSG_SWERVE_ACTION;
		break;
	case 7:
		action_type = MSG_OVERTACK_ACTION;
		break;
	case 8:
		action_type = MSG_START_ACTION;
		break;
	case 9:
		action_type = MSG_SLOWDOWN_ACTION;
		break;
	case 10:
		action_type = MSG_CHANGE_DESTINATION;
		break;
	case 11:
		action_type = MSG_WAITING_ACTION;
		break;
	case 12:
		action_type = MSG_DESTINATION_REACHED;
		break;
	}

	return action_type;
}

} /* namespace hmi */
} /* namespace op */
