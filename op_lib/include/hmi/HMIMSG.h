
/// \file HMIMSG.h
/// \brief HMI message structure. used to serialize.deserialize information to be send to and received from remote HMI application.
/// 		Commands, information and error messages to and from Global Planning state machine.
/// \author Hatem Darweesh
/// \date April 18, 2020

#ifndef HMIMSG_H_
#define HMIMSG_H_

#include <string>
#include <vector>
#include <stdlib.h>
#include <ctime>
#include <iostream>

namespace op
{
	namespace hmi
	{
enum MSG_TYPE{
	COMMAND_MSG = 0,
	CONFIRM_MSG = 1,
	OPTIONS_MSG = 2, // This indicate what options are available for the HMI clinet to be send as command, also used for passing information such as current and next destination id.
	DESTINATIONS_MSG = 3, // Send destinations list
	ERROR_MSG = 4,
	UNKNOWN_MSG = 5};

enum MSG_ACTION{
	MSG_FORWARD_ACTION = 0, //Continue as normal
	MSG_BACKWARD_ACTION = 1,
	MSG_STOP_ACTION = 2, //unscheduled stop
	MSG_LEFT_TURN_ACTION = 3,
	MSG_RIGHT_TURN_ACTION = 4,
	MSG_U_TURN_ACTION = 5,
	MSG_SWERVE_ACTION = 6, //Enable Swerve Action
	MSG_OVERTACK_ACTION = 7, //Enable Overtake Action, this action include changing lane
	MSG_START_ACTION = 8, //Route Start
	MSG_SLOWDOWN_ACTION = 9, //Manual slow down
	MSG_CHANGE_DESTINATION = 10, //Cancel Route, Skip next bus stop, depend on the rout ID
	MSG_WAITING_ACTION = 11,
	MSG_DESTINATION_REACHED=12}; //End of Route

struct DESTINATION
{
	int id;
	std::string name;
	int hour; // 0-24
	int minute; // 0-59
};

class HMI_MSG
{
public:
	MSG_TYPE type; // message type, command , confirm , options, or destinations
	int msg_id; //used when confirmation is send back, in some cases the state machine will not continue executing until confirmation is sent. Used when type ==CONFIRM_MSG
	std::vector<MSG_ACTION> available_actions; // used only with type == OPTIONS_MSG
	MSG_ACTION current_action; // with type == COMMAND_MSG, what the action to execute, but with type == OPTIONS_MSG, what is the current state
	int next_destination_id; // with type == COMMAND_MSG, what the destination to go execute, but with type == OPTIONS_MSG, what is the next destination id
	int curr_destination_id; // with type == OPTIONS_MSG , indicates what is the current destination the planning is heading for
	std::vector<DESTINATION> destinations; // used with type == DESTINATIONS_MSG

	bool bErr; // when type == ERROR_MSG
	std::string err_msg; // when type == ERROR_MSG

	HMI_MSG();
	virtual ~HMI_MSG();

	/**
	 * @brief Serialize the message object to string
	 * @return the message as concatinated string
	 */
	std::string CreateStringMessage();

	/**
	 * @brief construct a message object from string message, Sample string,
	 * command,msg_id,action1;action2;action3;,currAction,next_destination,curr_destination,dest1|name1|departure1;dest2|name2|departure2;,has_error,err_msg,
	 * @param msg the HMI message received from socket or ROS topic
	 * @return HMI_MSG object
	 */
	static HMI_MSG FromString(const std::string& msg);


private:
	/**
	 * @brief Helper function, parse string by splitting it using given token
	 * @param str string to split
	 * @param token the splitting criteria
	 * @return list of strings that were separated by the token
	 */
	static std::vector<std::string> SplitString(const std::string& str, const std::string& token);
	static MSG_ACTION GetActionType(int id);
};
	} /* namespace hmi */
} /* namespace op */

#endif /* HMIMSG_H_ */
