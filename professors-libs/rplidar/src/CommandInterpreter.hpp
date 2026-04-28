/*
 *	CommandInterpreter.hpp
 *
 *	  Date: 14 march 2017
 *	Author: Johannes van Esch
 *
 */


#ifndef _COMMAND_INTERPRETER_HPP
#define _COMMAND_INTERPRETER_HPP

#define COMMAND_PARAMETER_MAX_SIZE	256

typedef struct
{
	unsigned char commandId;
	unsigned char commandParameterLength;
}
command_t;


namespace hh
{

class CommandInterpreter
{
	private:

	public:

		virtual void processCommand(command_t *command, unsigned char *commandParameters) = 0;
	
};

}

#endif
