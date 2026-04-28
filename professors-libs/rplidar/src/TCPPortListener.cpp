 /*
 *	TCPPortListener.cpp
 *
 *	  Date: 15 march 2017
 *	Author: Johannes van Esch
 *
 */

#include <unistd.h>

#include <TCPPortListener.hpp>


namespace hh
{

TCPPortListener::TCPPortListener()
{
	_isListening = false;
}


TCPPortListener::~TCPPortListener()
{

}


void TCPPortListener::setup()
{

}


void TCPPortListener::execute(void *args)
{
	command_t command;
	unsigned char commandParameters[256];

	while (_running)
	{
		if (_isListening)
		{
			if (_tcpAcceptor->acceptPending())
				_tcpStream = _tcpAcceptor->accept();
			else
				_tcpStream = NULL;

			if (_tcpStream != NULL)
			{
				// Read command from stream
				_tcpStream->receive((unsigned char *) &command, 2, 0);

				// Read command parameters from stream (if any)
				if (command.commandParameterLength > 0)
					_tcpStream->receive((unsigned char *) commandParameters, command.commandParameterLength, 0);

				// Let the command interpreter take care of yhe command
				_commandInterpreter->processCommand(&command, commandParameters);

				// Clean up and wait for next connect
				delete _tcpStream;
			}
			else
				usleep(100);

		}
		else
			usleep(100);
	}

	if (_isListening)
		stopListening();

}


void TCPPortListener::startListening(const std::string address, int port, CommandInterpreter *ci)
{
	if (_isListening)
		stopListening();

	_port = port;
	_commandInterpreter = ci;

	_tcpAcceptor = new libcpp::TCPAcceptor(_port, address.c_str());

	_tcpAcceptor->start();

	_isListening = true;

}


void TCPPortListener::stopListening()
{
	if (_isListening)
	{
		_isListening = false;

		_tcpAcceptor->stop();

		delete _tcpAcceptor;
	}
}
		
}