/*
 *	TCPPortListener.hpp
 *
 *	  Date: 15 march 2017
 *	Author: Johannes van Esch
 *
 */

#ifndef _TCP_PORT_LISTENER_HPP
#define _TCP_PORT_LISTENER_HPP

#include <string>

#include <Thread.hpp>
#include <TCPAcceptor.hpp>
#include <TCPStream.hpp>
#include <CommandInterpreter.hpp>

namespace hh
{

class TCPPortListener: public libcpp::Thread
{
	private:

		int _port;
		CommandInterpreter *_commandInterpreter;
		bool _isListening;
		
		libcpp::TCPAcceptor *_tcpAcceptor;
		libcpp::TCPStream *_tcpStream;

	public:

		TCPPortListener();
		virtual ~TCPPortListener();

		void setup();
		void execute(void *args);

		void startListening(const std::string address, int port, CommandInterpreter *ci);
		void stopListening();

};

}

#endif