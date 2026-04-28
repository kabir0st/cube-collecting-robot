/*
 *  TCPAcceptor.hpp
 *
 *  Based on code from Vic Hargrave - http://vichargrave.com
 *
 *  Created on: 24 mar 2016
 *      Author: Johannes van Esch
 */

#ifndef _TCPACCEPTOR_HPP
#define _TCPACCEPTOR_HPP

#include <string>
#include <netinet/in.h>
#include <sstream>

namespace libcpp
{

class TCPStream;

class TCPAcceptor
{
    private:

        int m_lsd;
        int m_port;
        std::string m_address;
        bool m_listening;

        TCPAcceptor() {}

        std::ostringstream _stringStream;

    public:

        TCPAcceptor(int port, const char* address="");
        ~TCPAcceptor();

        int start();
        void stop();

        bool acceptPending();
        TCPStream* accept();
};

}

#endif
