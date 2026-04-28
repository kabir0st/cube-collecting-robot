/*
 *	TCPConnector.hpp
 *
 *  Based on code from Vic Hargrave - http://vichargrave.com
 *
 *  Created on: 24 mar 2016
 *      Author: Johannes van Esch
 */

#ifndef _TCPCONNECTOR_HPP
#define _TCPCONNECTOR_HPP

#include <netinet/in.h>

namespace libcpp
{

class TCPStream;

class TCPConnector
{
    private:

        int resolveHostName(const char* host, struct in_addr* addr);

    public:

        TCPConnector();

        TCPStream* connect(const char* server, int port);
        TCPStream* connect(const char* server, int port, int timeout);
};

}

#endif
