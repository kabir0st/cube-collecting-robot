/*
 *  TCPStream.hpp
 *
 *  Based on code from Vic Hargrave - http://vichargrave.com
 *
 *  Created on: 24 mar 2016
 *      Author: Johannes van Esch
 */

#ifndef _TCPSTREAM_HPP
#define _TCPSTREAM_HPP

#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>


namespace libcpp
{

class TCPStream
{

    private:
      
        std::string m_peerIP;
        int m_peerPort;
        int m_sd;

        bool waitForReadEvent(int timeout);

        TCPStream(int sd, struct sockaddr_in* address);
        TCPStream();
        TCPStream(const TCPStream& stream);

    public:

        friend class TCPAcceptor;
        friend class TCPConnector;

        ~TCPStream();

        ssize_t send(const unsigned char* buffer, size_t len);
        ssize_t receive(unsigned char* buffer, size_t len, int timeout=0);

        std::string getPeerIP();
        int getPeerPort();

        enum 
        {
            connectionClosed = 0,
            connectionReset = -1,
            connectionTimedOut = -2
        };

};

}

#endif
