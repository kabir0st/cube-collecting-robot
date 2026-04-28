/*
 *  TCPAcceptor.cpp
 *
 *  Based on code from Vic Hargrave - http://vichargrave.com
 *
 *  Created on: 24 mar 2016
 *      Author: Johannes van Esch
 */

#include <TCPAcceptor.hpp>

#include <TCPStream.hpp>

#include <iostream>
#include <string>
#include <cstring>
#include <arpa/inet.h>
#include <syslog.h>

namespace libcpp
{

TCPAcceptor::TCPAcceptor(int port, const char* address) 
{
    m_lsd = 0;
    m_port = port;
    m_address = address;
    m_listening = false;
}

TCPAcceptor::~TCPAcceptor()
{
    if (m_lsd > 0) 
    {
        close(m_lsd);
    }
}

int TCPAcceptor::start()
{
    if (m_listening == true) 
    {
        return 0;
    }

    m_lsd = socket(PF_INET, SOCK_STREAM, 0);
    struct sockaddr_in address;

    memset(&address, 0, sizeof(address));
    address.sin_family = PF_INET;
    address.sin_port = htons(m_port);

    if (m_address.size() > 0) 
    {
        inet_pton(PF_INET, m_address.c_str(), &(address.sin_addr));
    }
    else 
    {
        address.sin_addr.s_addr = INADDR_ANY;
    }
    
    int optval = 1;
    setsockopt(m_lsd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval); 
    
    int result = bind(m_lsd, (struct sockaddr*)&address, sizeof(address));

    if (result != 0) 
    {
        syslog(LOG_ERR, "TCPAcceptor bind failed");
        return result;
    }
    
    result = listen(m_lsd, 5);
    if (result != 0) 
    {
        syslog(LOG_ERR, "TCPAcceptor Listen failed");
        return result;
    }

    m_listening = true;

    return result;
}


bool TCPAcceptor::acceptPending()
{
    bool pendingAccept;
    struct timeval tv;
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(m_lsd, &rfds);

    tv.tv_sec = 0;
    tv.tv_usec = 1000;

    if (select(m_lsd + 1, &rfds, (fd_set *) 0, (fd_set *) 0, &tv) > 0)
        pendingAccept = true;
    else
        pendingAccept = false;

    return pendingAccept;;
}


void TCPAcceptor::stop()
{
    close(m_lsd);
}


TCPStream* TCPAcceptor::accept() 
{
    if (m_listening == false) 
    {
        return NULL;
    }

    struct sockaddr_in address;
    socklen_t len = sizeof(address);
    memset(&address, 0, sizeof(address));

    int sd = ::accept(m_lsd, (struct sockaddr*)&address, &len);

    if (sd < 0) 
    {
        syslog(LOG_ERR, "TCPAcceptor accept failed");

        return NULL;
    }

    return new TCPStream(sd, &address);
}

}
