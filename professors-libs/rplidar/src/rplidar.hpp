/*
 *	rplidar.hpp
 *
 *	  Date: 13 march 2017
 *	Author: Johannes van Esch
 *
 */

#ifndef _RPLIDAR_HPP
#define _RPLIDAR_HPP

#define LIDAR_COMMAND_START				0x10
#define LIDAR_COMMAND_STOP				0x20
#define LIDAR_SERVER_TERMINATE			0x99

#define DEFAULT_LIDAR_SERVER_TCP_ADDRESS	"127.0.0.1"
#define DEFAULT_LIDAR_SERVER_TCP_PORT		9887
#define DEFAULT_LIDAR_CLIENT_TCP_ADDRESS	"127.0.0.1"
#define DEFAULT_LIDAR_CLIENT_TCP_PORT		9888

#include <CommandInterpreter.hpp>
#include <TCPPortListener.hpp>
#include <Lidar.hpp>
#include <MapDataStreamer.hpp>

namespace hh
{

class rplidar: public virtual CommandInterpreter
{
	private:

		Lidar *_lidar;
		TCPPortListener *_tcpPortListener;
		MapDataStreamer *_mapDataStreamer;

	public:

		int run();

		void processCommand(command_t *command, unsigned char *commandParameters);
};

}

#endif