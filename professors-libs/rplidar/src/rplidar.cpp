/*
 *	rplidar.cpp
 *
 *	  Date: 13 march 2017
 *	Author: Johannes van Esch
 *
 */

#include <syslog.h>
#include <rplidar.hpp>

namespace hh
{

void rplidar::processCommand(command_t *command, unsigned char *commandParameters)
{
	syslog(LOG_NOTICE, "received command 0x%02x", command->commandId);

	switch (command->commandId)
	{
		case LIDAR_COMMAND_START:

			_lidar->setMotorPwm(LIDAR_DEFAULT_MOTOR_PWM);
			_mapDataStreamer->startStreaming(_lidar, 
											 DEFAULT_LIDAR_CLIENT_TCP_ADDRESS, 
											 DEFAULT_LIDAR_CLIENT_TCP_PORT,
											 SINGLE_PACKET_NORMAL);
			break;

		case LIDAR_COMMAND_STOP:

			_mapDataStreamer->stopStreaming();
			_lidar->stop();
			_lidar->setMotorPwm(LIDAR_IDLE_MOTOR_PWM);

			break;

		case LIDAR_SERVER_TERMINATE:

			_tcpPortListener->stopListening();
			_tcpPortListener->stop();

			break;

		default:

			break;
	}
}

int rplidar::run()
{
	// Open and set the syslog options
	openlog("rplidar", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);
	syslog(LOG_NOTICE, "start");

	// Create the objects
	_lidar = new Lidar();
	_tcpPortListener = new TCPPortListener();
	_mapDataStreamer = new MapDataStreamer();

	// Initialize the lidar sensor
	_lidar->connect();
	_lidar->stop();
	_lidar->setMotorPwm(LIDAR_IDLE_MOTOR_PWM);

	// Start the threads
	_tcpPortListener->start(NULL);
	_mapDataStreamer->start(NULL);

	// Start listening for commands on the TCP port
	_tcpPortListener->startListening(DEFAULT_LIDAR_SERVER_TCP_ADDRESS, DEFAULT_LIDAR_SERVER_TCP_PORT, this);

	// Wait for tcpPortListener thread to terminate through terminate command
	_tcpPortListener->join();

	// Terminate the MapDataStreamer thread
	_mapDataStreamer->stop();
	_mapDataStreamer->join();

	// Close syslog
	syslog(LOG_NOTICE, "stop");
	closelog();

	return LIDAR_SUCCESS;
}

}

int main(int argc, char **argv)
{
	hh::rplidar *lidar = new hh::rplidar();

	return lidar->run();
}
