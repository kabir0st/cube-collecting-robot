/*
 *	MapDataStreamer.hpp
 *
 *	  Date: 14 march 2017
 *	Author: Johannes van Esch
 *
 */

#ifndef _MAP_DATA_STREAMER_HPP
#define _MAP_DATA_STREAMER_HPP

#include <Thread.hpp>
#include <Lidar.hpp>
#include <TCPConnector.hpp>
#include <TCPStream.hpp>

#define STREAMER_CONNECT_TIMEOUT_SECONDS			10
#define STREAMER_CONNECT_TRIES						10
#define STREAMER_WAIT_BETWEEN_CONNECT_RETRIES_US	10000
#define STREAMER_SCAN_BUFFER_SIZE					8192

#define STREAMING_HEADER_SYNC_BYTE	0xa5
#define HEADER_PACKET_LENGTH		5

typedef enum
{
	LIDAR_SINGLE_SCAN_PACKET 	= 0x10,		
	LIDAR_COMPLETE_SCAN_PACKET 	= 0x15,		
	LIDAR_INFO_PACKET			= 0x20,		// Not yet implemented!
	LIDAR_HEALTH_PACKET			= 0x30,		// Not yet implemented!
	LIDAT_SAMPLE_RATE_PACKET	= 0x40		// Not yet implemented!
}
streaming_packet_content_t;

typedef enum 
{
	SINGLE_PACKET_NORMAL	= 10,
	SINGLE_PACKET_EXPRESS	= 20,
	COMPLETE_SCAN_NORMAL	= 30,
	COMPLETE_SCAN_EXPRESS	= 40,
	UNKNOWN_STREAMING_MODE	= 99
} 
streaming_mode_t;

namespace hh
{

class TCPStream;

class MapDataStreamer: public libcpp::Thread
{
	private:

		bool _isStreaming;
		Lidar *_lidar;
		std::string *_tcpStreamAddress;
		int _tcpStreamPort;
		streaming_mode_t _streamingMode;

		libcpp::TCPStream *_tcpStream;

		lidar_normal_scan_data_packet_t *_scanBuffer;
		unsigned int _scanBufferIndex;

		bool _streamingError;

		void streamSinglePacketNormal();
		void streamCompleteScanNormal();

		int sendHeader(streaming_packet_content_t contentType, int numberOfBytes);

	public:

		MapDataStreamer();
		virtual ~MapDataStreamer();

		void setup();
		void execute(void *args);

		void startStreaming(Lidar *lidar, const std::string address, int port, streaming_mode_t mode);
		void stopStreaming();

};

}

#endif