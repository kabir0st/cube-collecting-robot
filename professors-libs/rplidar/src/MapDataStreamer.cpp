/*
 *	MapDataStreamer.cpp
 *
 *	  Date: 14 march 2017
 *	Author: Johannes van Esch
 *
 */

#include <unistd.h>
#include <syslog.h>

#include <MapDataStreamer.hpp>

namespace hh
{

MapDataStreamer::MapDataStreamer()
{
	_isStreaming = false;
	_streamingError = false;
	_lidar = NULL;
	_streamingMode = UNKNOWN_STREAMING_MODE;
	_tcpStream = NULL;

	_scanBuffer = (lidar_normal_scan_data_packet_t *) malloc(STREAMER_SCAN_BUFFER_SIZE * sizeof(lidar_normal_scan_data_packet_t));

	if (!_scanBuffer)
		syslog(LOG_ERR, "unable to allocate memory for scan buffer");

	_scanBufferIndex = 0;
}


MapDataStreamer::~MapDataStreamer()
{
	free(_scanBuffer);

	_tcpStream = NULL;
}


void MapDataStreamer::setup()
{

}


void MapDataStreamer::execute(void *args)
{

	while (_running)
	{
		if (_isStreaming)
		{

			switch (_streamingMode)
			{
				case SINGLE_PACKET_NORMAL:

					streamSinglePacketNormal();
					break;

				case COMPLETE_SCAN_NORMAL:

					streamCompleteScanNormal();
					break;

				default:

					// Unknown streaming mode, terminate streaming
					stopStreaming();
					break;
			}

		}
		else
		{
			if (_tcpStream != NULL)
			{
				delete _tcpStream;
				_tcpStream = NULL;
			}

			_scanBufferIndex = 0;

			usleep(100);
		}
	}
}


void MapDataStreamer::streamSinglePacketNormal()
{
	lidar_normal_scan_data_packet_t scanData;
	int sendBytes = 0;

	_lidar->readNormalScanDataPacket(&scanData);

	if (sendHeader(LIDAR_SINGLE_SCAN_PACKET, sizeof(scanData)) == HEADER_PACKET_LENGTH)
		sendBytes = _tcpStream->send((unsigned char *) &scanData, sizeof(scanData));

	// If something went wrong we stop streaming
	if (sendBytes <= 0)
	{
		stopStreaming();
	}
}


void MapDataStreamer::streamCompleteScanNormal()
{
	lidar_normal_scan_data_packet_t scanData;
	int sendBytes;
	int dataSize;

	_lidar->readNormalScanDataPacket(&scanData);

	// Check if we received a start indicator of a complete scan,
	// if so we send the previous complete scan data
	if (scanData.byte0 & 0x01)
	{
		// Check if we have something in the buffer
		if (_scanBufferIndex > 0)
		{
			dataSize = sizeof(scanData) * _scanBufferIndex;

			// Send the scandata
			if (sendHeader(LIDAR_COMPLETE_SCAN_PACKET, dataSize) == HEADER_PACKET_LENGTH)
				sendBytes = _tcpStream->send((unsigned char *) _scanBuffer, dataSize);

			// If something went wrong we stop streaming
			if (sendBytes <= 0)
			{
				stopStreaming();
			}

			_scanBufferIndex = 0;
		}
	}

	// Save scandata in buffer
	_scanBuffer[_scanBufferIndex] = scanData;

	if (_scanBufferIndex < (STREAMER_SCAN_BUFFER_SIZE - 1))
		_scanBufferIndex++;
}


void MapDataStreamer::startStreaming(Lidar *lidar, const std::string address, int port, streaming_mode_t mode)
{
	int tries = 0;

	libcpp::TCPConnector *tcpConnector = new libcpp::TCPConnector();

	_lidar = lidar;
	_tcpStreamAddress = new std::string(address);
	_tcpStreamPort = port;
	_streamingMode = mode;

	while (_tcpStream == NULL && tries++ < STREAMER_CONNECT_TRIES)
	{
		_tcpStream = tcpConnector->connect(_tcpStreamAddress->c_str(), _tcpStreamPort, STREAMER_CONNECT_TIMEOUT_SECONDS);

		if (_tcpStream == NULL)
		{
			syslog(LOG_NOTICE, "waiting for connect, try %d", tries);
			usleep(STREAMER_WAIT_BETWEEN_CONNECT_RETRIES_US);
		}
	}

	delete tcpConnector;

	if (_tcpStream == NULL)
	{
		_streamingError = true;
		_isStreaming = false;
	}
	else
	{
		_streamingError = false;
		_isStreaming = true;

		_lidar->scan();

	}
}


void MapDataStreamer::stopStreaming()
{
	if (_streamingError)
		syslog(LOG_ERR, "streaming stopped due streaming error");

	_isStreaming = false;
}

int MapDataStreamer::sendHeader(streaming_packet_content_t contentType, int numberOfBytes)
{
	int sendBytes;
	unsigned char header[HEADER_PACKET_LENGTH];

	header[0] = STREAMING_HEADER_SYNC_BYTE;
	header[1] = contentType;
	header[2] = (numberOfBytes >> 16) & 0xff;
	header[3] = (numberOfBytes >> 8) & 0xff;
	header[4] = numberOfBytes & 0xff;

	sendBytes = _tcpStream->send(header, HEADER_PACKET_LENGTH);

	return sendBytes;
}

}