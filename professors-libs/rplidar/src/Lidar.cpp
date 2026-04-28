/*
 *	Lidar.cpp
 *
 *	  Date: 13 march 2017
 *	Author: Johannes van Esch
 *
 */

#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <iomanip>
#include <sys/ioctl.h>

#include <Lidar.hpp>

namespace hh
{


Lidar::Lidar()
{
	_serialHandle = -1;
}


Lidar::~Lidar()
{

}


/*
 * Connect to the rplidar using the default serial device.
 */
lidar_result_t Lidar::connect()
{
	return connect(LIDAR_SERIAL_DEVICE);
}


/*
 * Connect to the rplidar using the specified serial device.
 */
lidar_result_t Lidar::connect(const std::string device)
{
	_serialHandle = serialOpen(device.c_str(), LIDAR_SERIAL_BAUDRATE);

	if (_serialHandle == -1)
		return LIDAR_SERIAL_CONNECT_ERROR;
	else
	{
		// For the motor to spin we have to clear the DTR line, this
		// is not supported by wiringPi so we have to do that manually
	    uint32_t DTRBit = TIOCM_DTR;
    	ioctl(_serialHandle, TIOCMBIC, &DTRBit);

		return LIDAR_SUCCESS;
	}
}


lidar_result_t Lidar::stop()
{
	if (_serialHandle == -1)
	{
		return LIDAR_NOT_CONNECTED_ERROR;
	}
	else
	{
		serialPutchar(_serialHandle, LIDAR_REQUEST);
		serialPutchar(_serialHandle, LIDAR_STOP_COMMAND);

		delay(2);	

		serialFlush(_serialHandle);

		return LIDAR_SUCCESS;
	}
}


lidar_result_t Lidar::reset()
{
	if (_serialHandle == -1)
	{
		return LIDAR_NOT_CONNECTED_ERROR;
	}
	else
	{
		serialPutchar(_serialHandle, LIDAR_REQUEST);
		serialPutchar(_serialHandle, LIDAR_RESET_COMMAND);

		delay(4);	

		serialFlush(_serialHandle);

		return LIDAR_SUCCESS;
	}
}


unsigned char *Lidar::getInfo()
{	
	static unsigned char deviceInfo[LIDAR_DEVICE_INFO_MAX_SIZE];

	if (_serialHandle == -1)
	{
		return NULL;
	}
	else
	{
		serialPutchar(_serialHandle, LIDAR_REQUEST);
		serialPutchar(_serialHandle, LIDAR_GET_INFO_COMMAND);

		deviceInfo[0] = serialGetchar(_serialHandle);

		// Read the device information in the static array
		for (int i = 1; i < LIDAR_DEVICE_INFO_MAX_SIZE && serialDataAvail(_serialHandle); i++)
		{
			deviceInfo[i] = serialGetchar(_serialHandle);
		}

		return deviceInfo;
	}
}


lidar_result_t Lidar::getHealth(lidar_health_t *health)
{
	if (_serialHandle == -1)
	{
		return LIDAR_NOT_CONNECTED_ERROR;
	}
	else
	{
		unsigned char receivedByte[LIDAR_HEALTH_INFO_SIZE];
		lidar_response_descriptor_t response;

		serialPutchar(_serialHandle, LIDAR_REQUEST);
		serialPutchar(_serialHandle, LIDAR_GET_HEALTH_COMMAND);

		readResponseDescriptor(&response);

		for (int i = 0; i < LIDAR_HEALTH_INFO_SIZE; i++)
		{
			receivedByte[i] = serialGetchar(_serialHandle);
		}

		health->status = receivedByte[0];
		health->errorCode = receivedByte[2] << 8 | 
						    receivedByte[1];

		return LIDAR_SUCCESS;
	}

}


lidar_result_t Lidar::getSampleRate(lidar_samplerate_t *sampleRate)
{
	if (_serialHandle == -1)
	{
		return LIDAR_NOT_CONNECTED_ERROR;
	}
	else
	{
		unsigned char receivedByte[LIDAR_SAMPLERATE_SIZE];
		lidar_response_descriptor_t response;

		serialPutchar(_serialHandle, LIDAR_REQUEST);
		serialPutchar(_serialHandle, LIDAR_GET_SAMPLERATE_COMMAND);

		readResponseDescriptor(&response);

		for (int i = 0; i < LIDAR_SAMPLERATE_SIZE; i++)
		{
			receivedByte[i] = serialGetchar(_serialHandle);
		}

		sampleRate->tStandard = receivedByte[1] << 8 | receivedByte[0];
		sampleRate->tExpress = receivedByte[3] << 8 | receivedByte[2];

		return LIDAR_SUCCESS;
	}

}


lidar_result_t Lidar::readResponseDescriptor(lidar_response_descriptor_t *response)
{
	if (_serialHandle == -1)
	{
		return LIDAR_NOT_CONNECTED_ERROR;
	}
	else
	{
		unsigned char receivedByte[LIDAR_RESPONSE_DESCRIPTOR_SIZE];

		for (int i = 0; i < LIDAR_RESPONSE_DESCRIPTOR_SIZE; i++)
		{
			receivedByte[i] = serialGetchar(_serialHandle);
		}

		response->startFlag1 = receivedByte[0];
		response->startFlag2 = receivedByte[1];
		response->dataLength = (receivedByte[5] >> 2) << 24 |
							   receivedByte[4] << 16 |
							   receivedByte[3] << 8 |
							   receivedByte[2];
		response->sendMode = receivedByte[5] & 0x03;
		response->dataType = receivedByte[6];

		return LIDAR_SUCCESS;
	}
}

lidar_result_t Lidar::expressScan()
{
	if (_serialHandle == -1)
	{
		return LIDAR_NOT_CONNECTED_ERROR;
	}
	else
	{
		// TODO

		
		return LIDAR_SUCCESS;
	}

}


lidar_result_t Lidar::scan()
{
	if (_serialHandle == -1)
	{
		return LIDAR_NOT_CONNECTED_ERROR;
	}
	else
	{
		lidar_response_descriptor_t response;

		// Send scan command
		serialPutchar(_serialHandle, LIDAR_REQUEST);
		serialPutchar(_serialHandle, LIDAR_SCAN_COMMAND);

		// Read response descriptor
		readResponseDescriptor(&response);

		return LIDAR_SUCCESS;
	}
}

lidar_result_t Lidar::setMotorPwm(int pwm)
{
	if (_serialHandle == -1)
	{
		return LIDAR_NOT_CONNECTED_ERROR;
	}
	else
	{
		unsigned char message[LIDAR_PWM_MESSAGE_SIZE];

		// Make sure PWM value is within boundaries
		if (pwm < 0) pwm = 0;
		if (pwm > LIDAR_MAX_MOTOR_PWM) pwm = LIDAR_MAX_MOTOR_PWM;

		// Construct set pwm command
		message[0] = LIDAR_REQUEST;
		message[1] = LIDAR_SET_MOTOR_PWM_COMMAND;
		message[2] = 2;
		message[3] = pwm & 0xff;
		message[4] = (pwm & 0xff00) >> 8;
		message[5] = message[0] ^ message[1] ^ message[2] ^ message[3] ^ message[4];

		for (int i = 0; i < LIDAR_PWM_MESSAGE_SIZE; i++)
		{
			serialPutchar(_serialHandle, message[i]);
		}

		return LIDAR_SUCCESS;
	}

}


lidar_result_t Lidar::readNormalScanDataPacket(lidar_normal_scan_data_packet_t *scanData)
{
	if (_serialHandle == -1)
	{
		return LIDAR_NOT_CONNECTED_ERROR;
	}
	else
	{
		unsigned char receivedByte[LIDAR_NORMAL_SCAN_DATA_RESPONSE_SIZE];

		for (int i = 0; i < LIDAR_NORMAL_SCAN_DATA_RESPONSE_SIZE; i++)
			receivedByte[i] = serialGetchar(_serialHandle);

		scanData->byte0 = receivedByte[0];
		scanData->byte1 = receivedByte[1];
		scanData->byte2 = receivedByte[2];
		scanData->byte3 = receivedByte[3];
		scanData->byte4 = receivedByte[4];

		return LIDAR_SUCCESS;
	}	
}

}