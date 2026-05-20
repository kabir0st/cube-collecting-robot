#pragma once
/*
 *	Lidar.hpp
 *
 *	  Date: 13 march 2017
 *	Author: Johannes van Esch
 *
 */

#ifndef _LIDAR_HPP
#define _LIDAR_HPP

#include <string>

#define LIDAR_SERIAL_DEVICE				"/dev/ttyUSB0"
#define LIDAR_SERIAL_BAUDRATE			115200

#define LIDAR_REQUEST					0xa5

#define LIDAR_STOP_COMMAND				0x25
#define LIDAR_RESET_COMMAND				0x40
#define LIDAR_SCAN_COMMAND				0x20
#define LIDAR_EXPRESS_SCAN_COMMAND		0x82
#define LIDAR_FORCE_SCAN_COMMAND		0x21
#define LIDAR_GET_INFO_COMMAND			0x50
#define LIDAR_GET_HEALTH_COMMAND		0x52
#define LIDAR_GET_SAMPLERATE_COMMAND	0x59
#define LIDAR_SET_MOTOR_PWM_COMMAND		0xF0

#define LIDAR_START_FLAG1				0xa5
#define LIDAR_START_FLAG2				0x5a

#define LIDAR_RESPONSE_DESCRIPTOR_SIZE			7
#define LIDAR_HEALTH_INFO_SIZE					3
#define LIDAR_SAMPLERATE_SIZE					4
#define LIDAR_DEVICE_INFO_MAX_SIZE				64
#define LIDAR_PWM_MESSAGE_SIZE					6
#define LIDAR_NORMAL_SCAN_DATA_RESPONSE_SIZE	5

#define LIDAR_MAX_MOTOR_PWM						1023
#define LIDAR_DEFAULT_MOTOR_PWM					660
#define LIDAR_IDLE_MOTOR_PWM					0

typedef enum
{
	LIDAR_SUCCESS					= 0,
	LIDAR_SERIAL_CONNECT_ERROR		= 10,
	LIDAR_BUFFER_TOO_SMALL_ERROR	= 20,
	LIDAR_NOT_CONNECTED_ERROR		= 30,
	LIDAR_UNKOWN_ERROR				= 999
} 
lidar_result_t;


typedef enum
{
	LIDAR_IDLE 				= 0,
	LIDAR_SCANNING			= 1,
	LIDAR_EXPRESS_SCANNING	= 2,
	LIDAR_FORCE_SCANNING	= 3,
	LIDAR_UNKNOWN_MODE		= 999
} 
lidar_mode_t;


typedef struct
{
	unsigned char status;
	unsigned int errorCode;
}
lidar_health_t;

typedef struct
{
	unsigned char startFlag1;
	unsigned char startFlag2;	
	unsigned int dataLength;
	unsigned char sendMode;
	unsigned char dataType;
}
lidar_response_descriptor_t;

typedef struct 
{
	unsigned int tStandard;
	unsigned int tExpress;
}
lidar_samplerate_t;

typedef struct 
{
	unsigned char byte0;
	unsigned char byte1;
	unsigned char byte2;
	unsigned char byte3;
	unsigned char byte4;
}
lidar_normal_scan_data_packet_t;


namespace hh
{

	class Lidar
	{
		private:

			int _serialHandle;

			lidar_result_t readResponseDescriptor(lidar_response_descriptor_t *response);

		public:

			Lidar();
			virtual ~Lidar();

			lidar_result_t connect();
			lidar_result_t connect(const std::string device);
			// Close the underlying serial FD if open. Safe to call multiple
			// times; no-op when already disconnected. Called from the
			// destructor so the FD is released even on error/exception paths.
			void disconnect();

			lidar_result_t stop();
			lidar_result_t reset();
			unsigned char *getInfo();
			lidar_result_t getHealth(lidar_health_t *health);
			lidar_result_t getSampleRate(lidar_samplerate_t *sampleRate);
			lidar_result_t scan();
			lidar_result_t setMotorPwm(int pwm);

			lidar_result_t readNormalScanDataPacket(lidar_normal_scan_data_packet_t *scanData);

	};

}

#endif
