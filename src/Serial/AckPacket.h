#ifndef ACK_PACKET_H
#define ACK_PACKET_H

#include "WriteRawPacket.h"

enum AckType
{
	UPLOAD_IO_CONFIG_ACK = 9,
	SET_PID_ACK = 10,	// successfully set desired telemetry
	UPLOAD_FLOAT_PACKET_ACK = 12,
	UPLOAD_INT_PACKET_ACK = 13,
	UPLOAD_BYTE_PACKET_ACK = 14,
	UPLOAD_FLASH_PACKET_ACK = 15,
	SET_FLC_ACK = 30,	// successfully set FLC
	CHECK_SENSORS_ACK = 33,
	RECALIBRATE_SENSORS_ACK = 34,
	UPLOAD_TRIMS_ACK = 40,
	ZERO_STICKS_ACK = 41,
	ALT_SPEED_OVERRIDE_ACK = 48,
	COMMAND_RECEIVED_ACK = 50,	// waypoint received successfully
	SET_CURRENT_COMMAND_ACK = 52,
	EDIT_COMMAND_ACK = 53,
	SET_AUTOPILOT_MODE_ACK = 55,
	SEND_SERVO_RAD_ACK = 63,
	SERVO_DISCONNECT_ACK = 64,
	FLASH_WRITTEN_ACK = 70,	// flash written successfully
	SET_FAILSAFE_CONFIG_ACK = 190,
	DATALOG_START_ACK = 202,	// Datalog Start Command was received
	SET_DESIRED_UNIV_ACK = 231,
	PID_UPLOAD_ACK = 238,	// successfully uploaded a PID 4-value set
	PID_TUNING_ACK = 239,
	GPS_ACK_PACKET = 248,
	DIGITAL_OUTPUT_TOGGLE_PACKET_ACK = 250
};

class DYNAMIC_LINK CAckPacket : public CWriteRawPacket
{
	friend class CTranslatePacket;

public:

	// ------------------------------------------------------------------------------
	//														  Public Member Functions
	// ------------------------------------------------------------------------------
	
	//! Constructs a Ack Packet to send
	CAckPacket(AckType Type, __u16 DestAddress = DEFAULT_SEND_DEST_ADDR, __u16 SrcAddress = DEFAULT_SEND_SRC_ADDR, __u8 PacketIDNum = 0);

	//! Gets the Ack Type
	AckType GetAckType(){ return (AckType)m_Data[6];}

	//! Gets the Nack Type
	int GetNackType(){ return (int)m_Data[6];}

	//! Changes to a Nack Pack since default is always ack
	void SetAsNackPkt();
};

#endif
