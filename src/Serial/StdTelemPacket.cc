#include "StdTelemPacket.h"

CStdTelemPacket::CStdTelemPacket()
{
	Clear();
}

CStdTelemPacket::~CStdTelemPacket()
{
}

CStdTelemPacket::CStdTelemPacket(CStdTelemPacket *copy)
{
	//Call Base Class Copy Constructor first
	CCommPacket((CCommPacket *)copy);

	m_BatteryVoltage =		copy->m_BatteryVoltage;
	m_BatteryCurrent =		copy->m_BatteryCurrent;
	m_Altitude =			copy->m_Altitude;
	m_Velocity =			copy->m_Velocity;
	m_Roll =				copy->m_Roll;
	m_Pitch =				copy->m_Pitch;
	m_Heading =				copy->m_Heading;
	m_TurnRate =			copy->m_TurnRate;
	m_DesiredAltitude =		copy->m_DesiredAltitude;
	m_DesiredVelocity =		copy->m_DesiredVelocity;
	m_DesiredRoll =			copy->m_DesiredRoll;
	m_DesiredPitch =		copy->m_DesiredPitch;
	m_DesiredHeading =		copy->m_DesiredHeading;
	m_DesiredTurnRate =		copy->m_DesiredTurnRate;
	m_NumSatellites =		copy->m_NumSatellites;
	m_SystemStatus =		copy->m_SystemStatus;
	m_AIAltState =			copy->m_AIAltState;
	m_RCPacketsPerSec =		copy->m_RCPacketsPerSec;
	m_RSSIPlane =			copy->m_RSSIPlane;
	m_UAVMode =				copy->m_UAVMode;
	m_FailSafeStatus =		copy->m_FailSafeStatus;
	m_MagHeading =			copy->m_MagHeading;
	m_AirborneTimer =		copy->m_AirborneTimer;
	m_AvionicsTimer =		copy->m_AvionicsTimer;
	m_SystemFlags =			copy->m_SystemFlags;
	m_Payload1 =			copy->m_Payload1;
	m_Payload2 =			copy->m_Payload2;

	for(int i=0; i < MAX_NUM_SERVOS; i++)
		m_ServoVal[i] =		copy->m_ServoVal[i];
}

const CStdTelemPacket &CStdTelemPacket::operator =(const CStdTelemPacket &right)
{
	if(&right != this)
	{
		this->CCommPacket::operator =(right);
		
		m_BatteryVoltage =		right.m_BatteryVoltage;
		m_BatteryCurrent =		right.m_BatteryCurrent;
		m_Altitude =			right.m_Altitude;
		m_Velocity =			right.m_Velocity;
		m_Roll =				right.m_Roll;
		m_Pitch =				right.m_Pitch;
		m_Heading =				right.m_Heading;
		m_TurnRate =			right.m_TurnRate;
		m_DesiredAltitude =		right.m_DesiredAltitude;
		m_DesiredVelocity =		right.m_DesiredVelocity;
		m_DesiredRoll =			right.m_DesiredRoll;
		m_DesiredPitch =		right.m_DesiredPitch;
		m_DesiredHeading =		right.m_DesiredHeading;
		m_DesiredTurnRate =		right.m_DesiredTurnRate;
		m_NumSatellites =		right.m_NumSatellites;
		m_SystemStatus =		right.m_SystemStatus;
		m_AIAltState =			right.m_AIAltState;
		m_RCPacketsPerSec =		right.m_RCPacketsPerSec;
		m_RSSIPlane =			right.m_RSSIPlane;
		m_UAVMode =				right.m_UAVMode;
		m_FailSafeStatus =		right.m_FailSafeStatus;
		m_MagHeading =			right.m_MagHeading;
		m_AirborneTimer =		right.m_AirborneTimer;
		m_AvionicsTimer =		right.m_AvionicsTimer;
		m_SystemFlags =			right.m_SystemFlags;
		m_Payload1 =			right.m_Payload1;
		m_Payload2 =			right.m_Payload2;

		for(int i=0; i < MAX_NUM_SERVOS; i++)
			m_ServoVal[i] =		right.m_ServoVal[i];
	}

	return *this;
}

void CStdTelemPacket::Clear()
{
	//Call Base Class Clear first
	CCommPacket::Clear();

	m_BatteryVoltage =		0;
	m_BatteryCurrent =		0;
	m_Altitude =			0;
	m_Velocity =			0;
	m_Roll =				0;
	m_Pitch =				0;
	m_Heading =				0;
	m_TurnRate =			0;
	m_DesiredAltitude =		0;
	m_DesiredVelocity =		0;
	m_DesiredRoll =			0;
	m_DesiredPitch =		0;
	m_DesiredHeading =		0;
	m_DesiredTurnRate =		0;
	m_NumSatellites =		0;
	m_SystemStatus =		0;
	m_AIAltState =			0;
	m_RCPacketsPerSec =		0;
	m_RSSIPlane =			0;
	m_UAVMode =				0;
	m_FailSafeStatus =		0;
	m_MagHeading =			0;
	m_AirborneTimer =		0;
	m_AvionicsTimer =		0;
	m_SystemFlags =			0;
	m_Payload1 =			0;
	m_Payload2 =			0;

	for(int i=0; i < MAX_NUM_SERVOS; i++)
		m_ServoVal[i] =		0;

}

void CStdTelemPacket::Fill(PacketData NewData)
{
	//Call Base Class Fill first
	CCommPacket::Fill(NewData);

	//Copy each part to the correct area
	// actual (current) values
	m_Altitude =				(ReadUnsignedShort(6) / 6.0f) - 1000.0f;
	m_Velocity =				(ReadUnsignedShort(8) / 20.0f) - 10.0f;
	m_Roll =					ReadShort(10) / 1000.0f;
	m_Pitch =					ReadShort(12) / 1000.0f;
	m_Heading =					ReadUnsignedShort(14) / 1000.0f;
	m_TurnRate =				ReadShort(16) / 1000.0f;

	// system values
	m_RSSIPlane =				ReadUnsignedChar(18);
	m_RCPacketsPerSec =			ReadUnsignedChar(19) / 2;
	m_BatteryCurrent =			ReadUnsignedChar(20) / 3.0f;
	m_BatteryVoltage =			ReadUnsignedChar(21) / 5.0f;
	m_SystemStatus =			ReadUnsignedShort(22);
	m_NumSatellites =			ReadUnsignedChar(24);
	m_AIAltState =				ReadUnsignedChar(25);

	// desired values
	m_DesiredAltitude =			(ReadUnsignedShort(26) / 6.0f) - 1000.0f;
	m_DesiredVelocity =			(ReadUnsignedChar(28) / 2.0f) - 10.0f;
	m_DesiredRoll =				ReadShort(29) / 1000.0f;
	m_DesiredPitch =			ReadShort(31) / 1000.0f;
	m_DesiredHeading =			ReadShort(33) / 1000.0f;
	m_DesiredTurnRate =			ReadShort(35) / 1000.0f;

	// servos
	m_ServoVal[0] =			(ReadUnsignedChar(37) - 128.0f) / 140.0f;
	m_ServoVal[1] =			(ReadUnsignedChar(38) - 128.0f) / 140.0f;
	m_ServoVal[2] =			ReadUnsignedChar(39);
	m_ServoVal[3] =			(ReadUnsignedChar(40) - 128.0f) / 140.0f;

	//UAV Mode
	m_UAVMode =				ReadUnsignedChar(41);

	//Fail Safe
	m_FailSafeStatus =		ReadUnsignedShort(42);

	//Mag heading
	m_MagHeading =			ReadUnsignedShort(44) / 1000.0f;

	//Airborne timer
	m_AirborneTimer =		ReadFloat(46);

	//Avionics timer
	m_AvionicsTimer =		ReadFloat(50);

	//System Flags
	m_SystemFlags =			ReadUnsignedShort(54);

	//Payload1
	m_Payload1 =			ReadUnsignedShort(56);

	//Payload2
	m_Payload2 =			ReadUnsignedShort(58);
}

float CStdTelemPacket::GetServo(s32 ServoNum) const
{
	if(ServoNum < 0 || ServoNum >= MAX_NUM_SERVOS) return 0;

	return m_ServoVal[ServoNum];
}
