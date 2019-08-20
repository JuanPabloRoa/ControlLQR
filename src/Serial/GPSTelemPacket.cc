#include "GPSTelemPacket.h"
#include <math.h>

CGPSTelemPacket::CGPSTelemPacket()
{
	//Clear it out
	Clear();
}

CGPSTelemPacket::~CGPSTelemPacket()
{
	//Clear it out
	Clear();
}


CGPSTelemPacket::CGPSTelemPacket(CGPSTelemPacket *copy)
{
	//Call Base Class Copy Constructor first
	CCommPacket((CCommPacket *)copy);

	m_GPSAltitude		= copy->m_GPSAltitude;
	m_GPSVelocity		= copy->m_GPSVelocity;
	m_GPSHeading		= copy->m_GPSHeading;
	m_CountDownTimer    = copy->m_CountDownTimer;
	m_Latitude			= copy->m_Latitude;
	m_Longitude			= copy->m_Longitude;
	m_RelEastX			= copy->m_RelEastX;
	m_RelNorthY			= copy->m_RelNorthY;
	m_HomeLatitude		= copy->m_HomeLatitude;
	m_HomeLongitude		= copy->m_HomeLongitude;
	m_CurrentCommand	= copy->m_CurrentCommand;
	m_NAVAIState		= copy->m_NAVAIState;
	m_DesLatitude		= copy->m_DesLatitude;
	m_DesLongitude		= copy->m_DesLongitude;
	m_TimeOverTarget	= copy->m_TimeOverTarget;
	m_DistanceToTarget	= copy->m_DistanceToTarget;
	m_HeadingToTarget	= copy->m_HeadingToTarget;
	m_FLC				= copy->m_FLC;
	m_WindDirection		= copy->m_WindDirection;
	m_WindSpeed			= copy->m_WindSpeed;
	m_IOPinInfo			= copy->m_IOPinInfo;
	m_UTCYear			= copy->m_UTCYear;
	m_UTCMonth			= copy->m_UTCMonth;
	m_UTCDay			= copy->m_UTCDay;
	m_UTCHour			= copy->m_UTCHour;
	m_UTCMinute			= copy->m_UTCMinute;
	m_UTCSecond			= copy->m_UTCSecond;
	m_DevShort			= copy->m_DevShort;
	m_TemperatureR		= copy->m_TemperatureR;

	for(int i=0; i < MAX_NUM_SATELLITES; i++)
		m_SatStrength[i] = copy->m_SatStrength[i];
}

const CGPSTelemPacket& CGPSTelemPacket::operator =(const CGPSTelemPacket &right)
{
	if(&right != this)
	{
		this->CCommPacket::operator =(right);
		
		m_GPSAltitude		= right.m_GPSAltitude;
		m_GPSVelocity		= right.m_GPSVelocity;
		m_GPSHeading		= right.m_GPSHeading;
		m_CountDownTimer	= right.m_CountDownTimer;
		m_Latitude			= right.m_Latitude;
		m_Longitude			= right.m_Longitude;
		m_RelEastX			= right.m_RelEastX;
		m_RelNorthY			= right.m_RelNorthY;
		m_HomeLatitude		= right.m_HomeLatitude;
		m_HomeLongitude		= right.m_HomeLongitude;
		m_CurrentCommand	= right.m_CurrentCommand;
		m_NAVAIState		= right.m_NAVAIState;
		m_DesLatitude		= right.m_DesLatitude;
		m_DesLongitude		= right.m_DesLongitude;
		m_TimeOverTarget	= right.m_TimeOverTarget;
		m_DistanceToTarget	= right.m_DistanceToTarget;
		m_HeadingToTarget	= right.m_HeadingToTarget;
		m_FLC				= right.m_FLC;
		m_WindDirection		= right.m_WindDirection;
		m_WindSpeed			= right.m_WindSpeed;
		m_IOPinInfo			= right.m_IOPinInfo;
		m_UTCYear			= right.m_UTCYear;
		m_UTCMonth			= right.m_UTCMonth;
		m_UTCDay			= right.m_UTCDay;
		m_UTCHour			= right.m_UTCHour;
		m_UTCMinute			= right.m_UTCMinute;
		m_UTCSecond			= right.m_UTCSecond;
		m_DevShort			= right.m_DevShort;
		m_TemperatureR		= right.m_TemperatureR;

		for(int i=0; i < MAX_NUM_SATELLITES; i++)
			m_SatStrength[i] = right.m_SatStrength[i];	
	}

	return *this;
}

void CGPSTelemPacket::Clear()
{
	//Call Base Class First
	CCommPacket::Clear();

	m_GPSAltitude		= 0;
	m_GPSVelocity		= 0;
	m_GPSHeading		= 0;
	m_CountDownTimer	= 0;
	m_Latitude			= 0;
	m_Longitude			= 0;
	m_RelEastX			= 0;
	m_RelNorthY			= 0;
	m_HomeLatitude		= 0;
	m_HomeLongitude		= 0;
	m_CurrentCommand	= 0;
	m_NAVAIState		= 0;
	m_DesLatitude		= 0;
	m_DesLongitude		= 0;
	m_TimeOverTarget	= 0;
	m_DistanceToTarget	= 0;
	m_HeadingToTarget	= 0;
	m_FLC				= 0;	
	m_WindDirection		= 0;
	m_WindSpeed			= 0;
	m_IOPinInfo			= 0;
	m_UTCYear			= 0;
	m_UTCMonth			= 0;
	m_UTCDay			= 0;
	m_UTCHour			= 0;
	m_UTCMinute			= 0;
	m_UTCSecond			= 0;
	m_DevShort			= 0;
	m_TemperatureR		= 0;

	for(int i=0; i < MAX_NUM_SATELLITES; i++)
		m_SatStrength[i] = 0;
}

void CGPSTelemPacket::Fill(PacketData NewData)
{
	//Call base class first
	CCommPacket::Fill(NewData);

	m_GPSVelocity		= (ReadUnsignedShort(6) / 20.0f) - 10.0f;
	m_GPSAltitude		= (ReadUnsignedShort(8) / 6.0f) - 1000.0f;
	m_GPSHeading		= ReadUnsignedShort(10) / 1000.0f;

	m_CountDownTimer	= ReadUnsignedChar(12);
	m_TemperatureR		= (ReadUnsignedChar(13) / 2.8f) - 10.0f;
	m_Latitude			= ReadFloat(14);

	m_DevShort			= ReadShort(18);
	m_Longitude			= ReadFloat(20);
	m_HomeLatitude		= ReadFloat(24);
	m_HomeLongitude		= ReadFloat(28);

	//Compute relative position from home lat/lon
	m_RelNorthY			= (m_Latitude - m_HomeLatitude) * 1852.23f * 60.0f;
	m_RelEastX			= (m_Longitude - m_HomeLongitude) * cos(m_Latitude * .017453f) * 1852.23f * 60.0f;

	m_CurrentCommand	= ReadUnsignedChar(32);
	m_NAVAIState		= ReadUnsignedChar(33);
	m_DesLatitude		= ReadFloat(34);
	m_DesLongitude		= ReadFloat(38);
	m_TimeOverTarget	= ReadFloat(42);
	m_DistanceToTarget	= ReadFloat(46);
	m_HeadingToTarget	= ReadUnsignedShort(50) / 1000.0f;
	m_FLC				= ReadShort(52);
	m_WindDirection		= ReadUnsignedShort(54) / 1000.0f;
	m_WindSpeed			= ReadUnsignedChar(56) / 6.0f;
	m_IOPinInfo			= ReadUnsignedShort(57);
	m_UTCYear			= ReadUnsignedChar(59);
	m_UTCMonth			= ReadUnsignedChar(60);
	m_UTCDay			= ReadUnsignedChar(61);
	m_UTCHour			= ReadUnsignedChar(62);
	m_UTCMinute			= ReadUnsignedChar(63);
	m_UTCSecond			= ReadUnsignedChar(64);
	
	int i;
	for(i = 0; i < MAX_NUM_SATELLITES; i++)
		m_SatStrength[i] = (ReadUnsignedChar(65+i) & 0x3f);

	for(i = 0; i < MAX_NUM_SATELLITES; i++)
		m_SatStatus[i] = (ReadUnsignedChar(65+i) & 0xC0) >> 6;
}
