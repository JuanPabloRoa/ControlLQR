#ifndef GPS_TELEM_PACKET_H
#define GPS_TELEM_PACKET_H

#define MAX_NUM_SATELLITES 12

#include "CommPacket.h"

#define SAT_STATUS_SEARCHING				0
#define SAT_STATUS_TRACKING					1
#define SAT_STATUS_POS_FIXED				2

class DYNAMIC_LINK CGPSTelemPacket : public CCommPacket
{
public:

	// ------------------------------------------------------------------------------
	//														  Public Member Functions
	// ------------------------------------------------------------------------------

	//! Standard Constructor

	CGPSTelemPacket();

	//! Standard Destructor

	virtual ~CGPSTelemPacket();

	//! Copy Constructor

	CGPSTelemPacket(CGPSTelemPacket *copy);

	//! Assignment Operator

	const CGPSTelemPacket & operator=(const CGPSTelemPacket &right);

	//! Fills up the packet member variables

	void Fill(PacketData NewData);

	//! Clears all member variables

	virtual void Clear();

	//! Get the GPS Velocity

	float GetGPSVelocity() { return m_GPSVelocity;}

	//! Get the GPS Altitude
	
	float GetGPSAltitude() { return m_GPSAltitude;}

	//! Get the GPS Heading

	float GetGPSHeading() { return m_GPSHeading;}

	//! Get the count down timer

	__u8 GetCountDownTimer(){ return m_CountDownTimer; }

	//! Get the Latitude 

	float GetGPSLatitude() { return m_Latitude;}

	//! Get the Longitude

	float GetGPSLongitude() { return m_Longitude;}

	//! Gets the Relative East X

	float GetRelEastX() { return m_RelEastX;}

	//! Gets the Relative North Y

	float GetRelNorthY() { return m_RelNorthY;}

	//! Gets the Home Latitude

	float GetHomeLatitude() { return m_HomeLatitude; }

	//! Gets the Home Longitude

	float GetHomeLongitude() { return m_HomeLongitude; }

	//! Returns the current command (i.e. Waypoint number)

	__u8 GetCurrentCommand() { return m_CurrentCommand;}

	//! Gets the NAV AI State

	__u8 GetNAVAIState() { return m_NAVAIState;}

	//! Get Desired Longitude

	float GetDesLongitude() { return m_DesLongitude;}

	//! Get Desired Latitude

	float GetDesLatitude() { return m_DesLatitude;}

	//! Get Time over target used for loiter times

	float GetTimeOverTarget() { return m_TimeOverTarget;}

	//! Get Distance to Target (i.e. to waypoint)

	float GetDistanceToTarget() { return m_DistanceToTarget;}

	//! Gets the Heading to the target

	float GetHeadingToTarget() { return m_HeadingToTarget;}
	
	//! Get the Feedback Loop Configuration value
	
	__u16 GetFLC() { return m_FLC;}

	//! Gets the wind heading estimate

	float GetWindDirection() { return m_WindDirection; }

	//! Gets the wind speed estimate

	float GetWindSpeed() { return m_WindSpeed;}

	//! Gets the IO pin state

	__u16 GetIOPinInfo(){ return m_IOPinInfo; }

	//! Gets the UTC Year

	__u8 GetUTCYear() { return m_UTCYear; }

	//! Gets the UTC Month

	__u8 GetUTCMonth() { return m_UTCMonth; }

	//! Gets the UTC Day

	__u8 GetUTCDay() { return m_UTCDay; }

	//! Gets the UTC Hour

	__u8 GetUTCHour() { return m_UTCHour; }

	//! Gets the UTC Minute

	__u8 GetUTCMinute() { return m_UTCMinute; }

	//! Gets the UTC Second

	__u8 GetUTCSecond() { return m_UTCSecond; }

	//! Gets the signal strength of the requested satellite

	__u8 GetSatStrength(int i) { if(i < 0 || i >= MAX_NUM_SATELLITES) return 0; return m_SatStrength[i]; }

	//! Gets the status of each satellite

	__u8 GetSatStatus(int i) { if(i < 0 || i >= MAX_NUM_SATELLITES) return 0; return m_SatStatus[i]; }

	//! Gets the Dev Short

	__u16 GetDevShort() { return m_DevShort; }

	//! Gets the Gyro R Temperature

	float GetTemperatureR(){ return m_TemperatureR; }



	// ------------------------------------------------------------------------------
	//														  Public Member Variables
	// ------------------------------------------------------------------------------
	
protected:

	// ------------------------------------------------------------------------------
	//													   Protected Member Functions
	// ------------------------------------------------------------------------------

	// ------------------------------------------------------------------------------
	//													   Protected Member Variables
	// ------------------------------------------------------------------------------

	//! Ground Speed Velocity of the UAV

	float m_GPSVelocity;

	//! GPS Altitude of the UAV

	float m_GPSAltitude;

	//! GPS Heading

	float m_GPSHeading;

	//! Count Down Timer for takeoff

	__u8 m_CountDownTimer;

	//! Latitude of the UAV
	
	float m_Latitude;

	//! Longitude of the UAV

	float m_Longitude;

	//! Relative East (X)

	float m_RelEastX;

	//! Relative North (Y)

	float m_RelNorthY;

	//! UAV Home Latitude

	float m_HomeLatitude;

	//! UAV Home Longitude

	float m_HomeLongitude;

	//! Current Command

	__u8 m_CurrentCommand;

	//! NAV AI State

	__u8 m_NAVAIState;

	//! Desired Latitude

	float m_DesLatitude;

	//! Desired Longitude

	float m_DesLongitude;

	//! Time over Target

	float m_TimeOverTarget;

	//! Distance to Target

	float m_DistanceToTarget;

	//! Heading to Target

	float m_HeadingToTarget;

	//! Feedback Loop Configuration

	__u16 m_FLC;

	//! Wind Direction

	float m_WindDirection;

	//! Wind Speed

	float m_WindSpeed;

	//! I/O Pins

	__u16 m_IOPinInfo;

	//! UTC Year

	__u8 m_UTCYear;

	//! UTC Month

	__u8 m_UTCMonth;

	//! UTC Day

	__u8 m_UTCDay;

	//! UTC Hour

	__u8 m_UTCHour;

	//! UTC Minute

	__u8 m_UTCMinute;

	//! UTC Second

	__u8 m_UTCSecond;

	//! Satellite strength
	__u8 m_SatStrength[MAX_NUM_SATELLITES];

	//! Satellite status
	__u8 m_SatStatus[MAX_NUM_SATELLITES];

	//! Dev Short
	__u16 m_DevShort;

	//! Temperature R
	float m_TemperatureR;
};

	
#endif 
