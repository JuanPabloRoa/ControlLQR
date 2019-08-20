#ifndef STDTELEM_PACKET_H
#define STDTELEM_PACKET_H

#include "CommPacket.h"

#define TELEMETRY_SYSTEM_STATUS_SERVO_INIT			0x0001
#define TELEMETRY_SYSTEM_STATUS_PRESSURE_INIT		0x0002
#define TELEMETRY_SYSTEM_STATUS_GPS_HOME_INIT		0x0004
#define TELEMETRY_SYSTEM_STATUS_GPS_LOCK			0x0008
#define TELEMETRY_SYSTEM_STATUS_PSI_INIT			0x0010
#define TELEMETRY_SYSTEM_STATUS_PILOT_IN_CONTROL	0x0020
#define TELEMETRY_SYSTEM_STATUS_HAVE_DATALOG_DATA	0x0040
#define TELEMETRY_SYSTEM_STATUS_GPS_GOOD_COMM		0x0080
#define TELEMETRY_SYSTEM_STATUS_FLASH_WRITE			0x0100
#define TELEMETRY_SYSTEM_STATUS_BAD_COMM			0x0200
#define TELEMETRY_SYSTEM_STATUS_LOW_BATTERY			0x0400
#define TELEMETRY_SYSTEM_STATUS_CIC_NO_RC_COMM		0x0800
#define TELEMETRY_SYSTEM_STATUS_TEMP_COMM_RUNNING	0x1000
#define TELEMETRY_SYSTEM_STATUS_WAYPOINT_UPLDED		0x2000
#define TELEMETRY_SYSTEM_STATUS_AIRBORNE			0x4000
#define TELEMETRY_SYSTEM_STATUS_MAG_CAL_RUNNING		0x8000

#define TELEMETRY_AUTOPILOT_MODE_SPEED				0
#define TELEMETRY_AUTOPILOT_MODE_ALTITUDE			1
#define TELEMETRY_AUTOPILOT_MODE_NAVIGATION			2
#define TELEMETRY_AUTOPILOT_MODE_HOME				3
#define TELEMETRY_AUTOPILOT_MODE_RALLEY				4
#define TELEMETRY_AUTOPILOT_MODE_LOITER				5
#define TELEMETRY_AUTOPILOT_MODE_TOFF_SPIRAL		6
#define TELEMETRY_AUTOPILOT_MODE_LAND_APPRCH		7
#define TELEMETRY_AUTOPILOT_MODE_LAND_CIRCLE		8
#define TELEMETRY_AUTOPILOT_MODE_LAND_NOW			9
#define TELEMETRY_AUTOPILOT_MODE_MANUAL				10
#define TELEMETRY_AUTOPILOT_MODE_TOFF_WPT			11
#define TELEMETRY_AUTOPILOT_MODE_LAND_JOY			12
#define TELEMETRY_AUTOPILOT_MODE_LAND_GENERIC		13
#define TELEMETRY_AUTOPILOT_MODE_TOFF_JOY			14
#define TELEMETRY_AUTOPILOT_MODE_TOFF_WPT_JOY		15
#define TELEMETRY_AUTOPILOT_MODE_FOLOW				16

#define TELEMETRY_FAILSAFE_MANUAL_LEVEL				0x0001
#define	TELEMETRY_FAILSAFE_MANUAL_GO_HOME_GPS		0x0002
#define TELEMETRY_FAILSAFE_MANUAL_GO_HOME_NO_GPS	0x0004
#define TELEMETRY_FAILSAFE_LOSS_COMM_GPS			0x0008
#define TELEMETRY_FAILSAFE_LOSS_COMM_NO_GPS			0x0010
#define TELEMETRY_FAILSAFE_GROUND					0x0020
#define TELEMETRY_FAILSAFE_LOW_BATT_GPS				0x0040
#define TELEMETRY_FAILSAFE_LOW_BATT_NO_GPS			0x0080
#define TELEMETRY_FAILSAFE_CRIT_BATT_GPS			0x0100
#define TELEMETRY_FAILSAFE_CRIT_BATT_NO_GPS			0x0200
#define TELEMETRY_FAILSAFE_LOSS_GPS_CIRCLE			0x0400
#define TELEMETRY_FAILSAFE_LOSS_GPS_LAND			0x0800			

#define TELEMETRY_SERVO_AILERON						0
#define TELEMETRY_SERVO_ELEVATOR					1
#define TELEMETRY_SERVO_THROTTLE					2
#define TELEMETRY_SERVO_RUDDER						3

#define MAX_NUM_SERVOS 4


class DYNAMIC_LINK CStdTelemPacket : public CCommPacket
{
public:

//Public Member Functions

	//! Constructor

	CStdTelemPacket();

	//! Destructor

	virtual ~CStdTelemPacket();

	//! Copy Constructor

	CStdTelemPacket(CStdTelemPacket *copy);

	//! Assignemnt Operator

	const CStdTelemPacket &operator=(const CStdTelemPacket &right);

	//! Fills up the packets Member Variables

	void Fill(PacketData NewData);

	//! Clears all Member Variables

	virtual void Clear();

	//! Gets the Batter Voltage from the Packet

	float GetBatteryVoltage(){ return m_BatteryVoltage;}

	//! Gets the Batter Current from the Packet

	float GetBatteryCurrent(){ return m_BatteryCurrent;}

	//! Gets the Altitude from the Packet

	float GetAltitude(){ return m_Altitude;}

	//! Gets the Velocity from the Packet

	float GetVelocity(){ return m_Velocity;}

	//! Gets the Roll Angle from the Packet

	float GetRoll(){ return m_Roll;}

	//! Gets the Pitch Angle from the Packet

	float GetPitch(){ return m_Pitch;}

	//! Gets the Heading from the Packet

	float GetHeading(){ return m_Heading;}

	//! Gets the Turn Rate from the Packet

	float GetTurnRate(){ return m_TurnRate;}

	//! Gets the Desired Altitude from the Packet

	float GetDesiredAltitude(){ return m_DesiredAltitude;}

	//! Gets the Desired Velocity from the Packet

	float GetDesiredVelocity(){ return m_DesiredVelocity;}

	//! Gets the Desired Roll from the Packet

	float GetDesiredRoll(){ return m_DesiredRoll;}

	//! Gets the Desired Pitch from the Packet

	float GetDesiredPitch(){ return m_DesiredPitch;}

	//! Gets the Desired Heading from the Packet

	float GetDesiredHeading(){ return m_DesiredHeading;}

	//! Gets the Desired Turn Rate from the Packet

	float GetDesiredTurnRate(){ return m_DesiredTurnRate;}

	//! Gets the Number of Satellites from the Packet

	__u8 GetNumSatellites() { return m_NumSatellites;}

	//! Gets the System Status from the Packet
	/** Use the #defines at the top to mask the system status
	  * For example: SystemStatus | TELEMETRY_SYSTEM_STATUS_SERVO_INIT
	  * to tell if the servos have been initialized yet
	  */

	__u16 GetSystemStatus() { return m_SystemStatus;}

	//! Gets the AI Altitude State from the Packet
	/** The AI Altitude State tells how the autopilot is commanding
	  * its altitude controllers.  For example it could be using climb rate
	  * to get to a desired altitude instead of its measure barrometric altitude
	  */

	__u8 GetAIAltState() { return m_AIAltState;}

	//! Gets the RC Packets Per Second from the Packet

	__u8 GetRCPacketsPerSec() { return m_RCPacketsPerSec;}

	//! Gets the Received Signal Strength Indicator on the Plane from the Packet

	__u8 GetRSSIPlane() { return m_RSSIPlane;}

	//! Gets the Servo Values from the Packet

	float GetServo(__s32 ServoNum) const;

	//! Returns the current autopilot mode

	__u8 GetUAVMode() { return m_UAVMode;}

	//! Returns the fail safe status bitmask

	__u16 GetFailSafeStatus() { return m_FailSafeStatus;}

	//! Returns the magnetometer heading

	float GetMagHeading() { return m_MagHeading; }

	//! Returns the Airborne timer

	float GetAirborneTimer() { return m_AirborneTimer; }

	//! Returns the Avionics timer

	float GetAvionicsTimer() { return m_AvionicsTimer; }

	//! Returns the System flags

	__u16	GetSystemFlags() { return m_SystemFlags; }

	//! Returns the payload status 1

	__u16 GetPayload1() { return m_Payload1; }

	//! Returns the payload status 2

	__u16 GetPayload2() { return m_Payload2; }

//Public Member Variables
	
protected:

//Protected Member Functions

//Protected Member Variables

	//! Battery Voltage

	float m_BatteryVoltage;

	//! Battery Current

	float m_BatteryCurrent;

	//! Altitude

	float m_Altitude;

	//! Velocity

	float m_Velocity;

	//! Roll

	float m_Roll;

	//! Pitch

	float m_Pitch;

	//! Heading

	float m_Heading;

	//! Turn Rate

	float m_TurnRate;

	//! Desired Altitude

	float m_DesiredAltitude;

	//! Desired Velocity

	float m_DesiredVelocity;

	//! Desired Roll

	float m_DesiredRoll;

	//! Desired Pitch

	float m_DesiredPitch;

	//! Desired Heading

	float m_DesiredHeading;

	//! Desired Turn Rate

	float m_DesiredTurnRate;

	//! Number of Satellites

	__u8 m_NumSatellites;

	//! System Status

	__u16 m_SystemStatus;

	//! Altitude Hold AI State

	__u8 m_AIAltState;

	//! RC Comm Packets Per Sec

	__u8 m_RCPacketsPerSec;

	//! RSSI Value of the plane

	__u8 m_RSSIPlane;
	
	//! Servo Values
	
	float m_ServoVal[MAX_NUM_SERVOS];

	//! UAV Mode

	__u8 m_UAVMode;

	//! Fail Safe Status

	__u16 m_FailSafeStatus;

	//! Magnotometer Heading

	float m_MagHeading;

	//! Airborne Timer

	float m_AirborneTimer;

	//! Avionics Timer

	float m_AvionicsTimer;	

	//! System Flags

	__u16 m_SystemFlags;

	//! Payload 1

	__u16 m_Payload1;

	//! Payload 2

	__u16 m_Payload2;
};

#endif
