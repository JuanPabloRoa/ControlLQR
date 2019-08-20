#ifndef COMM_PACKET_H
#define COMM_PACKET_H


#include "dllsetup.h"
#include "type.h"
#include <vector>
#include <linux/types.h> 
#include <asm/types.h>
#include <sys/types.h>

//#ifndef LINUX
	//EXTERN_LINK template class DYNAMIC_LINK std::vector<__u8>;
//#endif

#define DYNAMIC_LINK

//! Packet Data Type defines for easy writing of code.
typedef std::vector<__u8> PacketData;
typedef std::vector<__u8>::iterator PacketDataIter;

//! Packet Type Enumeration

/** Packet Type enumeration values.
 *  These values are used to check the packet types
	*/
enum PacketType
{
	UNKNOWN_PACKET = -1,						///< Packet is Unknown.
	ALL_PACKET = 0,								///< All packet types.
	ACK_PACKET = 1,
	NACK_PACKET = 2,
	REQUEST_TC_PACKET = 3,
	REQUEST_TC2_PACKET = 4,
	REQUEST_SENSOR_PACKET = 6,
	REQUEST_PYLD_AD_PACKET = 7,
	REQUEST_MAG_PACKET = 8,
	UPLOAD_IO_CONFIG_PACKET = 9,
	SET_PID_PACKET = 10,
	UPLOAD_FLOAT_PACKET = 12,
	UPLOAD_INT_PACKET = 13,
	UPLOAD_BYTE_PACKET = 14,
	UPLOAD_FLASH_PACKET = 15,
	REQUEST_IO_CONFIG_PACKET = 19,
	REQUEST_FLOAT_PACKET = 22,
	REQUEST_INT_PACKET = 23,
	REQUEST_BYTE_PACKET = 24,
	REQUEST_FLASH_PACKET = 25,
	STD_TELEMETRY_REQUEST_PACKET = 26,
	GPS_TELEMETRY_REQUEST_PACKET = 27,
	REQUEST_CODE_VER_PACKET = 28,
	UPLOAD_FLC = 30,
	CHECK_SENSORS = 33,
	RECALIBRATE_SENSORS = 34,
	UPLOAD_TRIMS = 40,
	ZERO_STICKS = 41,
	ZERO_THROTTLE = 42,
	SET_GPS_HOME_PACKET = 43,
	SEND_JOYSTICKS_2 = 46,
	GIMBAL_JOYSTICK_PACKET = 47,
	ALT_SPEED_OVERRIDE_PACKET = 48,
	UPLOAD_COMMAND_PACKET = 50,
	DOWNLOAD_WAYPOINT_PACKET = 51,
	SET_CURRENT_COMMAND_PACKET = 52,
	EDIT_COMMAND_PACKET = 53,
	SET_AUTOPILOT_MODE = 55,
	REQUEST_SERVO_PPM_PACKET = 60,
	REQUEST_SERVO_RAD_PACKET = 61,
	SEND_SERVO_RAD_PACKET = 63,
	SERVO_DISCONNECT_PACKET = 64,
	WRITE_FLASH_PACKET = 70,
	COMM_BOX_STATUS_PACKET = 110,
	RC_PILOT_ADDRESS_PACKET = 111,
	COMM_BOX_STICK_POS_PACKET = 112,
	SET_FAILSAFE_CONFIG_PACKET = 190,
	REQ_FAILSAFE_CONFIG_PACKET = 191,
	REQUEST_DATALOG_PACKET = 201,
	DATALOG_START_PACKET = 202,
	SET_DESIRED_ALL_PACKET = 230,
	SET_DESIRED_UNIVERSAL_PACKET = 231,
	REQUEST_PID_PACKET = 237,
	UPLOAD_PID_PACKET = 238,
	REQUEST_TUNING_PACKET = 239,
	RECEIVE_TUNING_PACKET = 240,
	GPS_TELEMETRY_PACKET = 248,
	STD_TELEMETRY_PACKET = 249,
	DIGITAL_OUTPUT_TOGGLE_PACKET = 250,
};

#define DEFAULT_SEND_SRC_ADDR 0
#define DEFAULT_SEND_DEST_ADDR 1032

#define DEFAULT_RECV_SRC_ADDR 1032
#define DEFAULT_RECV_DEST_ADDR 0

#define BROADCAST_ADDRESS 0xFFFF


//! The CommPacket Class.
/** This class is a base class
  * for all the different CommPacket types.
  * It provides basic functions that are used in all
  * derived classes.
  */

class DYNAMIC_LINK CCommPacket
{
	friend class CTranslatePacket;
	
public:
	
	// ------------------------------------------------------------------------------
	//														  Public Member Functions
	// ------------------------------------------------------------------------------

	//! Class Constructor
	/** Simple Class Constructor.
	  * Clears out the member variables
	  */

	CCommPacket();

	//! Class Destructor
	/** Frees any memory allocated
	  */

	virtual ~CCommPacket();
	
	//! Copy Constructor
	/**
		\param copy a copy of a CCommPacket
		\return nothing
	*/
	
	CCommPacket(CCommPacket *copy);	

	//! Assignment Operator

	const CCommPacket & operator =(const CCommPacket &right);
	
	//! Gets the packet type
	/**
	  * Returns the type of packet that has been created for 
	  * this object.
		\return Returns a PacketType enum for the object
	  */

	PacketType GetType(void);

	//! The size of the Data
	/** 
	  * Called when one wants to know the size of 
	  * the packet's data variable.
	  * \return Size of the packet
	  */

	unsigned long Size() { return m_Data.size(); }
	
	//! Clears the packet
	/** 
	  * Clears out all member variables
	  */

	virtual void Clear(); 

	//! Fills packet members
	/**
	  * This function is called with new Raw Data which is then parsed by the packet
	  * and stored in the corresponding member varibles
	  */

	void Fill(PacketData NewData);

	//! Get Source Address
	/** 
	  * Returns the Source Address of the packet
	  * /return Source Address read from the raw data.
	  */

	__u16 GetSrcAddress();

	//! Get Destination Address
	/** 
	  * Returns the Destination Address of the packet
	  * /return Destination Address read from the raw data.
	  */

	__u16 GetDestAddress();

	//! Returns the packet ID number
	
	__u8 GetPacketIDNum();

	//! Read an u8 from the data

	__u8 ReadUnsignedChar(__u32 i) const;

	//! Read an u16 from the data

	__u16 ReadUnsignedShort(__u32 i) const;

	//! Read a short from the specified location i

	__s16 ReadShort(__u32 i) const;

	//! Read an int from the specified location i

	__s32 ReadInt(__u32 i) const;

	//! Read an unsigned int from the specified location i

	__u32 ReadUnsignedInt(__u32 i) const;

	//! Read a float from the specified location i

	float ReadFloat(__u32 i) const;

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

	//! Source Address of the Packet

	__u16 m_SrcAddress;

	//! Destination Address of the Packet

	__u16 m_DestAddress;

	//! Source Read Indicator Flag
	/** Tells the private functions whether the 
	  * source address has already been read
	  */

	BOOL m_bSrcAddRead;

	//! Destination Read Indicator Flag
	/** Tells the private functions whether the 
	  * destination address has already been read
	  */

	BOOL m_bDestAddRead;

	//! Packet ID Number
	/** Packet ID Number used to correlate between 
	* packets requested and sent from the autopilot
	*/

	__u8 m_PacketIDNum;

	//! Packet Raw Data

	PacketData m_Data;

}; //End Class CCommPacket

#endif
