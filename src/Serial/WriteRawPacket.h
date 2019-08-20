#ifndef WRITERAW_PACKET_H
#define WRITERAW_PACKET_H

//! The Write Raw Packet Class.
/** This class is a simple data
  * only class for sending raw data to the autopilot
  * Users should follow the UAV communications protocol
  * when assembling this packet.
  */
#include "CommPacket.h"

class DYNAMIC_LINK CWriteRawPacket : public CCommPacket
{
	friend class CTranslatePacket;
	
	// ------------------------------------------------------------------------------
	//														  Public Member Functions
	// ------------------------------------------------------------------------------


public:

	//! Default Settings Constructor

	CWriteRawPacket();

	//! Specific Write Packet Constructor
	/** Takes input parameters to setup certain areas of the write packet
	  * \param Type The packet type to be sent.
	  * \param DestAddr The Destination Address in a multi UAV system
	  * \param SrcAddr Where this packet is coming from.  Default is 0
	  */

	CWriteRawPacket(PacketType Type, __u16 DestAddr = DEFAULT_SEND_DEST_ADDR, __u16 SrcAddr = DEFAULT_SEND_SRC_ADDR, __u8 PacketIDNum = 0);

	//! Copy constructor

	CWriteRawPacket(CWriteRawPacket *copy);

	//! Assignment operator

	const CWriteRawPacket &operator =(const CWriteRawPacket &right);

	//! Destructor
	
	~CWriteRawPacket();

	//! Returns the Packet Data
	PacketData * GetData() { return &m_Data;}

	//! Encode the byte and append it

	void AppendUnsignedChar(__u8 b);

	//! Encode the unsigned short and append it

	void AppendUnsignedShort(__u16 us);

	//! Encode the short and append it

	void AppendShort(__s16 s);

	//! Encode the long and append it

	void AppendUnsignedInt(__u32 s);

	//! Encode the float and append it
	
	void AppendFloat(float f);

	//! Sets the packet ID number
	/** This needs to be the last thing done
	  * cause it will append it onto the end
	  * of the packet
	  */

	void SetPacketIDNum(__u8 ID);
};

	
#endif
