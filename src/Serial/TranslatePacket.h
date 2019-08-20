#ifndef TRANSLATE_PACKET_H
#define TRANSLATE_PACKET_H

#include "CommPacket.h"
#include "StdTelemPacket.h"
#include "GPSTelemPacket.h"
#include "WriteRawPacket.h"
#include "AckPacket.h"

class DYNAMIC_LINK CTranslatePacket
{
public:

	//Public Member Functions

	//! Constructor

	CTranslatePacket();

	//! Destructor

	virtual ~CTranslatePacket();

	//! Translate a unsigned char vector into the appropriate Packet type
	
	PacketType Translate(PacketData NewData, CCommPacket **NewPacket);

	//! Encodes a Comm Packet Type
	/** Function uses the member variables to fill up a Comm Packet that needs to be sent
	  * to the autopilot
	  */
	
	void Encode(CCommPacket EncodedPacket, PacketData *SendData);

private:

	BOOL XORCheck(PacketData TestData);
	void XORGenerate(PacketData GenData, __u16 *CheckValue);

};
#endif
