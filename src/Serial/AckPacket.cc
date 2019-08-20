#include "AckPacket.h"

CAckPacket::CAckPacket(AckType Type, u16 DestAddress, u16 SrcAddress, u8 PacketIDNum) : CWriteRawPacket(ACK_PACKET, DestAddress, SrcAddress, PacketIDNum)
{
	AppendUnsignedChar(Type);
	AppendUnsignedChar(PacketIDNum);
	AppendUnsignedChar(0); //zero checksum
	AppendUnsignedChar(0); //zero checksum
	AppendUnsignedChar(0xFE); //end byte
}

void CAckPacket::SetAsNackPkt()
{
	m_Data[1] = NACK_PACKET;
}
