#include "WriteRawPacket.h"
#include <string.h>
void *memcpy(void *dest, const void *src, size_t n);
CWriteRawPacket::CWriteRawPacket()
{
}
	

CWriteRawPacket::CWriteRawPacket(PacketType Type, u16 DestAddr, u16 SrcAddr, u8 PacketIDNum)
{
	u8 nType = (u8)Type;

	//Create a packet with the basic info given
	m_Data.push_back((u8)0xFF); //Start Header
	m_Data.push_back(nType);    //Packet Type
	m_Data.push_back((u8)(SrcAddr & 0x00FF));  //Add Src Addr Virtual Cockpit Default
	m_Data.push_back((u8)((SrcAddr & 0xFF00) >> 8));
	m_Data.push_back((u8)(DestAddr & 0x00FF));  //Add Dest Addr 1032 Default
	m_Data.push_back((u8)((DestAddr & 0xFF00) >> 8));	

	m_SrcAddress = SrcAddr;
	m_DestAddress = DestAddr;

	m_PacketIDNum = PacketIDNum;
}

CWriteRawPacket::CWriteRawPacket(CWriteRawPacket *copy)
{
	//Call Base Class Copy Constructor
	CCommPacket((CCommPacket *)copy);

}

const CWriteRawPacket &CWriteRawPacket::operator =(const CWriteRawPacket &right)
{
	//check for self assignment
	if(&right != this)
		this->CCommPacket::operator =(right);	//call base class

	return *this;
}

CWriteRawPacket::~CWriteRawPacket()
{
	CCommPacket::Clear();	
}


void CWriteRawPacket::AppendUnsignedChar(u8 b)
{
	m_Data.push_back(b);
}

void CWriteRawPacket::AppendUnsignedShort(u16 us)
{
	AppendUnsignedChar(us & 0xFF);
	AppendUnsignedChar((us & 0xFF00) >> 8);
}

void CWriteRawPacket::AppendShort(s16 s)
{
	AppendUnsignedChar(s & 0xFF);
	AppendUnsignedChar((s & 0xFF00) >> 8);
}

void CWriteRawPacket::AppendUnsignedInt(u32 i)
{
	AppendUnsignedChar(i & 0xFF);
	AppendUnsignedChar((i & 0xFF00) >> 8);
	AppendUnsignedChar((i & 0xFF0000) >> 16);
	AppendUnsignedChar((i & 0xFF000000) >> 24);
}

void CWriteRawPacket::AppendFloat(float f)
{
	u32 i;

	memcpy(&i, &f, 4);

	AppendUnsignedChar(i & 0xFF);
	AppendUnsignedChar((i & 0xFF00) >> 8);
	AppendUnsignedChar((i & 0xFF0000) >> 16);
	AppendUnsignedChar((i & 0xFF000000) >> 24);
}

void CWriteRawPacket::SetPacketIDNum(u8 ID)
{
	m_PacketIDNum = ID;
	AppendUnsignedChar(m_PacketIDNum);
}
