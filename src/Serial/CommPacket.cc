#include "CommPacket.h"
#include <string.h>
void *memcpy(void *dest, const void *src, size_t n);
CCommPacket::CCommPacket()
{
	Clear();  //Default Clear
}

CCommPacket::~CCommPacket()
{
	Clear(); //Just for fun clear out the data
}


CCommPacket::CCommPacket(CCommPacket *copy)
{
	//Copy Address
	m_SrcAddress = copy->m_SrcAddress;
	m_DestAddress = copy->m_DestAddress;

	//Flags
	m_bSrcAddRead = copy->m_bSrcAddRead;
	m_bDestAddRead = copy->m_bDestAddRead;

	//Copy Data vector
	m_Data = copy->m_Data;

	//Packet ID
	m_PacketIDNum = copy->m_PacketIDNum;
}

const CCommPacket & CCommPacket::operator =(const CCommPacket &right)
{
	if(&right != this)
	{
		//Copy Address
		m_SrcAddress = right.m_SrcAddress;
		m_DestAddress = right.m_DestAddress;

		//Flags
		m_bSrcAddRead = right.m_bSrcAddRead;
		m_bDestAddRead = right.m_bDestAddRead;

		//Copy Packet ID
		m_PacketIDNum = right.m_PacketIDNum;

		//Don't copy over the data vector
	}

	return *this;
}

void CCommPacket::Clear()
{
	//Clear Addresses
	m_SrcAddress = m_DestAddress = 0;

	//Clear Packet ID
	m_PacketIDNum = 0;

	//Clear Flags
	m_bSrcAddRead = FALSE;
	m_bDestAddRead = FALSE;

	//Clear Data vector
	m_Data.clear();
}

void CCommPacket::Fill(PacketData NewData)
{
	m_Data = NewData;

	//Read Src and Destination Address
	GetSrcAddress();
	GetDestAddress();

	if(m_Data.size() > 4)
		m_PacketIDNum = ReadUnsignedChar(m_Data.size() - 4);
}

u16 CCommPacket::GetSrcAddress()
{
	if(m_bSrcAddRead == FALSE)
	{
		m_SrcAddress = ReadUnsignedShort(2);
		m_bSrcAddRead = TRUE;
	}

	return m_SrcAddress;
}

u16 CCommPacket::GetDestAddress()
{
	if(m_bDestAddRead == FALSE)
	{
		m_DestAddress = ReadUnsignedShort(4);
		m_bDestAddRead = TRUE;
	}

	return m_DestAddress;
}

u8 CCommPacket::GetPacketIDNum()
{
	return m_PacketIDNum;
}

u8 CCommPacket::ReadUnsignedChar(u32 i) const
{
	if(i >= m_Data.size()) return 0;

	return m_Data[i];
}

u16 CCommPacket::ReadUnsignedShort(u32 i) const
{
	if(i + 1 >= m_Data.size()) return 0;

	u16 val;
	
	val = m_Data[i];
	val |= m_Data[i + 1] << 8;

	return val;
}

s16 CCommPacket::ReadShort(u32 i) const
{
	if(i + 1 >= m_Data.size()) return 0;

	s16 val;
	
	val = m_Data[i];
	val |= m_Data[i + 1] << 8;

	return val;
}

s32 CCommPacket::ReadInt(u32 i) const
{
	if(i + 3 >= m_Data.size()) return 0;

	s32 val;
	
	val = m_Data[i];
	val |= m_Data[i + 1] << 8;
	val |= m_Data[i + 2] << 16;
	val |= m_Data[i + 3] << 24;

	return val;
}

u32 CCommPacket::ReadUnsignedInt(u32 i) const
{
	if(i + 3 >= m_Data.size()) return 0;

	u32 val;
	
	val = m_Data[i];
	val |= m_Data[i + 1] << 8;
	val |= m_Data[i + 2] << 16;
	val |= m_Data[i + 3] << 24;

	return val;
}


float CCommPacket::ReadFloat(u32 i) const
{
	if(i + 3 >= m_Data.size()) return 0;

	u32 iVal;
	float fVal;
	
	iVal = m_Data[i];
	iVal |= m_Data[i + 1] << 8;
	iVal |= m_Data[i + 2] << 16;
	iVal |= m_Data[i + 3] << 24;

	memcpy(&fVal, &iVal, 4);

	return fVal;
}

PacketType CCommPacket::GetType(void)
{ 
	//Make sure the packet type has been filled in
	if(m_Data.size() >= 2)
		return (PacketType)m_Data[1];
	else
		return UNKNOWN_PACKET;
}
