#include "TranslatePacket.h"

CTranslatePacket::CTranslatePacket()
{

}

CTranslatePacket::~CTranslatePacket()
{

}

PacketType CTranslatePacket::Translate(PacketData NewData, CCommPacket **NewPacket)
{
	//First thing to do is remove special characters
	//Do so by copying current vector into new one

	PacketDataIter DataIter;
	PacketData StrippedData;

	DataIter = NewData.begin();
	while(DataIter != NewData.end())
	{	
		if(*DataIter == 0xFD)
		{
			DataIter++; //increment 1 past
			switch(*DataIter)
			{

			case 0:
				StrippedData.push_back((u8)0xFD);
				break;
			case 1:
				StrippedData.push_back((u8)0xFE);
				break;
			case 2:
				StrippedData.push_back((u8)0xFF);
				break;
			case 3:
				StrippedData.push_back((u8)0xCC);
				break;
			default:
				return UNKNOWN_PACKET;

			}
			DataIter++;  //increment past the extra char
			continue;    //skip back up to the while
		}

		StrippedData.push_back(*DataIter);
		DataIter++;
	}

	//Now that we are finished run the XOR check
	if(XORCheck(StrippedData) == FALSE) return UNKNOWN_PACKET;

	//Now we can Fill that Data into a Address Packet
	CCommPacket AddressPacket;
	AddressPacket.Fill(StrippedData);

	CStdTelemPacket *NewTelemPacket;
	CGPSTelemPacket *NewGPSPacket;
	CAckPacket *NewAckPacket;
	CCommPacket *NewCommPacket;

	//Switch on the Packet Type
	switch(AddressPacket.GetType())
	{
	case ACK_PACKET:
		
		NewAckPacket = new CAckPacket((AckType)AddressPacket.m_Data[6],AddressPacket.m_DestAddress,AddressPacket.m_SrcAddress, AddressPacket.m_PacketIDNum);
		*NewPacket = static_cast<CAckPacket *>(NewAckPacket);
		return ACK_PACKET;

	break;
	case NACK_PACKET:

		NewAckPacket = new CAckPacket((AckType)AddressPacket.m_Data[6],AddressPacket.m_DestAddress,AddressPacket.m_SrcAddress, AddressPacket.m_PacketIDNum);
		NewAckPacket->SetAsNackPkt();
		*NewPacket = static_cast<CAckPacket *>(NewAckPacket);
		return NACK_PACKET;

	break;
	case REQUEST_TC_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_TC_PACKET;

	break;
	case REQUEST_TC2_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_TC2_PACKET;

	break;
	case REQUEST_SENSOR_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_SENSOR_PACKET;

	break;
	case REQUEST_PYLD_AD_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_PYLD_AD_PACKET;

	break;
	case REQUEST_MAG_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_MAG_PACKET;

	break;
	case REQUEST_IO_CONFIG_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_IO_CONFIG_PACKET;

	break;
	case REQUEST_FLOAT_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_FLOAT_PACKET;

	break;
	case REQUEST_INT_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_INT_PACKET;

	break;
	case REQUEST_BYTE_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_BYTE_PACKET;

	break;
	case REQUEST_FLASH_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_FLASH_PACKET;

	break;
	case REQUEST_CODE_VER_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_CODE_VER_PACKET;

	break;
	case CHECK_SENSORS:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return CHECK_SENSORS;

	break;
	case DOWNLOAD_WAYPOINT_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return DOWNLOAD_WAYPOINT_PACKET;

	break;
	case REQUEST_SERVO_RAD_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_SERVO_RAD_PACKET;

	break;
	case REQUEST_SERVO_PPM_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_SERVO_PPM_PACKET;

	break;
	case REQ_FAILSAFE_CONFIG_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQ_FAILSAFE_CONFIG_PACKET;

	case REQUEST_DATALOG_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_DATALOG_PACKET;

	break;
	case REQUEST_PID_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return REQUEST_PID_PACKET;

	break;
	case RECEIVE_TUNING_PACKET:

		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return RECEIVE_TUNING_PACKET;

	break;
	case GPS_TELEMETRY_PACKET:

		if(StrippedData.size() != 81) return UNKNOWN_PACKET; //make sure size is correct

		NewGPSPacket = new CGPSTelemPacket();
		NewGPSPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewGPSPacket);
		return GPS_TELEMETRY_PACKET;

	break;
	case STD_TELEMETRY_PACKET:

		if(StrippedData.size() != 64) return UNKNOWN_PACKET; //make sure size is correct

		NewTelemPacket = new CStdTelemPacket();
		NewTelemPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewTelemPacket);
		return STD_TELEMETRY_PACKET;

	break;
	default:
		//Defaults could come from the dev program
		NewCommPacket = new CCommPacket();
		NewCommPacket->Fill(StrippedData);
		*NewPacket = static_cast<CCommPacket *>(NewCommPacket);
		return AddressPacket.GetType();

	break;
	}

	//Don't know it
	return UNKNOWN_PACKET;
}

BOOL CTranslatePacket::XORCheck(PacketData TestData)
{
	u8 checksum1 = 0;
	u8 checksum2 = 0;

	for (unsigned int i=1; i<TestData.size()-4; i+=2)
	{
		checksum1 ^= TestData[i];
		checksum2 ^= TestData[i+1];
	}

	if ((TestData.size() - 4) % 2)
		checksum1 ^= TestData[TestData.size() - 4];

	return (checksum1 == TestData[TestData.size() - 3]) && (checksum2 == TestData[TestData.size() - 2]);
}

void CTranslatePacket::Encode(CCommPacket EncodedPacket, PacketData *SendData)
{
	//Now that m_Data is filled up correctly determine the Check Value
	u16 CheckValue;
	XORGenerate(EncodedPacket.m_Data,&CheckValue);
	//Fill up the checksum
	EncodedPacket.m_Data.push_back((u8)((CheckValue & 0xFF00) >> 8));
	EncodedPacket.m_Data.push_back((u8)(CheckValue & 0xFF));


	//Now encode SendData removing all special characters
	PacketDataIter PacketIter = EncodedPacket.m_Data.begin();
	
	PacketIter++; //Step 1 past the initial 0xFF
	SendData->push_back((u8)0xFF);

	while(PacketIter != EncodedPacket.m_Data.end())
	{
		if ((*PacketIter == 0xCC) || (*PacketIter == 0xFD) || (*PacketIter == 0xFE) || (*PacketIter == 0xFF))
		{
			SendData->push_back((u8)0xFD);
			if (*PacketIter == 0xCC)
				SendData->push_back((u8)3);
			else
				SendData->push_back((u8)(*PacketIter - 0xFD));
		}
		else
			SendData->push_back(*PacketIter);

		PacketIter++;
	}

	//End with an FE
	EncodedPacket.m_Data.push_back((u8)0xFE);
	SendData->push_back((u8)0xFE);
	int val = SendData->size();
}

void CTranslatePacket::XORGenerate(PacketData GenData, u16 *CheckValue)
{
	*CheckValue = 0; //Clear it out

	for (u32 i=1; i<GenData.size()-1; i += 2)
	{
		*CheckValue ^= GenData[i] << 8;
		*CheckValue ^= GenData[i+1];
	}

	if((GenData.size() - 1)%2)
		*CheckValue ^= GenData[GenData.size()-1] << 8;
}
