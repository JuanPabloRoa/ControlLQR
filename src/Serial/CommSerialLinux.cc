#include "CommSerialLinux.h"
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h> 

//External threaded serial read function
void ReadSerial(void *arg)
{
	//assign the pointer
	CCommSerialLinux *SerialLinux = (CCommSerialLinux*)arg;

	while(1) //Loop forever
	{
		if(!SerialLinux->Receive()) break;		//have the serial class read in the data
	}
}

CCommSerialLinux::CCommSerialLinux()
{
	//Initialize everything
	m_PartialStartTime = 0;
	m_PartialPkt.clear();
	memset(m_CallBackFuncReg,0,sizeof(BOOL)*256);
	m_SerialDevice = -1;
	m_IsSerialPortOpened = FALSE;
}

CCommSerialLinux::~CCommSerialLinux()
{
	if(m_IsSerialPortOpened)
		close(m_SerialDevice);
}

BOOL CCommSerialLinux::Open(const char *port)
{
	int iFlag;
	struct termios newtio;

	if(m_IsSerialPortOpened && m_SerialDevice >= 0) return TRUE; //already opened

	//Open the serial port
	// Set up the serial connection to the autopilot
	m_SerialDevice = open(port, O_RDWR | O_NOCTTY);// | O_NONBLOCK);
	if (m_SerialDevice < 0)
	{
		perror("Serial port could not open\n"); 
		return FALSE;
	}
	else
		printf("Serial port opened with handle %d\n", m_SerialDevice);

	//Set the serial port settings such as 8 bit, read on, ignore parity errors, etc.
	tcgetattr(m_SerialDevice, &newtio);
	memset(newtio.c_cc,0,sizeof(newtio.c_cc));
	newtio.c_cc[VMIN] = 1;
	newtio.c_cflag = (CS8 | CLOCAL | CREAD);
	newtio.c_iflag = (IGNPAR | ICRNL);
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	cfsetispeed (&newtio, B115200);				//Set input baud
	cfsetospeed (&newtio, B115200);				//Set output baud
	tcflush(m_SerialDevice, TCIFLUSH);						//Flush all data so we start clean
	iFlag = tcsetattr(m_SerialDevice, TCSANOW, &newtio);	//Set the values

	if(iFlag != 0) 
	{ 
		printf("tcsetattr error %d\n",iFlag); 
		return FALSE;
	}

	m_IsSerialPortOpened = TRUE;

	// Create a thread for recieving from the autopilot
	pthread_create( &m_ReadThreadHnd, NULL, (void*(*)(void*))ReadSerial, (void*)this);

	return TRUE;
}

BOOL CCommSerialLinux::Close()
{
	if(m_IsSerialPortOpened)
		close(m_SerialDevice);

	m_IsSerialPortOpened = FALSE;
	return TRUE;
}


BOOL CCommSerialLinux::Receive()
{
	if(!m_IsSerialPortOpened)
		return FALSE; //Comm port isn't open


	u8 ReadBuff[1024];
	DWORD NumRead;

	//Read in as much as we can up to 1024
	NumRead = read(m_SerialDevice, ReadBuff,1024);
	if(NumRead <= 0) return FALSE; //some error

	//Initialize our Index
	DWORD Index = 0;
	DWORD Start = 0;

	while (Index < NumRead)
	{
		//Do we have a previous uncomplete packet?
		if (m_PartialPkt.size() == 0)
		{
			// look for potential packet start code (0xFF)
			while ((Index < NumRead) && (ReadBuff[Index] != (u8)0xFF))
				Index++;

			Start = Index;
//			m_PartialStartTime = timeGetTime();
		}

		while ((Index < NumRead) && (ReadBuff[Index] != (u8)0xFE))
		{
			if ((Index != Start) && (ReadBuff[Index] == 0xFF))
			{
				m_PartialPkt.clear();						//We received a FF before we got another FE so just start over
				Start = Index;
			}
			
			m_PartialPkt.push_back(ReadBuff[Index]); 
			Index++;
		}

		if ((Index < NumRead) && (ReadBuff[Index] == (u8)0xFE))
		{
			//Add the 0xFE
			m_PartialPkt.push_back((u8)0xFE);

			//move the Index to past the 0xFE
			Index++;

//			if (timeGetTime() - m_PartialStartTime > 250)
//			{
//				//return FALSE;
//			}
//			else if (m_PartialPkt.size() < 7)
//			{
//				//Clear out the partial and reset start
//				//to see if there are anymore messages in our read buffer
//				m_PartialPkt.clear();
//				Start = Index;
//			}
//			else
			if(m_PartialPkt.size() > 7)
			{
				//Packet is ready to be translated
				CCommPacket *NewPkt = 0;
				PacketType NewPktType = m_Translate.Translate(m_PartialPkt,&NewPkt);

				//Dispatch packet to any subscribed functions
				switch(NewPktType)
				{
				case UNKNOWN_PACKET:
					printf("Received Bad packet\n");
				break;

				case ACK_PACKET:

					if(m_CallBackFuncReg[ACK_PACKET] == TRUE)
						m_CallBackFuncPtr[ACK_PACKET](NewPkt, NewPktType);

				break;
				case NACK_PACKET:

					if(m_CallBackFuncReg[NACK_PACKET] == TRUE)
						m_CallBackFuncPtr[NACK_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_TC_PACKET:

					if(m_CallBackFuncReg[REQUEST_TC_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_TC_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_TC2_PACKET:

					if(m_CallBackFuncReg[REQUEST_TC2_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_TC2_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_SENSOR_PACKET:

					if(m_CallBackFuncReg[REQUEST_SENSOR_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_SENSOR_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_PYLD_AD_PACKET:

					if(m_CallBackFuncReg[REQUEST_PYLD_AD_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_PYLD_AD_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_MAG_PACKET:

					if(m_CallBackFuncReg[REQUEST_MAG_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_MAG_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_IO_CONFIG_PACKET:

					if(m_CallBackFuncReg[REQUEST_IO_CONFIG_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_IO_CONFIG_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_FLOAT_PACKET:

					if(m_CallBackFuncReg[REQUEST_FLOAT_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_FLOAT_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_INT_PACKET:

					if(m_CallBackFuncReg[REQUEST_INT_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_INT_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_BYTE_PACKET:

					if(m_CallBackFuncReg[REQUEST_BYTE_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_BYTE_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_FLASH_PACKET:

					if(m_CallBackFuncReg[REQUEST_FLASH_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_FLASH_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_CODE_VER_PACKET:

					if(m_CallBackFuncReg[REQUEST_CODE_VER_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_CODE_VER_PACKET](NewPkt, NewPktType);

				break;
				case CHECK_SENSORS:

					if(m_CallBackFuncReg[CHECK_SENSORS] == TRUE)
						m_CallBackFuncPtr[CHECK_SENSORS](NewPkt, NewPktType);

				break;
				case DOWNLOAD_WAYPOINT_PACKET:

					if(m_CallBackFuncReg[DOWNLOAD_WAYPOINT_PACKET] == TRUE)
						m_CallBackFuncPtr[DOWNLOAD_WAYPOINT_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_SERVO_PPM_PACKET:

					if(m_CallBackFuncReg[REQUEST_SERVO_PPM_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_SERVO_PPM_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_SERVO_RAD_PACKET:

					if(m_CallBackFuncReg[REQUEST_SERVO_RAD_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_SERVO_RAD_PACKET](NewPkt, NewPktType);

				break;
				case COMM_BOX_STATUS_PACKET:

					if(m_CallBackFuncReg[COMM_BOX_STATUS_PACKET] == TRUE)
						m_CallBackFuncPtr[COMM_BOX_STATUS_PACKET](NewPkt, NewPktType);

				break;
				case COMM_BOX_STICK_POS_PACKET:

					if(m_CallBackFuncReg[COMM_BOX_STICK_POS_PACKET] == TRUE)
						m_CallBackFuncPtr[COMM_BOX_STICK_POS_PACKET](NewPkt, NewPktType);

				break;
				case REQ_FAILSAFE_CONFIG_PACKET:

					if(m_CallBackFuncReg[REQ_FAILSAFE_CONFIG_PACKET] == TRUE)
						m_CallBackFuncPtr[REQ_FAILSAFE_CONFIG_PACKET](NewPkt, NewPktType);

				break;
				case REQUEST_DATALOG_PACKET:
					if(m_CallBackFuncReg[REQUEST_DATALOG_PACKET] == TRUE)
						m_CallBackFuncPtr[REQUEST_DATALOG_PACKET](NewPkt, NewPktType);

				break;
				case UPLOAD_PID_PACKET:

					if(m_CallBackFuncReg[UPLOAD_PID_PACKET] == TRUE)
						m_CallBackFuncPtr[UPLOAD_PID_PACKET](NewPkt, NewPktType);

				break;
				case RECEIVE_TUNING_PACKET:

					if(m_CallBackFuncReg[RECEIVE_TUNING_PACKET] == TRUE)
						m_CallBackFuncPtr[RECEIVE_TUNING_PACKET](NewPkt, NewPktType);


				break;
				case GPS_TELEMETRY_PACKET:

					if(m_CallBackFuncReg[GPS_TELEMETRY_PACKET] == TRUE)  //Make sure they have registered for this function
						m_CallBackFuncPtr[GPS_TELEMETRY_PACKET](NewPkt, NewPktType); //Call the registered function

	
				break;
				case STD_TELEMETRY_PACKET:

					if(m_CallBackFuncReg[STD_TELEMETRY_PACKET] == TRUE)				//Make sure they have registered for this function
						m_CallBackFuncPtr[STD_TELEMETRY_PACKET](NewPkt,NewPktType); //Call the registered function
					
				break;
				default:
					break;					
				}

				//Send to the one and only handler if they subscribed to it
				if(m_CallBackFuncReg[ALL_PACKET] && NewPktType != UNKNOWN_PACKET)	//Make sure they have registered for this function
						m_CallBackFuncPtr[ALL_PACKET](NewPkt,NewPktType);			//Call the registered function

				//Delete the finished packet
				if(NewPkt) delete NewPkt;
			}

			//Clear out the partial and reset start
			//to see if there are anymore messages in our read buffer
			m_PartialPkt.clear();
			Start = Index;
		}
	}

	return TRUE;
}

void CCommSerialLinux::RegisterCallBack(CallBackFuncPtr NewFuncPtr, PacketType RegisterType)
{
	if(RegisterType < 0 || RegisterType > 255) return;

	m_CallBackFuncPtr[RegisterType] = NewFuncPtr;
	m_CallBackFuncReg[RegisterType] = TRUE;
}

BOOL CCommSerialLinux::SendRaw(CWriteRawPacket *SendPacket)
{
	if(!m_IsSerialPortOpened) return FALSE;  //Port not open
	if(SendPacket == NULL || SendPacket->GetData()->size() == 0) return FALSE; //There is nothing to send
	
	PacketData SendData;
	m_Translate.Encode(*SendPacket, &SendData);

	u8 *CharData = new u8[SendData.size()];
	
	//Copy data into byte array
	for(u32 i=0; i < SendData.size(); i++)
		CharData[i] = SendData[i];

	if(write(m_SerialDevice, CharData, SendData.size()) == -1)
	{
		//Something wrong with the comm port...close it
		delete [] CharData;
		return FALSE;
	}

	delete [] CharData;
	return TRUE;
}

u32 CCommSerialLinux::EncodePkt(CWriteRawPacket *SendPacket, u8 *Data, int MaxSize)
{
	if(SendPacket == NULL || SendPacket->GetData()->size() == 0) return 0; //There is nothing to send
	
	PacketData SendData;
	m_Translate.Encode(*SendPacket, &SendData);

	//Copy data into byte array
	for(u32 i=0; i < SendData.size() && i < (u32)MaxSize; i++)
		Data[i] = SendData[i];

	return SendData.size();
}
