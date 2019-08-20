#ifndef CommSerialLinux_H
#define CommSerialLinux_H

#include "dllsetup.h"
#include "TranslatePacket.h"

#include <vector>
#include <pthread.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>


//! Call back function pointers typedef
typedef void (*CallBackFuncPtr)(CCommPacket *, PacketType);

class DYNAMIC_LINK CCommSerialLinux
{

public:

//Public Member Functions
	CCommSerialLinux();
	virtual ~CCommSerialLinux();

	//! Opens the Comm Port
	BOOL Open(const char *port);
	
	//! Closes the Comm Port
	BOOL Close();

	//! Returns whether the comm port is currently open
	BOOL IsOpen(){ return m_IsSerialPortOpened; }

	//Receives a packet or partial from the previously opened Comm port
	BOOL Receive();

	BOOL SendRaw(CWriteRawPacket *SendPacket);

	//! Encodes a packet with the comm protocol and places the data into the memmory passed in.  Returns size of data filled
	__u32 EncodePkt(CWriteRawPacket *SendPacket, __u8 *Data, int MaxSize);

	//! Registers a call back function
	/** This function registers a call back function
	  * a long with the packet type that they are interested in receiving
	  * /param callfunc The callback function pointer which is called when a packet is received
	  */
	
	void RegisterCallBack(CallBackFuncPtr NewFuncPtr, PacketType RegisterType);

//Public Member Variables

private:

//Private Member Functions


//Private Member Variables

	//! Serial Port Open Flag
	BOOL	m_IsSerialPortOpened;

	//! Serial Port Device Handle
	int		m_SerialDevice;

	//! Pthread handle
	pthread_t m_ReadThreadHnd;

	//! Translation Class
	CTranslatePacket m_Translate;

	//Partial Data Packet
	PacketData m_PartialPkt;
	//Partial Start Time
	double m_PartialStartTime;

	//! Call Back function pointers

	CallBackFuncPtr m_CallBackFuncPtr[256];

	//! BOOL Flags to determine if function is registered
	
	BOOL m_CallBackFuncReg[256];

};


#endif




