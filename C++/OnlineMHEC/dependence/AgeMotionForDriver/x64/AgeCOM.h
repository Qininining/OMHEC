/*

Copyright (C) 2001-2024 Hangzhou AutomaticAge Co., Ltd.

Module Name:

	AgeCOM.h

Abstract:

	Defines Data Types and Declarations for AgeCOM.DLL.

Author:

	AutomaticAge  2024.08.29

Revision History:

	2015.07.23    Version: 1.00.32.00		First release version, support 32Bit WindowsXP/7

	2016.05.17    Version: 1.00.35.00		Support plug and play

	2017.11.16    Version: 1.00.51.00		Support 32Bit WindowsXP/7/8/10 and 64Bit Windows7/8/10

	2018.12.19    Version: 1.00.81.00		Support 500Kbps, 1Mbps, 2Mbps, 3Mbps
																			Add AgeCOMReadMWORD/AgeCOMWriteMWORD function, Maximum 64 WORDs
																			Modify AgeCOMGetBusInfo
																			Improve Modbus-RTU operation

	2019.12.19    Version: 1.00.86.00		Maximum free authority RTU address is modified from 1 to 4

	2021.09.15    Version: 1.00.89.00		Support new chips

	2023.11.11    Version: 1.00.91.00		Maximum free authority RTU address is modified from 4 to 16

	2023.12.28    Version: 1.00.95.00		AgeCOMReadMWORD support 125WORD, AgeCOMWriteMWORD support 123WORD

	2024.03.09    Version: 1.01.03.00		Inner optimization

	2024.05.26    Version: 1.01.04.00		Support AVC4XXX chips

	2024.08.23    Version: 1.01.05.00		Add FnAgeCOMGetCOM, FnAgeCOMSetCOM

	2024.08.26    Version: 1.01.07.00		Add AgeCOMGetUSBID

	2024.08.29    Version: 1.01.08.00		Inner optimization

	2024.09.07    Version: 1.01.09.00		Baudrate support up to 10Mbps
*/


#pragma once

#ifndef BOOL32
	typedef long BOOL32;	// FALSE: 0, TRUE: 1
#endif

#ifndef LONGLONG
	typedef long long LONGLONG;
#endif

#ifndef BYTE
	typedef unsigned char BYTE;
#endif

#ifndef WORD
	typedef unsigned short WORD;
#endif

#ifndef DWORD
	typedef unsigned long DWORD;
#endif

#ifndef QWORD
	typedef unsigned long long QWORD;
#endif

// If serial number is error, can visit Address 1 or 0(broadcast) only 
// pucSerial: Serial number, dwLength: Serial number length
// Will auto connect if no connect or invalid
__declspec(dllimport) BOOL32 AgeCOMSerial(BYTE* pucSerial, DWORD dwLength);

// bAutoConnect: TRUE - Auto connect if no connect or invalid, FALSE - Do none
__declspec(dllimport) BOOL32 AgeCOMIsValid(BOOL32 bAutoConnect);

__declspec(dllimport) BOOL32 AgeCOMGetCOMID(WORD& wCOMID);		// Get COM ID, such as COM1 is 1
__declspec(dllimport) BOOL32 AgeCOMGetUSBID(BYTE* pucUSBID);	// Get USB ID, such as (0x)2376E1712833, 12char

// dwBaudRate: 9600, 19200, 38400, 76800, 115200, 250K, 500K, 1M, 2M, 3M, others are illegal. Default is 115200.
// wParity   : 0 None, 1 Odd, 2 Even. Default is even parity.
// Will auto connect if no connect or invalid
__declspec(dllimport) BOOL32 AgeCOMGetCOM(DWORD& dwBaudRate, WORD& wParity);
__declspec(dllimport) BOOL32 AgeCOMSetCOM(DWORD dwBaudRate, WORD wParity);

__declspec(dllimport) BOOL32 AgeCOMGetBaudRate(DWORD& dwBaudRate);
__declspec(dllimport) BOOL32 AgeCOMSetBaudRate(DWORD dwBaudRate);

__declspec(dllimport) BOOL32 AgeCOMGetParity(WORD& wParity);
__declspec(dllimport) BOOL32 AgeCOMSetParity(WORD wParity);

__declspec(dllimport) BOOL32 AgeCOMGetBusInfo(LONGLONG& llHostRunTime,		// Host run time from dll run, ms
																							LONGLONG& llBusRunTime,			// Bus run time of Tx and Rx, ms
																							LONGLONG& llLastOpTime,			// Last operation time, ms
																							LONGLONG& llMaxOpTime,			// Max operation time, ms
																							LONGLONG& llMinOpTime,			// Min operation time, ms
																							LONGLONG& llBusOpCounts,		// Bus operation counts
																							LONGLONG& llTxFrames,				// Transmit frame counts
																							LONGLONG& llRxFrames,				// Receive frame counts
																							LONGLONG& llTxBytes,				// Transmit byte counts
																							LONGLONG& llRxBytes,				// Receive byte counts
																							LONGLONG& llHostErrors,			// Host hardware and software error counts 
																							LONGLONG& llBusOpErrors,		// Bus operation error counts
																							LONGLONG& llTxFrameErrors,	// Transmit frame error counts
																							LONGLONG& llRxFrameErrors);	// Receive frame error counts

// ucRTUAddr:	0			: Broadcast address, can't read
//						1~247	: RTU address
//						others: Reserved
//
// wRegAddr : See RTU datasheet
//
// dwTimeout: wait mS counts for RTU response after transmition.
//						If dwTimeout = -1 or 0xFFFFFFFF, then function will not read RTU response;
//						If dwTimeout = 0, then function will calculate dwTimeout(mS) for RTU response automatically;
//						Else, function will wait RTU response for dwTimeout(mS).
//
// pwData		: WORDs buffer
//
// wLength	: WORDs of pwData pointed buffer, write <= 123 WORDs, read <= 125 WORDs. If wLength > maximum WORDs, will return FALSE.
//
// Will auto connect if no connect or invalid
__declspec(dllimport) BOOL32 AgeCOMReadWORD(BYTE ucRTUAddr, WORD wRegAddr, WORD& wData, DWORD dwTimeout);
__declspec(dllimport) BOOL32 AgeCOMReadDWORD(BYTE ucRTUAddr, WORD wRegAddr, DWORD& dwData, DWORD dwTimeout);
__declspec(dllimport) BOOL32 AgeCOMReadQWORD(BYTE ucRTUAddr, WORD wRegAddr, QWORD& qwData, DWORD dwTimeout);
__declspec(dllimport) BOOL32 AgeCOMReadMWORD(BYTE ucRTUAddr, WORD wRegAddr, WORD* pwData, WORD wLength, DWORD dwTimeout);

__declspec(dllimport) BOOL32 AgeCOMWriteWORD(BYTE ucRTUAddr, WORD wRegAddr, WORD wData, DWORD dwTimeout);
__declspec(dllimport) BOOL32 AgeCOMWriteDWORD(BYTE ucRTUAddr, WORD wRegAddr, DWORD dwData, DWORD dwTimeout);
__declspec(dllimport) BOOL32 AgeCOMWriteQWORD(BYTE ucRTUAddr, WORD wRegAddr, QWORD qwData, DWORD dwTimeout);
__declspec(dllimport) BOOL32 AgeCOMWriteMWORD(BYTE ucRTUAddr, WORD wRegAddr, WORD* pwData, WORD wLength, DWORD dwTimeout);

typedef BOOL32 (FnAgeCOMSerial)(BYTE* pucSerial, DWORD dwLength);

typedef BOOL32 (FnAgeCOMIsValid)(BOOL32 bAutoConnect);

typedef BOOL32 (FnAgeCOMGetCOMID)(WORD& wCOMID);
typedef BOOL32 (FnAgeCOMGetUSBID)(BYTE* pucUSBID);

typedef BOOL32 (FnAgeCOMGetCOM)(DWORD& dwBaudRate, WORD& wParity);
typedef BOOL32 (FnAgeCOMSetCOM)(DWORD dwBaudRate, WORD wParity);

typedef BOOL32 (FnAgeCOMGetBaudRate)(DWORD& dwBaudRate);
typedef BOOL32 (FnAgeCOMSetBaudRate)(DWORD dwBaudRate);

typedef BOOL32 (FnAgeCOMGetParity)(WORD& wParity);
typedef BOOL32 (FnAgeCOMSetParity)(WORD wParity);

typedef BOOL32 (FnAgeCOMGetBusInfo)(LONGLONG& llHostRunTime,
																		LONGLONG& llBusRunTime,
																		LONGLONG& llLastOpTime,
																		LONGLONG& llMaxOpTime,
																		LONGLONG& llMinOpTime,
																		LONGLONG& llBusOpCounts,
																		LONGLONG& llTxFrames,
																		LONGLONG& llRxFrames,
																		LONGLONG& llTxBytes,
																		LONGLONG& llRxBytes,
																		LONGLONG& llHostErrors,
																		LONGLONG& llBusOpErrors,
																		LONGLONG& llTxFrameErrors,
																		LONGLONG& llRxFrameErrors);

typedef BOOL32 (FnAgeCOMReadWORD)(BYTE ucRTUAddr, WORD wRegAddr, WORD& wData, DWORD dwTimeout);
typedef BOOL32 (FnAgeCOMReadDWORD)(BYTE ucRTUAddr, WORD wRegAddr, DWORD& dwData, DWORD dwTimeout);
typedef BOOL32 (FnAgeCOMReadQWORD)(BYTE ucRTUAddr, WORD wRegAddr, QWORD& qwData, DWORD dwTimeout);
typedef BOOL32 (FnAgeCOMReadMWORD)(BYTE ucRTUAddr, WORD wRegAddr, WORD* pwData, WORD wLength, DWORD dwTimeout);

typedef BOOL32 (FnAgeCOMWriteWORD)(BYTE ucRTUAddr, WORD wRegAddr, WORD wData, DWORD dwTimeout);
typedef BOOL32 (FnAgeCOMWriteDWORD)(BYTE ucRTUAddr, WORD wRegAddr, DWORD dwData, DWORD dwTimeout);
typedef BOOL32 (FnAgeCOMWriteQWORD)(BYTE ucRTUAddr, WORD wRegAddr, QWORD qwData, DWORD dwTimeout);
typedef BOOL32 (FnAgeCOMWriteMWORD)(BYTE ucRTUAddr, WORD wRegAddr, WORD* pwData, WORD wLength, DWORD dwTimeout);

/*
	// Demo for AHD83XX/AHD842X/AHD8CXX/ASD90XX/ASD91XX/ASD92XX
	

	// Start
	// Load AgeCOM.dll
	m_hAgeCOM = ::LoadLibrary(_T("AgeCOM.dll"));
	if (m_hAgeCOM)
	{
		// Get dll function
		VERIFY(m_pAgeCOMSerial = (FnAgeCOMSerial*)::GetProcAddress(m_hAgeCOM, "AgeCOMSerial"));

		VERIFY(m_pAgeCOMIsValid = (FnAgeCOMIsValid*)::GetProcAddress(m_hAgeCOM, "AgeCOMIsValid"));

		VERIFY(m_pAgeCOMGetCOMID = (FnAgeCOMGetCOMID*)::GetProcAddress(m_hAgeCOM, "AgeCOMGetCOMID"));
		VERIFY(m_pAgeCOMGetUSBID = (FnAgeCOMGetUSBID*)::GetProcAddress(m_hAgeCOM, "AgeCOMGetUSBID"));

		VERIFY(m_pAgeCOMGetCOM = (FnAgeCOMGetCOM*)::GetProcAddress(m_hAgeCOM, "AgeCOMGetCOM"));
		VERIFY(m_pAgeCOMSetCOM = (FnAgeCOMSetCOM*)::GetProcAddress(m_hAgeCOM, "AgeCOMSetCOM"));

		VERIFY(m_pAgeCOMGetBaudRate = (FnAgeCOMGetBaudRate*)::GetProcAddress(m_hAgeCOM, "AgeCOMGetBaudRate"));
		VERIFY(m_pAgeCOMSetBaudRate = (FnAgeCOMSetBaudRate*)::GetProcAddress(m_hAgeCOM, "AgeCOMSetBaudRate"));

		VERIFY(m_pAgeCOMGetParity = (FnAgeCOMGetParity*)::GetProcAddress(m_hAgeCOM, "AgeCOMGetParity"));
		VERIFY(m_pAgeCOMSetParity = (FnAgeCOMSetParity*)::GetProcAddress(m_hAgeCOM, "AgeCOMSetParity"));
		
		VERIFY(m_pAgeCOMGetBusInfo = (FnAgeCOMGetBusInfo*)::GetProcAddress(m_hAgeCOM, "AgeCOMGetBusInfo"));

		VERIFY(m_pAgeCOMReadWORD = (FnAgeCOMReadWORD*)::GetProcAddress(m_hAgeCOM, "AgeCOMReadWORD"));
		VERIFY(m_pAgeCOMReadDWORD = (FnAgeCOMReadDWORD*)::GetProcAddress(m_hAgeCOM, "AgeCOMReadDWORD"));
		VERIFY(m_pAgeCOMReadQWORD = (FnAgeCOMReadQWORD*)::GetProcAddress(m_hAgeCOM, "AgeCOMReadQWORD"));
		VERIFY(m_pAgeCOMReadMWORD = (FnAgeCOMReadMWORD*)::GetProcAddress(m_hAgeCOM, "AgeCOMReadMWORD"));

		VERIFY(m_pAgeCOMWriteWORD = (FnAgeCOMWriteWORD*)::GetProcAddress(m_hAgeCOM, "AgeCOMWriteWORD"));
		VERIFY(m_pAgeCOMWriteDWORD = (FnAgeCOMWriteDWORD*)::GetProcAddress(m_hAgeCOM, "AgeCOMWriteDWORD"));
		VERIFY(m_pAgeCOMWriteQWORD = (FnAgeCOMWriteQWORD*)::GetProcAddress(m_hAgeCOM, "AgeCOMWriteQWORD"));
		VERIFY(m_pAgeCOMWriteMWORD = (FnAgeCOMWriteMWORD*)::GetProcAddress(m_hAgeCOM, "AgeCOMWriteMWORD"));
	}


	BOOL32 bResult;

	// Add AUR1252 Serial
	CString strSerial = _T("44742-40890-65242-54760-32341-31258-35993-51871");
	DWORD dwLength = strSerial.GetLength();
	WCHAR* puwSerial = strSerial.GetBufferSetLength(dwLength);
	CHAR pucSerial[MAX_PATH];
	CHAR pucUSBID[MAX_PATH];
	WideCharToMultiByte(CP_ACP, 0, puwSerial, dwLength, (LPSTR)pucSerial, dwLength, NULL, NULL);	
	
	bResult = m_pAgeCOMSerial((BYTE *)&pucSerial, dwLength);	// Can only visit RTU address 1 or 0(broadcast) if has no serial.

	BOOL32 bComValid = m_pAgeCOMIsValid(TRUE);

	WORD wCOMID;
	bResult = m_pAgeCOMGetCOMID(wCOMID);
	bResult = m_pAgeCOMGetUSBID(pucUSBID);

	// Set baudrate, set RTU with the same baudrate.
	// Baudrate can only set to 9600, 19200, 38400, 57600, 115200, 250K, 500K, 1M, 2M, 3M, 4M, 5M, 6M, 8M, 9M, 10M
	// Default RTU Modbus baudrate is 9600, connect RTU Modbus by AUR1252, and connect AUR1252 USB to PC.
	// Default RTU USB baudrate to 115200, can directly connect RTU USB to PC.

	WORD dwBaudRate;
	WORD wParity;

	bResult = m_pAgeCOMGetCOM(dwBaudRate, wParity);	
	bResult = m_pAgeCOMSetCOM(115200, 2);				// bps: 115200, parity: even

	// bResult = m_pAgeCOMGetBaudRate(dwBaudRate);	
	// bResult = m_pAgeCOMSetBaudRate(115200);	// bps: 115200
	
	// bResult = m_pAgeCOMGetParity(wParity);
	// bResult = m_pAgeCOMSetParity(2);					// Parity: even

	LONGLONG llBusRunTime;
	LONGLONG llLastOpTime;
	LONGLONG llMaxOpTime;
	LONGLONG llMinOpTime;
	LONGLONG llBusOpCounts;
	LONGLONG llTxFrames;
	LONGLONG llRxFrames;
	LONGLONG llTxBytes;
	LONGLONG llRxBytes;
	LONGLONG llHostErrors;
	LONGLONG llBusOpErrors;
	LONGLONG llTxFrameErrors;
	LONGLONG llRxFrameErrors;
	bResult = m_pAgeCOMGetBusInfo(llBusRunTime,
																llLastOpTime,
																llMaxOpTime,
																llMinOpTime,
																llBusOpCounts,
																llTxFrames,
																llRxFrames,
																llTxBytes,
																llRxBytes,
																llHostErrors,
																llBusOpErrors,
																llTxFrameErrors,
																llRxFrameErrors);

	// wControl demo
	WORD wControl;
	bResult = m_pAgeCOMReadWORD(1, 0, wControl, 0);		// Read wControl: RTU address 1, Reg address 0(wControl), 0(auto response wait time)

	wControl = 0x0001;																// Write Reset command: Reset if BIT0 = 1;
	bResult = m_pAgeCOMWriteWORD(1, 0, wControl, 0);	// Write wControl: RTU address 1, Reg address 0(wControl), 0x0001(Reset), 0(auto response wait time)

	// Normal to Continue
	wControl = 0x0000;
	bResult = m_pAgeCOMWriteWORD(1, 0, wControl, 0);	// Write wControl: RTU address 1, Reg address 0(wControl), 0x0000(Normal), 0(auto response wait time)

	// Free
	wControl = 0x0004;
	bResult = m_pAgeCOMWriteWORD(1, 0, wControl, 0);	// Write wControl: RTU address 1, Reg address 0(wControl), 0x0004(Free), 0(auto response wait time)

	// Normal to Enable
	wControl = 0x0000;
	bResult = m_pAgeCOMWriteWORD(1, 0, wControl, 0);	// Write wControl: RTU address 1, Reg address 0(wControl), 0x0000(Normal), 0(auto response wait time)


	// Before Motion
	// Set 1000ppr, that is 1000 step per revolution.
	// 1r = 3840000MMS, 1step = 3840MMS, so 1r = 1000step.
	bResult = m_pAgeCOMWriteDWORD(1, 0x002A, 3840, 0);// Write wPulseLength: RTU address 1, Reg address 0x002A(wPulseLength), 3840(3840000/3840=1000), 0(auto response wait time)

	// Offset current dwPositionSet to 0
	bResult = m_pAgeCOMWriteWORD(1, 0, 0x0100, 0);		// Write wControl: RTU address 1, Reg address 0(dwPulsePositionSet), 0x0100(offset to zero), 0(auto response wait time)


	// Motion now
	// Move to 125, that is +45Deg
	bResult = m_pAgeCOMWriteDWORD(1, 0x002E, 125, 0);						// Write dwPulsePositionSet: RTU address 1, Reg address 0x002E(dwPulsePositionSet),  125(+45Deg), 0(auto response wait time)

	// Move to 250, that is +90Deg
	bResult = m_pAgeCOMWriteDWORD(1, 0x002E, 250, 0);						// Write dwPulsePositionSet: RTU address 1, Reg address 0x002E(dwPulsePositionSet),  250(+90Deg), 0(auto response wait time)

	// Move to -125, that is -45Deg
	bResult = m_pAgeCOMWriteDWORD(1, 0x002E, (DWORD)(-125), 0);	// Write dwPulsePositionSet: RTU address 1, Reg address 0x002E(dwPulsePositionSet), -125(-45Deg), 0(auto response wait time)


	// End
	if (m_hAgeCOM)
		::FreeLibrary(m_hAgeCOM);
*/
