/*
 *  ArduinoModbusRTU.cpp
 *  Modbus RTU (serial) slave implementation for Arduino
 *
 * Copyright (c) 2020-2024 Benoit BOUCHEZ
 * License : MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "ArduinoModbusRTU.h"
#include "modbus_crc.h"

#define MODBUS_FUNCTION_READ_COILS          	0x01
#define MODBUS_FUNCTION_READ_INPUTS         	0x02
#define MODBUS_FUNCTION_READ_HOLDING_REGS   	0x03
#define MODBUS_FUNCTION_READ_INPUT_REGS     	0x04
#define MODBUS_FUNCTION_WRITE_SINGLE_COIL   	0x05
#define MODBUS_FUNCTION_WRITE_SINGLE_REG    	0x06
#define MODBUS_FUNCTION_WRITE_MULTIPLE_COILS    0x0F
#define MODBUS_FUNCTION_WRITE_MULTIPLE_REGS 	0x10

// MODBUS exception codes
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION  0x01
#define MODBUS_EXCEPTION_ILLEGAL_ADDRESS   0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA      0x03
#define MODBUS_EXCEPTION_SERVER_FAILURE    0x04
#define MODBUS_EXCEPTION_ACK               0x05

// Constructor
CArduinoModbusRTU::CArduinoModbusRTU()
{
	this->NodeAddress=0;
	this->RecvByteCounter=0;
	this->ReplySize=0;
	this->RS485Mode=false;
  this->NumDiscreteInputs = 0;
  this->NumOutputCoils = 0;
  this->UserCoilArray = 0;
  this->NumInputRegisters = 0;
  this->UserInputRegisterArray = 0;
  this->NumOutputRegisters = 0;
  this->UserOutputRegisterArray = 0;
}  // CArduinoModbus::CArduinoModbus
// ----------------------------------------------------

#ifdef ARDUINO_AVR_LEONARDO
void CArduinoModbusRTU::SetSerialPort (Serial_* port, long baud, bool UseRS485, int RS485TXEnable)
#else
void CArduinoModbusRTU::SetSerialPort (HardwareSerial* port, long baud, bool UseRS485, int RS485TXEnable)
#endif
{
	SerialPort = port;
	(*SerialPort).begin(baud);
	
	if (baud > 19200)
	{
		/*  "MODBUS over serial line V1.02" document  specifies that for baud rates higher than 19200 bauds:
     *   inter-char time shall be fixed at 750 microseconds 
     *   inter-frame time shall be fixed at 1.75 ms
		*/
		InterCharTime_us = 750;
		InterFrameTime_us = 1750;
	}
 else
 {
   switch (baud)
   {
     case 300 :
        InterCharTime_us = 50000;
        InterFrameTime_us = 117000;
       break;
     case 600 :
        InterCharTime_us = 25000;
        InterFrameTime_us = 58500;
       break;
     case 1200 :
        InterCharTime_us = 12500;
        InterFrameTime_us = 29250;
       break;
     case 2400 :
        InterCharTime_us = 6250;
        InterFrameTime_us = 14625;
       break;
     case 4800 :
        InterCharTime_us = 3125;
        InterFrameTime_us = 7312;     
       break;
     case 9600 :
        InterCharTime_us = 1562;
        InterFrameTime_us = 3656;     
        break;
     default :
      InterCharTime_us = 750;
      InterFrameTime_us = 1750;      
   }
 }
	
	RS485Mode=UseRS485;
	this->RS485TXEnablePin=RS485TXEnable;
	// Set RS485 TX ENABLE line as an output et deactivate transmitter
	if (RS485Mode)
	{
		pinMode(RS485TXEnablePin, OUTPUT);
		digitalWrite(RS485TXEnablePin, LOW);		
	}
}  // CArduinoModbusRTU::SetSerialPort
// ----------------------------------------------------

void CArduinoModbusRTU::SetSlaveAddress (unsigned char Address)
{
	this->NodeAddress = Address;
}  // CArduinoModbusRTU::SetSlaveAddress 
// ----------------------------------------------------

void CArduinoModbusRTU::LoopTask (void)
{
	unsigned char ReceivedByte;
	bool ReceivedBroadcast;
	unsigned short CRC;
	unsigned short ReceivedCRC;
	
	// Get received bytes from queue (in Arduino firmware)
	while ((*SerialPort).available() > 0)
	{
		// The Arduino buffers up to 64 bytes, so we may have more than one byte waiting to be read
		// depending on how long loop() takes time to run.
		
		while ((*SerialPort).available() > 0)
		{
			ReceivedByte = (*SerialPort).read();
			MasterFrame[RecvByteCounter++] = ReceivedByte;		// We need to store incoming bytes in local buffer as Arduino buffer is only 64 bytes
			if (RecvByteCounter>=MAX_ADU_FRAME_LEN) RecvByteCounter=MAX_ADU_FRAME_LEN-1;		// Avoid to pass buffer limit
		}
		
		// Transmitter must send next byte in frame in maximum 1.5 byte time
		// We wait here if at least one byte has been received (the first while avoids the timer if nothing has been received)
		delayMicroseconds (InterCharTime_us);
	}
	
	if (RecvByteCounter==0) return;		// Nothing received = nothing to do
	
	// if we have something in the buffer and elapsed time is greater than interframe time, interpret received message
	// Check first if frame contains enough data to be interpreted (corrupted frame)
	if (RecvByteCounter<4)
	{
		RecvByteCounter = 0;		// Init buffer
		return;
	}
	
	// Check if address is our address
	if (MasterFrame[0]==0xFF)
	{
		ReceivedBroadcast=true;
	}
	else
	{
		if (MasterFrame[0]!=this->NodeAddress) 
		{
			this->ResetReceiver();
			return;		// Frame is not for this node
		}
		ReceivedBroadcast=false;
	}
	
	// Check if CRC is valid (it seems this is a message for us...)
	CRC=ComputeCRC (&MasterFrame[0], RecvByteCounter-2);
	ReceivedCRC=(MasterFrame[RecvByteCounter-2]<<8) | MasterFrame[RecvByteCounter-1];
	if (ReceivedCRC != CRC)
	{
		this->ResetReceiver();
		return;
	}
	
	ReplySize=0;		// Prepare the next response
	
	// Interpret command
	this->InterpretMasterFrame();
	
	// Send the reply except if we received a broadcast command
	if (ReceivedBroadcast==false) this->SendReply();
	
	// Prepare for the next frame
	this->ResetReceiver();
}  // CArduinoModbusRTU::LoopTask
// ----------------------------------------------------

void CArduinoModbusRTU::ResetReceiver(void)
{
	RecvByteCounter=0;
}  // CArduinoModbusRTU::ResetReceiver
// ----------------------------------------------------

void CArduinoModbusRTU::SendReply (void)
{
	int i;
	unsigned short CRC;
	
	if (ReplySize==0) return;
	
	CRC=ComputeCRC(&ResponseFrame[0], ReplySize);
	ResponseFrame[ReplySize++]=CRC>>8;
	ResponseFrame[ReplySize++]=CRC&0xFF;
	
	if (this->RS485Mode)
	{
		digitalWrite(this->RS485TXEnablePin, HIGH);
		delay (1);			// Give time to transceiver to activate
	}
	
	for (i = 0 ; i < ReplySize ; i++) 
	{
		(*SerialPort).write(ResponseFrame[i]);
    }
	(*SerialPort).flush();		// Wait until all bytes are transmitted
	
	if (this->RS485Mode)
	{
		digitalWrite(this->RS485TXEnablePin, LOW);
		delay (InterCharTime_us);		// Give time to pull RS485 to idle (polarization resistors will finish the job)
	}
}  // CArduinoModbusRTU::SendReply
// ----------------------------------------------------

void CArduinoModbusRTU::InterpretMasterFrame (void)
{
	unsigned int StartAddress;
	unsigned int NumElements;
	unsigned short Value16;
	unsigned char NumDataBytes;

    ResponseFrame[0]=NodeAddress;
	
	// Parse function identifier
    switch (MasterFrame[1])
    {
		case MODBUS_FUNCTION_READ_COILS :
			StartAddress=(MasterFrame[2]<<8)+MasterFrame[3];
            NumElements=(MasterFrame[4]<<8)+MasterFrame[5];

            if ((NumElements==0)||(NumElements>2000))
            {
                ResponseFrame[1]=0x81;
                ResponseFrame[2]=MODBUS_EXCEPTION_ILLEGAL_DATA;
                ReplySize=3;
                return;
            }

            if (StartAddress+NumElements>this->NumOutputCoils)
            {
                ResponseFrame[1]=0x81;
                ResponseFrame[2]=MODBUS_EXCEPTION_ILLEGAL_ADDRESS;
                ReplySize=3;
                return;
            }

            // Successful
			ResponseFrame[0]=NodeAddress;
            ResponseFrame[1]=MODBUS_FUNCTION_READ_COILS;
            NumDataBytes=this->BuildReadDODIData (true, &ResponseFrame[3], StartAddress, NumElements);
            ResponseFrame[2]=NumDataBytes;
            ReplySize=NumDataBytes+3;
			break;

		case MODBUS_FUNCTION_READ_INPUTS :    // 0x02
			StartAddress=(MasterFrame[2]<<8)+MasterFrame[3];
            NumElements=(MasterFrame[4]<<8)+MasterFrame[5];

            if ((NumElements==0)||(NumElements>2000))
            {
                ResponseFrame[1]=0x82;
                ResponseFrame[2]=MODBUS_EXCEPTION_ILLEGAL_DATA;
                ReplySize=3;
                return;
            }

            if (StartAddress+NumElements>this->NumDiscreteInputs)
            {
                ResponseFrame[1]=0x82;
                ResponseFrame[2]=MODBUS_EXCEPTION_ILLEGAL_ADDRESS;
                ReplySize=3;
                return;
            }

            // Successful
			ResponseFrame[0]=NodeAddress;
            ResponseFrame[1]=MODBUS_FUNCTION_READ_INPUTS;
            NumDataBytes=this->BuildReadDODIData (false, &ResponseFrame[3], StartAddress, NumElements);
            ResponseFrame[2]=NumDataBytes;
            ReplySize=NumDataBytes+3;
			break;
			
		// Use the same code for functions 03 and 04 for now
		case MODBUS_FUNCTION_READ_INPUT_REGS :
        case MODBUS_FUNCTION_READ_HOLDING_REGS : 
			StartAddress=(MasterFrame[2]<<8)+MasterFrame[3];
            NumElements=(MasterFrame[4]<<8)+MasterFrame[5];

            if ((NumElements==0)||(NumElements>123))
            {
				ResponseFrame[0]=NodeAddress;
                ResponseFrame[1]=0x80+MasterFrame[1];       // Two possible function code, so recompute
                ResponseFrame[2]=MODBUS_EXCEPTION_ILLEGAL_DATA;
                ReplySize=3;
                return;
            }

            if (StartAddress+NumElements>this->NumInputRegisters)
            {
                ResponseFrame[0]=NodeAddress;
                ResponseFrame[1]=0x80+MasterFrame[1];
                ResponseFrame[2]=MODBUS_EXCEPTION_ILLEGAL_ADDRESS;
                ReplySize=3;
                return;
            }

			// Successful
            ResponseFrame[0]=NodeAddress;
			ResponseFrame[1]=MasterFrame[1];		// Get the original function code from command
            NumDataBytes=this->BuildReadAIData(&ResponseFrame[3], StartAddress, NumElements);
            ResponseFrame[2]=NumDataBytes;
			ReplySize=NumDataBytes+3;
            
            break;

		case MODBUS_FUNCTION_WRITE_SINGLE_COIL :
            StartAddress=(MasterFrame[2]<<8)+MasterFrame[3];
            if (StartAddress>=this->NumOutputCoils)
            {
				ResponseFrame[0]=NodeAddress;
                ResponseFrame[1]=0x85;
                ResponseFrame[2]=MODBUS_EXCEPTION_ILLEGAL_ADDRESS;
                ReplySize=3;
                return;
            }
			
			// Command is accepted : prepare answer (copy of the request)
			memcpy (&ResponseFrame[0], &MasterFrame[0], 6);
			ReplySize=6;
			
			// Ignore command if value is not 0xFF00 or 0x0000
			if (MasterFrame[5]!=0x00) return;
			
			if (MasterFrame[4]==0xFF) UserCoilArray[StartAddress]=1;
			if (MasterFrame[4]==0x00) UserCoilArray[StartAddress]=0;
			
			break;

		case MODBUS_FUNCTION_WRITE_MULTIPLE_COILS:
			StartAddress=(MasterFrame[2]<<8)+MasterFrame[3];
            NumElements=(MasterFrame[4]<<8)+MasterFrame[5];

            if ((NumElements==0)||(NumElements>2000))
            {
				ResponseFrame[0]=NodeAddress;
				ResponseFrame[1]=0x8F;
				ResponseFrame[2]=MODBUS_EXCEPTION_ILLEGAL_DATA;
				ReplySize=3;
                return;
            }

            if (StartAddress+NumElements>this->NumOutputCoils)
            {
				ResponseFrame[0]=NodeAddress;
				ResponseFrame[1]=0x8F;
				ResponseFrame[2]=MODBUS_EXCEPTION_ILLEGAL_DATA;
				ReplySize=3;
                return;
            }
			
			// MasterFrame[6] = Number of bytes following
			this->WriteMultipleCoils (&MasterFrame[7], StartAddress, NumElements);

			// Command is accepted : prepare answer (copy of the request - first 6 bytes)
			memcpy (&ResponseFrame[0], &MasterFrame[0], 6);
			ReplySize=6;
			
            break;
			
		case MODBUS_FUNCTION_WRITE_SINGLE_REG :
			StartAddress=(MasterFrame[2]<<8)+MasterFrame[3];            

            if (StartAddress>=this->NumOutputRegisters)		// Register number is 0..MODBUS_NUM_OUTPUT_REGISTERS-1
            {
				ResponseFrame[0]=NodeAddress;
                ResponseFrame[1]=0x86;
                ResponseFrame[2]=MODBUS_EXCEPTION_ILLEGAL_ADDRESS;
                ReplySize=3;
                return;
            }

            Value16=(MasterFrame[4]<<8)+MasterFrame[5]; 
			      UserOutputRegisterArray[StartAddress]=Value16;

			// Command is accepted : prepare answer (copy of the request)
			memcpy (&ResponseFrame[0], &MasterFrame[0], 6);
			ReplySize=6;
               
			break;
			
		case MODBUS_FUNCTION_WRITE_MULTIPLE_REGS : 
			StartAddress=(MasterFrame[2]<<8)+MasterFrame[3];
            NumElements=(MasterFrame[4]<<8)+MasterFrame[5];

            if ((NumElements==0)||(NumElements>123))
            {
                ResponseFrame[0]=NodeAddress;
                ResponseFrame[1]=0x90;
                ResponseFrame[2]=MODBUS_EXCEPTION_ILLEGAL_DATA;
                ReplySize=3;
                return;
            }

            if (StartAddress+NumElements>this->NumOutputRegisters)
            {
                ResponseFrame[0]=NodeAddress;
                ResponseFrame[1]=0x90;
                ResponseFrame[2]=MODBUS_EXCEPTION_ILLEGAL_ADDRESS;
                ReplySize=3;
                return;
            }

			// MasterFrame[6] = number of bytes in the data field
            WriteMultipleRegisters (&MasterFrame[7], StartAddress, NumElements);

			// Command is accepted : prepare answer (copy of the request - first 6 bytes)
			memcpy (&ResponseFrame[0], &MasterFrame[0], 6);
			ReplySize=6;

            break;

		default :
			// Generate ILLEGAL FUNCTION exception
            ResponseFrame[0]=NodeAddress;
            ResponseFrame[1]=0x80+MasterFrame[1];
            ResponseFrame[2]=MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
            ReplySize=3;
	}
}  // CArduinoModbusRTU::InterpretMasterFrame
// ----------------------------------------------------

unsigned char CArduinoModbusRTU::BuildReadDODIData (bool ForCoils, unsigned char* BitData, unsigned short StartAddress, unsigned short NumBits)
{
    unsigned short TotalBits;
    unsigned char NumBytes;
    unsigned char DataByte;
    unsigned char BitCounter;
	unsigned char Mask;

    // Loop until we have read the requested amount of coils
    NumBytes=0;
    TotalBits=0;
    DataByte=0;
    BitCounter=0;
	Mask=0x01;				// Modbus fills bytes from LSB to MSB
    while (TotalBits<NumBits)
    {
        // Get state for the adressed 
        //BitValue=0;
        if (ForCoils)
        {
			if (UserCoilArray[StartAddress]!=0) DataByte=DataByte|Mask;
        }
        else
        {
			if (UserDiscreteInputArray[StartAddress]!=0) DataByte=DataByte|Mask;
        }
		StartAddress++;
		Mask=Mask<<1;

        // Check if we have made a complete byte
        BitCounter+=1;
        if (BitCounter>=8)
        {  // Full byte created : store it in the answer
            *BitData=DataByte;
            BitData++;     // Move to next byte in reply
            BitCounter=0;
            NumBytes++;
            DataByte=0;
        }

        TotalBits+=1;
    }

    // Special case : if we have not made a full byte, save the current result and increment byte counter
    if (BitCounter!=0)
    {
        NumBytes++;
        *BitData=DataByte;
    }

    return NumBytes;
}  // CArduinoModbusRTU::BuildReadDODIData
// ----------------------------------------------------

void CArduinoModbusRTU::WriteMultipleCoils (unsigned char* BitData, unsigned short StartAddress, unsigned short NumCoils)
{
    unsigned short TotalCoils;
    unsigned char BitMask;
    unsigned char BitCount;
    unsigned char BitValue;
	unsigned char CmdByte;

    TotalCoils=0;
    BitMask=0x01;
    BitCount=0;
    while (TotalCoils<NumCoils)
    {
		CmdByte=*BitData;
        if ((CmdByte&BitMask)!=0) BitValue=1;
        else BitValue=0;

        UserCoilArray[StartAddress]=BitValue;

		StartAddress+=1;
        TotalCoils+=1;
		
        // Move to next bit from command
        BitMask=BitMask<<1;
        BitCount+=1;
        if (BitCount>=8)
        {
            BitCount=0;
            BitMask=0x01;
            BitData++;       // Move to next byte in command
        }
    }
}  // CArduinoModbusRTU::WriteMultipleCoils
// ----------------------------------------------------

void CArduinoModbusRTU::WriteMultipleRegisters (unsigned char* WordData, unsigned short StartAddress, unsigned short NumRegisters)
{
	unsigned short TotalRegisters;
	unsigned short RegisterValue;
	unsigned short MSB, LSB;
	
	TotalRegisters=0;
	while (TotalRegisters<NumRegisters)
	{
		MSB=*WordData;
		WordData++;
		LSB=*WordData;
		WordData++;
		
		UserOutputRegisterArray[StartAddress]=(MSB<<8)+LSB;
		
		StartAddress++;
		TotalRegisters++;
	}
}  // CArduinoModbusRTU::WriteMultipleRegisters
// ----------------------------------------------------

unsigned char CArduinoModbusRTU::BuildReadAIData (unsigned char* WordData, unsigned short StartAddress, unsigned short NumRegisters)
{
	unsigned short TotalRegisters;
	unsigned char NumBytes;
	unsigned short Value16;
	
	TotalRegisters=0;
	NumBytes=0;
	while (TotalRegisters<NumRegisters)
	{
		// Copy the 16 bits value in the message body
		Value16=UserInputRegisterArray[StartAddress];
		*WordData=Value16>>8;		// Write MSB
		WordData++;
		*WordData=Value16&0xFF;
		WordData++;
		
		// Switch to next register
		NumBytes+=2;
		StartAddress++;
		TotalRegisters++;
	}
	
	return NumBytes;
}  // CArduinoModbusRTU::BuildReadAIData
// ----------------------------------------------------
