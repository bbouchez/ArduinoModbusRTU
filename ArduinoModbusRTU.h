/*
 *  ArduinoModbusRTU.h
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

#ifndef __ARDUINOMODBUSRTU_H__
#define __ARDUINOMODBUSRTU_H__

#include <Arduino.h>

// Maximum size of Modbus RTU frame (see Modbus specification : max RS485 ADU = 256 bytes)
// The value can be reduced to save RAM as very few application will need to receive maximum length telegram
#define MAX_ADU_FRAME_LEN				256

class CArduinoModbusRTU {
public:
		CArduinoModbusRTU ();
		
		// Configuration methods
		void SetSlaveAddress (unsigned char Address);
#ifdef ARDUINO_AVR_LEONARDO
    void SetSerialPort (Serial_* port, long baud, bool UseRS485=false, int RS485TXEnablePin=0);
#else
		void SetSerialPort (HardwareSerial* port, long baud, bool UseRS485=false, int RS485TXEnablePin=0);
#endif
		
		// Main processing task. This method must call in the loop() function as often as possible
		void LoopTask (void);

   // I/O table configuration
   unsigned int NumDiscreteInputs;    // Number of discrete inputs
   unsigned char* UserDiscreteInputArray;     // 1 byte per input
   unsigned int NumOutputCoils;       // Number of output coils
   unsigned char* UserCoilArray;      // 1 byte per output
   unsigned int NumInputRegisters;    // Number of input registers (typically analog inputs)
   unsigned short* UserInputRegisterArray;
   unsigned int NumOutputRegisters;   // Number of output registers (typically analog outputs)
   unsigned short* UserOutputRegisterArray;
		
private:
#ifdef ARDUINO_AVR_LEONARDO
    Serial_* SerialPort;
#else
    HardwareSerial* SerialPort;		// Serial port used for Modbus communication
#endif
	
	unsigned char MasterFrame[MAX_ADU_FRAME_LEN];
	unsigned int RecvByteCounter;		// Number of bytes stored in RequestFrame
	
	unsigned char ResponseFrame[MAX_ADU_FRAME_LEN];
	unsigned int ReplySize;
	
	unsigned char NodeAddress;		// Node address on Modbus
	
	unsigned int InterCharTime_us;
	unsigned long InterFrameTime_us;
	
	bool RS485Mode;
	int RS485TXEnablePin;
	
	// Reset receiver variables to preâre for a new master frame
	void ResetReceiver(void);
	
	// Send message stored in ResponseFrame to serial port
	void SendReply (void);
	
	// Decode received message
	void InterpretMasterFrame (void);
	
	//! Fills the data field of answer to Read Coils or Read Discrete Inputs
	//! \return number of bytes added to the message
	unsigned char BuildReadDODIData (bool ForCoils, unsigned char* BitData, unsigned short StartAddress, unsigned short NumBits);
	
	//! Write multiple coils
	//! \param BitData : pointer to bytes containing data for coils (not the start of the frame)
	void WriteMultipleCoils (unsigned char* BitData, unsigned short StartAddress, unsigned short NumCoils);
	
	//! Write multiple output registers from Modbus
	//! \param WorData : pointer to bytes containing 16 bits values for the registers (not the start of the frame)
	void WriteMultipleRegisters (unsigned char* WordData, unsigned short StartAddress, unsigned short NumRegisters);
	
	//! Fills the data field of answer to Read Input Registers / Read Holding Registers
	//! \return number of bytes added to the message
	unsigned char BuildReadAIData (unsigned char* WordData, unsigned short StartAddress, unsigned short NumRegisters);
};

#endif
