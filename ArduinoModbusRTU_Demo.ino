/*
 *  ArduinoModbusRTU.ino
 *  Demo application of ArduinoModbusRTU library for Arduino
 *  This application maps I2C chips into Modbus memory using a Velleman K8000 I/O board
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

#include <Arduino.h>
#include <Wire.h>
#include "ArduinoModbusRTU.h"

// Define the number of registers, inputs and coils availabkle in our I/O module
#define NUM_DISCRETE_INPUTS     8   // PCF8574 #1  (8 boolean inputs)
#define NUM_COILS               8   // PCF8574 #2  (8 boolean outputs)
#define NUM_INPUT_REGISTERS     4   // PCF8591 4 analog input
#define NUM_OUTPUT_REGISTERS    1   // PCF8591 1 analog output

// I2C addresses
#define PCF8574_IN_ADDRESS      56
#define PCF8574_OUT_ADDRESS     57
#define PCF8591_ADDRESS         0x48

// Arrays containing I/O values available from Modbus
unsigned char DiscreteInput[NUM_DISCRETE_INPUTS];
unsigned char CoilOutput[NUM_COILS];
unsigned short InputRegister[NUM_INPUT_REGISTERS];
unsigned short OutputRegister[NUM_OUTPUT_REGISTERS];

CArduinoModbusRTU	Modbus;

// Change detection variable (avoid useless I²C communication if new data is the same as previous cycle)
unsigned char LastDigitalOutByte;
// Counter to control analog input multiplexer
unsigned char ADCChannelCounter;

void setup()
{
	// Activate I2C and configure I/O
  // We must stay at 100kHz as PCF8574 does not support 400kHz I2C clock frequency 
	Wire.begin();

	// Input configuration
	// Set PCF8574#1 to 0xFF when used for inputs
	Wire.beginTransmission (PCF8574_IN_ADDRESS);
	Wire.write (0xFF);
	Wire.endTransmission();
	
	// Output configuration
	// Set PCF8574# to 0xFF
	Wire.beginTransmission (PCF8574_OUT_ADDRESS);
	Wire.write (0xFF);
	Wire.endTransmission();
	
	// PCF8591 configuration
	Wire.beginTransmission (PCF8591_ADDRESS);
	Wire.write (0x40);    // Control byte (enable DAC, no auto increment, four single ended inputs, select ADC0)
	Wire.write (0x00);    // Output 0%
	Wire.endTransmission();

  // Configure Modbus slave
	Modbus.SetSerialPort (&Serial, 115200);
	Modbus.SetSlaveAddress (0);
  Modbus.NumDiscreteInputs = NUM_DISCRETE_INPUTS;
  Modbus.UserDiscreteInputArray = &DiscreteInput[0];
  Modbus.NumOutputCoils = NUM_COILS;
  Modbus.UserCoilArray = &CoilOutput[0];
  Modbus.NumInputRegisters = NUM_INPUT_REGISTERS;
  Modbus.UserInputRegisterArray = &InputRegister[0];
  Modbus.NumOutputRegisters = NUM_OUTPUT_REGISTERS;
  Modbus.UserOutputRegisterArray = &OutputRegister[0];
	
	LastDigitalOutByte=0;
	ADCChannelCounter=0;
}  // setup
// ----------------------------------------------------

void loop()
{
	int i;
	unsigned char OutByte;
	unsigned char InputByte;
	unsigned char Mask;
	unsigned int ADCValue;
	
	Modbus.LoopTask();
	
	// Read digital inputs at every cycle
	Wire.requestFrom(PCF8574_IN_ADDRESS, 1);
	InputByte=Wire.read();
	InputByte=~InputByte;
	
	// Write input bits into Modbus table
	Mask=0x01;
	for (i=0; i<8; i++)
	{
    if ((InputByte & Mask)!=0) DiscreteInput[i]=1;
    else DiscreteInput[i] = 0;
		Mask=Mask<<1;
	}
	
	// Update coil outputs from Modbus
	OutByte=0;
	Mask=0x01;
	for (i=0; i<8; i++)
	{
		if (CoilOutput[i]==0) OutByte+=Mask;		// Invert output when active (PCF8574 outputs are active low on K8000)
		Mask=Mask<<1;
	}
	
	// Write digital data to I²C only if data has changed from previous cycle
	// This will save loop cycle time (removing useless communication)
	if (OutByte!=LastDigitalOutByte)
	{
		Wire.beginTransmission (PCF8574_OUT_ADDRESS);
		Wire.write (OutByte);
		Wire.endTransmission();
		LastDigitalOutByte=OutByte;
	}

	// Update analog output from Modbus data and switch to next ADC channel
	Wire.beginTransmission (PCF8591_ADDRESS);   
	Wire.write (0x44);                // Control byte : activate DAC, single ended inputs, auto increment from ADC channel 0
	Wire.write (OutputRegister[0]>>8);    // Output value is 8 bits on DAC, while Modbus is 16 bits
	Wire.endTransmission();

  /*
	// Read analog inputs :  not used for now
	Wire.requestFrom (PCF8591_ADDRESS, 5);		// 4 ADC channels + 1 dummy channel (contains last conversion)
	for (i=0; i<5; i++)
	{
		ADCValue=(unsigned int)Wire.read();	
		if (i>0)		// First byte is the last converted byte, ignore it
		{	
			InputRegister[i] = ADCValue<<8;  // ADC is 8 bits, Modbus asks for 16 bits
		}
	}
 */
}  // loop
// ----------------------------------------------------
