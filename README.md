# ArduinoModbusRTU
Slave Modbus RTU (serial port) library for Arduino

This library implements a complete Modbus RTU Slave on Arduino Serial port.
Supported Modbus functions :
Read Coils (0x01)
Read Inputs (0x02)
Read Holding Registers (0x03)
Read Input Registers (0x04) - equivalent to Read Holding Registers
Write Single Coil (0x05)
Write Single Register (0x06)
Write Multiple Coils (0x0F)
Write Multiple Registers

Serial communication can be performed :
* using RX/TX with a RS232 adapter
* using RX/TX with a RS485 adapter
* using USB serial port (see note)

To use the library, add following files to your Arduino project:
* ArduinoModbusRTU.cpp
* ArduinoModbusRTU.h
* modbus_crc.c
* modbus_crc.h

In application code, boolean inputs and outputs must be represented as arrays of unsigned char, containing 0 or 1.
Register inputs and outputs are represented by arrays of unsigned short (16 bits)
The array addresses and size shall be passed into Modbus RTU library in the following way, in the setup() function :
```
  Modbus.NumDiscreteInputs = NUM_DISCRETE_INPUTS;
  Modbus.UserDiscreteInputArray = &DiscreteInput[0];
  Modbus.NumOutputCoils = NUM_COILS;
  Modbus.UserCoilArray = &CoilOutput[0];
  Modbus.NumInputRegisters = NUM_INPUT_REGISTERS;
  Modbus.UserInputRegisterArray = &InputRegister[0];
  Modbus.NumOutputRegisters = NUM_OUTPUT_REGISTERS;
  Modbus.UserOutputRegisterArray = &OutputRegister[0];
```

The Modbus slave processing is performed by a call to Modbus.LoopTask() which must be called as often as possible in the loop() function.

The example source code shows an application of the library to turn a Velleman K8000 board with two PCF8574 chips for boolean inputs and outputs and one PCF8591 for analog inputs and outputs, using a I2C link between the Arduino and the I/O board.

![K8000_cropped](https://github.com/user-attachments/assets/30fb886f-de01-48dd-b7be-d6f9d703186f)


>[!NOTE]
The library works only with "physical" serial ports like on Arduino Mega, Arduino Uno, Arduino Due, etc...
For an unknown reason, the library does not work with USB serial port emulation (like on the Arduino Leonardo)
