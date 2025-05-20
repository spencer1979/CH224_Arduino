# CH224 PD-sink Controller for  Arduino

A simple Arduino library for controlling the CH224 PD controller via I2C.

## Features
- I2C read/write for CH224 registers
- PDO (Power Data Object) parsing
- Voltage switching (5V, 9V, 12V, 15V, 20V, 28V, PPS, AVS)
- Current/voltage monitoring
- Supports EPR, QC 3.0/2.0, BC modes

## Key Registers
- **0x09**: Status (PD/QC/BC/EPR/AVS flags)
- **0x0A**: Voltage control (select output voltage)
- **0x50**: Current (unit: 50mA)
- **0x51/0x52**: AVS voltage config (unit: 25mV)
- **0x53**: PPS voltage config (unit: 100mV)
- **0x60~0x8F**: PD Source Capabilities

## Installation
1. Clone:  
   `git clone https://github.com/spencer1979/CH224_arduino.git`
2. Copy `CH224_Arduino_library` to your Arduino libraries folder.

## Hardware Connection
- **SDA**: Arduino SDA (e.g. A4)
- **SCL**: Arduino SCL (e.g. A5)
- **VCC**: 3.3V or 5V
- **GND**: GND

## Usage Example

`CH224_basic.ino` demonstrates:
- Automatic USB plug/unplug detection and CH224 power control
- Two buttons to switch between different fixed voltage PDOs
- Customizable I2C pins and real-time status via serial port

> For details, see `examples/CH224_Example/CH224_basic.ino` and modify pin definitions as needed.

## Debugging
Uncomment `#define CH224_DEBUG 1` in `CH224.h` to enable debug messages.

## License
MIT License. See `LICENSE`.