# MSPM0G350x Development Board

A hardware design for an Arduino UNO form-factor development board featuring the Texas Instruments MSPM0G350x Mixed-Signal Microcontroller with built-in CAN-FD interface.

---

## Overview

This project contains the KiCad design files for a custom development board based on the Texas Instruments MSPM0G3507 MCU, featuring CAN-FD connectivity in a familiar Arduino UNO-compatible form factor.

<img width="1166" height="596" alt="image" src="https://github.com/user-attachments/assets/186c431c-6bd9-49fb-9106-728b3500c71b" />

---

## Features

### Microcontroller
- **MCU**: Texas Instruments MSPM0G3507 (or MSPM0G3506)
- **Core**: ARM Cortex-M0+ 32-bit processor
- **Clock Speed**: Up to 80 MHz
- **Flash Memory**: 128 KB (MSPM0G3507) / 64 KB (MSPM0G3506)
- **SRAM**: 32 KB

### Integrated Peripherals
- **CAN-FD Controller**: Built-in CAN-FD with up to 5 Mbps
- **ADC**: 12-bit Analog-to-Digital Converter (up to 20 channels)
- **DAC**: 12-bit Digital-to-Analog Converter (2 channels)
- **Timers**: Multiple 16-bit and 32-bit timers
- **PWM**: High-resolution PWM outputs
- **Communication**: UART, SPI, I2C, DMA

### Board Features
- **Form Factor**: Arduino UNO R3 compatible (2.54mm pitch, 3.3V logic)
- **Power Supply**: 7-12V DC barrel jack or USB-C (5V)
- **USB Interface**: Integrated USB-C for programming and debug
- **CAN Connector**: Standard DB9 or 4-pin header for CAN-FD
- **Power LED**: Power indicator
- **User LED**: Programmable status LED on GPIO pin

---

## Pinout

### Arduino-Compatible Headers

#### Power Pins
| Pin | Function | Description |
|-----|----------|-------------|
| VIN | Power Input | External power (7-12V) |
| 5V | 5V Output | Regulated 5V (max 500mA) |
| 3.3V | 3.3V Output | Regulated 3.3V (max 300mA) |
| GND | Ground | Common ground |
| GND | Ground | Common ground |
| AREF | Analog Ref | Analog reference voltage |
| SDA | I2C Data | I2C data line (PA10) |
| SCL | I2C Clock | I2C clock line (PA11) |

#### Digital I/O Pins (Right Header)
| Pin | Arduino | MSPM0 Pin | Function |
|-----|---------|-----------|----------|
| D0 | RX0 | PA8 | UART0 RX / GPIO |
| D1 | TX0 | PA9 | UART0 TX / GPIO |
| D2 | INT0 | PA7 | External Interrupt / GPIO |
| D3 | PWM | PA6 | PWM / GPIO |
| D4 | - | PA5 | GPIO |
| D5 | PWM | PA4 | PWM / GPIO |
| D6 | PWM | PA3 | PWM / GPIO |
| D7 | - | PA2 | GPIO |

#### Digital I/O Pins (Left Header)
| Pin | Arduino | MSPM0 Pin | Function |
|-----|---------|-----------|----------|
| D8 | - | PC5 | GPIO |
| D9 | PWM | PC4 | PWM / GPIO |
| D10 | SPI SS | PC3 | SPI CS / GPIO |
| D11 | SPI MOSI | PC2 | SPI MOSI / GPIO |
| D12 | SPI MISO | PC1 | SPI MISO / GPIO |
| D13 | SPI SCK | PC0 | SPI SCK / GPIO |

#### Analog Pins (Bottom Header)
| Pin | Arduino | MSPM0 Pin | Function |
|-----|---------|-----------|----------|
| A0 | ADC0 | PA26 | ADC Channel 0 |
| A1 | ADC1 | PA27 | ADC Channel 1 |
| A2 | ADC2 | PA28 | ADC Channel 2 |
| A3 | ADC3 | PA25 | ADC Channel 3 |
| A4 | ADC4/SDA | PA10 | ADC / I2C SDA |
| A5 | ADC5/SCL | PA11 | ADC / I2C SCL |

### CAN-FD Interface
| Pin | Function | Notes |
|-----|----------|-------|
| CAN_H | CAN High | Requires 120Ω termination |
| CAN_L | CAN Low | Requires 120Ω termination |
| GND | Ground | Signal ground |

---

## Electrical Specifications

| Parameter | Value |
|-----------|-------|
| Operating Voltage | 3.3V (logic), 5V (USB power) |
| Input Voltage (Barrel) | 7-12V DC |
| Max Current (3.3V) | 300 mA |
| Max Current (5V USB) | 500 mA |
| I/O Voltage | 3.3V (5V tolerant inputs) |
| CAN-FD Speed | Up to 5 Mbps |
| Operating Temp | -40°C to 85°C |

---

## Mechanical Specifications

| Parameter | Value |
|-----------|-------|
| Board Dimensions | 68.6mm × 53.3mm (UNO form factor) |
| Mounting Holes | 4× M3 holes (2.9mm diameter) |
| Header Pitch | 2.54mm (0.1") |
| PCB Thickness | 1.6mm |

---

## Getting Started

### Prerequisites
- Texas Instruments MSPM0 MCU SDK
- CMSIS-DAP or compatible debug probe
- Arduino IDE with MSPM0 support

### Programming

1. **Via Debugger**: Connect a CMSIS-DAP probe to the SWD pins (SWDIO, SWDCLK, GND)
2. **Via USB**: Use the built-in USB bootloader (if programmed)

### Software Setup

```bash
# Install MSPM0 SDK
git clone https://github.com/TexasInstruments/mspm0_sdk.git

# Install ARM GCC toolchain
# Download from: https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm
```

---

## Development Tools

- **EDA**: KiCad 8.0+
- **MCU SDK**: TI MSPM0 SDK
- **IDE**: TI Code Composer Studio or VS Code with MSPM0 plugins
- **Programmer**: CMSIS-DAP compatible probe

---

## Project Structure

```
MSPMO-based-Hardware/
├── MSPMO-based-Hardware.kicad_pro    # KiCad project file
├── MSPMO-based-Hardware.kicad_sch    # Schematic
├── MSPMO-based-Hardware.kicad_pcb    # PCB layout
├── MSPMO-based-Hardware.kicad_prl    # Layout settings
├── MSPMO-based-Hardware.kicad_dru    # Design rules
└── Readme.md                         # This file
```

---

## Hardware Design

### Power Supply Circuit
- USB-C connector for 5V input
- Barrel jack for 7-12V DC input
- AMS1117-3.3 LDO for 3.3V regulation
- LM1117-5.0 LDO for 5V regulation (optional)

### CAN-FD Transceiver
- SN65HVD230 or MCP2562 CAN-FD transceiver
- 120Ω terminating resistor (selectable)
- ESD protection on CAN lines

### Additional Components
- Reset button
- Boot selector (for bootloader)
- Power LED (green)
- User LED (red, on PC7)
- Crystal oscillator (32.768 kHz for RTC)

---

## Programming Examples

### Basic Blink
```c
#include "ti_mspm0g3507.h"

int main(void) {
    // Configure PC7 as output (User LED)
    GPIOC->DIR_b.PIN7 = GPIO_DIR_OUT;
    
    while(1) {
        GPIOC->DOUT_BSET = (1 << 7);  // LED on
        delay_ms(500);
        GPIOC->DOUT_BCLR = (1 << 7);  // LED off
        delay_ms(500);
    }
}
```

### CAN-FD Transmit
```c
#include "ti_mspm0g3507.h"
#include "can.h"

int main(void) {
    CAN_Init(CAN_BAUD_500K);
    
    CAN_TxHeaderTypeDef txHeader;
    txHeader.Id = 0x123;
    txHeader.DLC = 8;
    txHeader.IDE = CAN_ID_STD;
    txHeader.FDF = CAN_FD_CAN;
    
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    
    while(1) {
        CAN_SendMessage(&txHeader, data);
        delay_ms(1000);
    }
}
```

---

## License

This hardware design is provided as-is for educational and development purposes.

---

## References

- [TI MSPM0G3507 Datasheet](https://www.ti.com/lit/ds/symlink/mspm0g3507.pdf)
- [TI MSPM0 SDK Documentation](https://dev.ti.com/tirex/explore/node?node=AGJ7H6JRK1A-jhO1mH3k0pw__LATEST)
- [CAN-FD Specification](https://www.can-cia.org/can-fd/)
- [Arduino UNO R3 Reference Design](https://store.arduino.cc/arduino-uno-rev3)

---

## Support

For issues or questions:
- Open an issue on the project repository
- Check TI's E2E forums for MSPM0-specific questions

---

**Version**: 1.0  
**Date**: February 2026  
**Author**: Odari-Designs  
