# STUSB4500
##### Arduino library (_and [Go module](#tinygo-upgrade)_) for real-time capabilities of the STUSB4500 USB PD sink controller

> This repository and README contains the original Arduino source code and docs. For the TinyGo port, please see the project referenced in the [Go module](#tinygo-upgrade) section.

----

## Arduino

###### Features include:
- [x] Works with native Arduino Wire (I²C) library
- [x] USB Power Delivery v2.0 and v3.0 compatible state machine
- [x] Designed for [STUSB4500 Compact Breakout](https://www.tindie.com/products/oxplot/stusb4500-compact-breakout/)
   - Same chip used on the [SparkFun Power Delivery Board](https://www.sparkfun.com/products/15801), although it hasn't been tested. Feel free to send me one!
- [x] Can analyze and identify all available power profiles
- [x] Can request and use any arbitrary power profile +5-20V/0-5A in 50mV/10mA increments
   - Does not lose VBUS on power transition, maintains power output
- [x] Detect cable orientation (CC1/CC2 termination)
- [x] Uses ALRT/ATCH interrupt pins for efficient response
- [x] Completely enable/disable power output
- [x] Provides user callback support for:
   - Cable attach/detach events 
   - PD source capability discovery events
- [x] Set up to 3 custom power profiles for fallback
- [ ] Reformat NVM for default power profiles in standalone mode (not yet implemented)

[An example sketch](examples/basic-demo/basic-demo.ino) is included that demonstrates callback functionality, analyzing available source power profiles, and setting a custom power profile. 
 
###### Supported Devices

I can confirm the library functions on the following devices (similar devices should also be supported).

|       Key        |Meaning                |
|:----------------:|:----------------------|
|:heavy_check_mark:|Tested, working        |
|        :x:       |Tested, **not** working|
|:heavy_minus_sign:|**Not** tested         |

|    Manufacturer    |      Board      |           MCU             |      Status      |
|:------------------:|:---------------:|:-------------------------:|:----------------:|
|      Adafruit      |   Trinket M0    |  ATSAMD21E18 (Cortex-M0)  |:heavy_check_mark:|
|      Adafruit      |  ItsyBitsy M0   |  ATSAMD21G18 (Cortex-M0)  |:heavy_check_mark:|
|      Adafruit      |  ItsyBitsy M4   | ATSAMD51G19A (Cortex-M4)  |:heavy_check_mark:|
|      Adafruit      |   Feather M4    | ATSAMD51J19A (Cortex-M4)  |:heavy_check_mark:|
|      Adafruit      |Grand Central M4 | ATSAMD51P20A (Cortex-M4)  |:heavy_check_mark:|
|      Adafruit      |STM32F405 Feather|  STM32F405RG (Cortex-M4)  |:heavy_check_mark:|
|        PJRC        |   Teensy 3.2    | MK20DX256VLH7 (Cortex-M4) |:heavy_check_mark:|
|        PJRC        |   Teensy 3.6    |   MK66FX1M0 (Cortex-M4)   |:heavy_check_mark:|
|        PJRC        |   Teensy 4.0    |MIMXRT1062DVL6A (Cortex-M7)|:heavy_check_mark:|
|        PJRC        |   Teensy 4.1    |MIMXRT1062DVJ6A (Cortex-M7)|:heavy_check_mark:|
| STMicroelectronics |STM32G0 Nucleo-64|  STM32G071RB (Cortex-M0)  |:heavy_check_mark:|
| STMicroelectronics |STM32G0 Nucleo-32|  STM32G031K8 (Cortex-M0)  |:heavy_check_mark:|
| STMicroelectronics |STM32G4 Nucleo-32|  STM32G431KB (Cortex-M4)  |:heavy_check_mark:|
|Nordic Semiconductor|        ∀        |    nRF51x (Cortex-M0)     |:heavy_minus_sign:|
|Nordic Semiconductor|        ∀        |    nRF52x (Cortex-M4)     |:heavy_minus_sign:|
|Nordic Semiconductor|        ∀        |    nRF53x (Cortex-M33)    |:heavy_minus_sign:|
|Nordic Semiconductor|        ∀        |    nRF91x (Cortex-M33)    |:heavy_minus_sign:|
|     Espressif      |        ∀        |          ESP8266          |:heavy_minus_sign:|
|     Espressif      |        ∀        |           ESP32           |:heavy_minus_sign:|

## Troubleshooting
The PD protocol has *very* rigid timing requirements. These can be difficult to accommodate even in normal circumstances, but is even more difficult since there is another device (our STUSB4500) mediating communication over an I2C bus, which is rather slow. This is partly why the interrupts are vital to successful operation.

Point being, it may be difficult to achieve reliable results on some systems (such as AVR-based devices like Arduino Uno), and you may need to experiment with different microcontrollers. 

If you have issues receiving cable attach/detach or PD capability discovery events, ensure the following:
- You are using a **hardware** I2C port on the microcontroller
- The I2C (SDA/SCL) and interrupt (ALRT/ATCH) wires connecting microcontroller and STUSB4500 are as short as possible
- GND on STUSB4500 is connected to GND on microcontroller
- VDD (VPP on oxplot breakout) is pulled up to 3.3V (or to same level as I2C bus voltage)
- VSYS (VCC on oxplot breakout) is pulled down to ground (use microcontroller ground pin)
  - The Sparkfun breakout pulls this high to VDD ([according to their schematic](https://cdn.sparkfun.com/assets/9/2/6/8/6/SparkFun_PowerDeliveryBoardSchematic.pdf)), but the [datasheet](https://www.st.com/resource/en/datasheet/stusb4500.pdf) says to connect it to ground if it is not used (and it isn't used, because the device is powered by VBUS from the USB-C receptacle). So I'm not sure about this one. Try pulling this in the opposite direction if you're having issues.
 
---

# [TinyGo Upgrade](https://github.com/ardnew/tinygo-stusb4500)

This library has been rewritten and ported to Go for use with the [TinyGo Project](https://tinygo.org/), with improved performance, stability, and device compatibility. 

If feasible, I highly recommend you use that software for future projects ([tinygo-stusb4500](https://github.com/ardnew/tinygo-stusb4500)), because it is subjected to more comprehensive testing, supports many more chips and boards, and won't suffer from the fate of Arduino's I2C software library (i.e., it will run better).

