# 180666 LoRaSwich

This is the firmware for both parts of the LoRaSwitch with Statefeedback. It consists of two Elektor LoRa Node Boards and two support PCBs. The code itself is written unsing the ArduinoIDE and shows the basic usage of the LoRaRAW mode provided by the MCCI LMIC Library. This will enable the Transmitter and Receiver to exchange data over 300 meter distance and provide a feedback on the Switch opperation to the user.  

## Getting Started

Besides the Hardware you need to have the ArduinoIDE 1.8.x or newer installed. Also make sure you have the latest STM32duino Board Support Package installed. Grab a copy from https://github.com/stm32duino/Arduino_Core_STM32 and jsut follow https://github.com/stm32duino/wiki/wiki/Getting-Started for the installation. To programm the Boards you need a Serial-to-USB converter, a cheap CH340 based one ( https://www.elektor.com/ch340-usb-to-ttl-converter-uart-module-ch340g-3-3-v-5-5-v ) and a few jumper wire will do the trick. For more information on how to upload firmware you can visit: https://www.elektormagazine.com/labs/lorawan-node-experimental-platform . 

### Prerequisites

You need the following Arduino librarys to be installed:

* MCCI LoRaWAN LMIC library 2.3.1
* CRC32 2.0.0 by Cristopher Baker
* Crypro 0.0.2 by Rhys Weatherley
* U8G2 by Oliver

The code is written for the Arduino Core STM32 1.8.0 and some changes may be requiered as with 1.9.0 the API changes for a few functions. Also not that in the current version 1.8.0 the EEPROM Emulation may be broken.

The whole project and description can be found here: https://www.elektormagazine.com/labs/lora-controlled-switch-with-state-feedback


