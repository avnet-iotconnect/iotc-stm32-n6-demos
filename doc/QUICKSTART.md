## 1. Introduction
This document outlines the steps of setting up the STM32N6570-DK with Avnet's /IOTCONNECT platform.  The QuickStart demonstrates ST's edge AI object detection model to 
provide object coordinates information to /IOTCONNECT.

## 2. Prerequisites
This guide has been tested with the following environment:
* PC with Windows 10/11
* 2x USB Type-C data cables
* A serial terminal application such as [Tera Term](https://sourceforge.net/projects/tera-term/) (Recommended) or a browser-based version such as [Google Chrome Labs Serial Terminal](https://googlechromelabs.github.io/serial-terminal/)
* A [MyST]() account to download software
* [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) software

## 3. Hardware Setup
* Connect the camera via the white ribbon cable to the board(blue stripe facing up on both sides).
* Connect two USB-A to USB type-C cables to the board from PC.

## 4. Flash the Firmware
* Download and unzip the pre-compiled [firmware](https://downloads.iotconnect.io/partners/st/demos/uvc/n6uvc-binary.zip).
* Switch the BOOT1 switch to the right position.
* Launch the STM32CubeProgrammer.
* Program `ai_fsbl.hex` (to be done once) (First stage boot loader).
* Program `network_data.hex` (parameters of the networks; to be changed only when the network is changed).
* Program `uvc_signed.bin` (firmware application). (Add 0x70100000 in the 'Start Address' when programming)
* Switch the BOOT1 switch to the left position.


## 5. Create device/template on IOTCONNECT
* An IOTCONNECT Device Template will need to be created or imported. This defines the data format the platform should expect from the device.
  * Download the premade  [Device Template](n6uvc_template.JSON)
  * **Click** the Device icon and the "Device" sub-menu.
  * At the bottom of the page, select the "Templates" icon from the toolbar and import the template.
* IOTCONNECT Device Creation:
  * **Click** the Device icon and the "Device" sub-menu:
  * At the top-right, click on the "Create Device" button:
  * Enter the **DUID** saved from earlier in the *Unique ID* field
  * Select the template from the dropdown box that was just imported ("n6uvc")
  * Ensure "Auto-generated (recommended)" is selected under *Device certificate*
  * Click **Save & View**
  * Download the device credentials from **Connection Info** (on the right side of webpage when you select the device)
  * The Company ID (**CPID**) and Environment (**ENV**) variables identifying your IOTCONNECT account are required and they are loacted in the "Settings" -> "Key Vault" section of the platform.
  * You have all the information for step 3.
  
## 6. Flash/Configure the DA16k PMOD module
Follow the instructions in the [QuickStart Guide](https://github.com/avnet-iotconnect/iotc-dialog-da16k-sdk/blob/main/doc/QUICKSTART.md) to flash the DA16k PMOD.


## 7. Run the Applicaton
* Power off the board by unplugging the USB cables.
* Connect the DA16k PMOD module to the board's STMOD+ Connector via [jumper wires](https://www.newark.com/multicomp-pro/mp006283/jumper-wire-kit-male-to-female/dp/15AJ6557) (PMOD pins are listed in the table below)

| PMOD Connector pin # |     Signal      |             
|:--------------------:|:---------------:| 
|          2           |   to HOST_TXD   | 
|          3           |   to HOST_RXD   | 
|       5  or 11       |       GND       |
|       6  or 12       | VCC (3V3 or 5V) |

* Power up the board.
* Launch the host camera application. (ex. N6CamViwer or ffplay)

> [!NOTE]
> If the application is not running properly. You will need to switch the JP2 jumper from 1-2 to 3-4 position in order to provide enough power to the board.

