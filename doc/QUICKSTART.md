## 1. Introduction
This document outlines the steps of setting up the STM32N6570-DK with Avnet's /IOTCONNECT platform.  The QuickStart demonstrates ST's edge AI computer vision model (based on the [n6-ai-h264-uvc](https://www.st.com/en/development-tools/stm32n6-ai.html#get-software) example) and 
provides object detection information to /IOTCONNECT for visualization. 

<img src="https://github.com/avnet-iotconnect/iotc-stm32-n6-demos/blob/main/media/n6.jpg" width="300">

## 2. Prerequisites
This guide has been written and tested with the hardware and software listed below, but may work with other environments with some modifications.

### Hardware
* PC with Windows 10/11
* 2x USB Type-C data cables
* 4x Male to Female header jumpers [Buy from Newark](https://www.newark.com/multicomp-pro/mp006283/jumper-wire-kit-male-to-female/dp/15AJ6557)
* /IOTCONNECT enabled PMOD such as the [DA16200](https://www.avnet.com/shop/us/products/renesas-electronics/us159-da16200mevz-3074457345649323747) or [DA16600](https://www.avnet.com/shop/us/products/renesas-electronics/us159-da16600evz-3074457345649323748).

### Software
* A serial terminal application such as [Tera Term](https://sourceforge.net/projects/tera-term/) (Recommended) or a browser-based version such as [Google Chrome Labs Serial Terminal](https://googlechromelabs.github.io/serial-terminal/)
* [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) (A free [MyST](https://my.st.com/cas/login) account is required to download)
* Download and unzip the pre-compiled firmware [n6uvc-binary.zip](https://downloads.iotconnect.io/partners/st/demos/uvc/n6uvc-binary.zip) to a location near the root of your drive.
* A multimedia player that supports the video stream from the N6 camera module. A player that has all the necessary codecs, and works out-of-the-box is [POT Player](https://potplayer.daum.net/).

## 3. STM32N6 Hardware Setup
<details>
  <summary>Board Components Reference</summary>
  <img src="../media/stm32n6_board_overview.jpg" width="1000">
</details>

* Connect the camera via the white ribbon cable to the board (the blue stripe should be facing up on both ends).
* Change the Power Input Select Header (JMP2) from the 1-2 position to the 3-4 position to allow power delivery from both USB-C ports.
* Connect two USB type-C cables from each side of the board to the PC.
* Move the **BOOT1** switch to the right position to enter the "Development Boot" mode.
* Press the **RESET** button.

## 4. Programming the board

### 4.1 Setup the Programmer
* Launch the **STM32CubeProgrammer**.
* In the left menu, click the `External loaders` ("EL" icon).
* Use the search box to find `STM32N6570-DK` and select the check box to the left.
* In the upper right corner, change the "connection type" to `ST-LINK`.
* Just below, change the ST-LINK configuration "mode" to `Hot Plug`
* Click `Connect`

### 4.2 Program the Binaries
* In the left menu, click the `Erasing & programming` to program the (3) binaries.
#### 4.2.1 Program Bootloader
The **Bootloader** initializes the hardware, validates the firmware the enforces secure boot.
* Click `Browse` and navigate to and select `ai_fsbl.hex` previously extracted.
* Click `Start Programming` and wait until complete.
#### 4.2.2 Program the AI Model
The **AI Model** contains optimized model data for Neural-ART accelerator.
* Click `Browse` and navigate to and select `network_data.hex` previously extracted.
* Click `Start Programming` and wait until complete.
#### 4.2.3 Program the Application
The **Application** implements the core functionalities of the demo such as the AI inference and camera processing.  This is a custom signed image of the UVC application.
* Click `Browse` and navigate to and select `uvc_signed.bin` previously extracted.
* In the `Start Address` field, enter the following address:
```
0x70100000
```
* Click `Start Programming` and wait until complete.
* When programming has completed, move the **BOOT1** switch back to the **LEFT** position to boot from the External Flash just programmed.
* **Disconnect** and **Close** the programmer.
* Press the **RESET** button.

## 5. Cloud Account Setup
An /IOTCONNECT account with AWS backend is required.  If you need to create an account, a free trial subscription is available and no credit card is required.

[/IOTCONNECT Free Trial (AWS Version)](https://subscription.iotconnect.io/subscribe?cloud=aws)

> [!NOTE]
> Be sure to check your SPAM folder for the temporary password after registering if you don't see it after a couple minutes.

See the /IOTCONNECT [Subscription Information](https://github.com/avnet-iotconnect/avnet-iotconnect.github.io/blob/main/documentation/iotconnect/subscription/subscription.md) for more details on the trial including message limits.

## 6. Create a Device Template in /IOTCONNECT
A Device Template defines the characteristics of a piece of hardware include the telemetry the platform should expect to receive and any support commands.
* Download the premade device template [n6uvc_template.JSON](n6uvc_template.JSON?raw=1) (**MUST** Right-Click and "Save-As" to get the raw json file)
* Log-in the [console.iotconnect.io](https://console.iotconnect.io/login) using the credentials received when registering. 
* Import the template into your /IOTCONNECT instance. (A guide on [Importing a Device Template](https://github.com/avnet-iotconnect/avnet-iotconnect.github.io/blob/main/documentation/iotconnect/import_device_template.md) is available or for more information, please see the [/IOTCONNECT Documentation](https://docs.iotconnect.io/iotconnect/) website.)

## 7. Create a Device /IOTCONNECT
1. Click the `Device` icon and the "Device" sub-menu
2. At the top-right, click on the `Create Device` button
3. Enter `MySTN6` for both **Unique ID** and **Device Name**
4. Select the entity in the drop-down (if this is a new/trial account, there is only one option)
5. Select the template `n6uvc` from the template dropdown box
6. Leave the Device Certificate as "Auto-generated"
7. Click `Save & View`
9. Click the link for `Connection Info` and then the icon in the top-right and save the file "MySTN6-certificates.zip" into your working directory. These certificates will be written to the PMOD in a subsequent section.

## 8. Obtain /IOTCONNECT Connection Info
* The Company ID (`CPID`) and Environment (`ENV`) variables identifying your unique /IOTCONNECT account will be used to instruct the PMOD how to connect to the platform. This information is located in the `Settings` -> `Key Vault` section of the platform.
* Save these two values for use in the next section.
  
## 9. Flash/Configure the DA16k PMOD module
Follow the instructions in the [DA16K QuickStart Guide](https://github.com/avnet-iotconnect/iotc-dialog-da16k-sdk/blob/main/doc/QUICKSTART.md) to flash the DA16k PMOD then return to this guide.  This guide will walk through flashing the PMOD and entering the /IOTCONNECT connection information and device certificates.


## 10. Run the Applicaton
* Power off the board by unplugging the USB cables.
* Connect the DA16k PMOD module to the board's STMod+ Connector via the male-to-female jumper wires. (See the [STMod+ Technical note](https://www.st.com/resource/en/technical_note/tn1238-stmod-interface-specification-stmicroelectronics.pdf) and the [Digilent Pmod™ Interface Specification](https://digilent.com/reference/_media/reference/pmod/pmod-interface-specification-1_2_0.pdf) for more information on pinouts).

| PMOD pin # | STMOD+ pin # |             
|:----------:|:------------:| 
|   2 (RX)   | 2 (UARTy_TX) | 
|   3 (TX)   | 3 (UARTy_RX) | 
|  5 (GND)   |   16 (GND)   |
|  6 (+5v)   |   6 (+5v)    |

* Reconnect the USB cables to the board.
* Launch the camera application (e.g. [POT Player](https://potplayer.daum.net/)) on the PC to begin streaming the video.

> [!NOTE]
> The video stream needs to be playing for the inferencing to be enabled.

The multimedia player should now be displaying bounding boxes around objected detected in the field of view of the camera module.

## 11. Verify Data stream and Import a Dashboard
* Switch back to the /IOTCONNECT browser window and verify the device status is displaying as `Connected`
* Download the demo [Dashboard Template](STM32N6_UVC_Demo_dashboard.json?raw=1) (**must** Right-Click, Save As)

* **Download** the template then select `Create Dashboard` from the top of the page
* **Select** the `Import Dashboard` option and select `n6uvc` for **template** and `MySTN6` for **device** 
* **Enter** a name (such as `MyN6 Demo Dashboard`) and complete the import

You will now be in the dashboard edit mode. You can add/remove widgets or just click `Save` in the upper-right corner to exit the edit mode.
## 12. Using the Demo
* 

## Resources
* [Webinar Slides](TBD)
* [Purchase the STM32N6570-DK](TBD)
* [STM32N6570-DK Product Overview](https://www.st.com/en/evaluation-tools/stm32n6570-dk.html)
* [Other /IOTCONNECT enabled ST Devices](https://www.avnet.com/iotconnect/st)
* [/IOTCONNECT Overview](https://www.iotconnect.io/)
* [/IOTCONNECT Knowledgebase](https://help.iotconnect.io/)
