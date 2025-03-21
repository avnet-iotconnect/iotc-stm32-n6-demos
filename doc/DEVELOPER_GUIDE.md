# Developer Guide for STM32N6 AI Projects and IoTConnect

## Overview
Using the Applications Examples from the [STM32N6 AI Ecosystem website](https://www.st.com/en/development-tools/stm32n6-ai.html), Getting started example codes
This guide will help you set up, build, and run AI-based demos on the STM32N6 microcontroller using STM32CubeIDE. The available projects include:  

- n6-ai-getstarted (1.0.0):  Getting started code examples for various AI use cases (audio and computer vision).- 
   - **Image Classification**
   - **Object Detection**
   - **Instance Segmentation**
   - **Pose Estimation**
   - **Semantic Segmentation**
- n6-ai-h264-uvc (1.0.0):  Multimedia application example with AI, H264 encoding and USB UVC streaming.
   - **UVC Object Detection**
- n6-ai-pose-estim (1.0.0): RTOS-based application example for multi pose estimation with AI.

Each project follows a similar setup process with slight variations for specific requirements. This guide will help you choose the right project and configure it with IoTConnect.

---

## Prerequisites

Before starting, ensure you have the following installed on your system:

- **STM32CubeIDE** (latest version) - [Download here](https://www.st.com/en/development-tools/stm32cubeide.html)
- **STM32CubeProgrammer** (for flashing firmware) - [Download here](https://www.st.com/en/development-tools/stm32cubeprog.html)
- **Git** (for cloning the repository) - [Download here](https://git-scm.com/)
- **STM32 N6 Development Board** (such as STM32N6570-DK)
- **USB Cable** (for power and debugging)

---

## Step 1: Select and Download the Required AI Projects

Depending on your project choice, follow the corresponding steps:

### Option 1: Getting Started AI Projects

These projects are ideal for AI applications running directly on the STM32N6 microcontroller:

1. Sign in to your **STMicroelectronics** account.
2. Download the **n6-ai-getstarted** package from the [ST AI software page](https://www.st.com/en/development-tools/stm32n6-ai.html#st-get-software).
3. Extract the package to a local directory. The following AI demo projects will be available:
   - **Image Classification** - Identifies objects in an image and assigns them to predefined categories.
   - **Object Detection** - Detects multiple objects within an image, providing bounding boxes and labels.
   - **Instance Segmentation** - Segments individual objects in an image at the pixel level.
   - **Pose Estimation** - Detects key points on a human body to track movement.
   - **Semantic Segmentation** - Classifies every pixel in an image for scene understanding.

### Option 2: UVC Object Detection

This project builds upon the **Object Detection** model but instead of displaying bounding boxes on the onboard LCD, it streams the output as a **USB Video Class (UVC) video stream**. The processed video, encoded using **H.264**, can be viewed on a PC using a media player like **ffplay** or **VLC**.

1. Sign in to your **STMicroelectronics** account.
2. Download the **n6-ai-h264-uvc** package from the [ST AI software page](https://www.st.com/en/development-tools/stm32n6-ai.html#st-get-software).
3. Extract the package to a local directory. This project performs real-time object detection and encodes the output to be streamed over USB.

---

## Step 2: Clone the IoTConnect Repository

Clone the Avnet IoTConnect repository to integrate IoT functionality:

```sh
git clone https://github.com/avnet-iotconnect/iotc-stm32-n6-demos.git
```

---

## Step 3: Locate the Demo Projects in the Extracted Package

After extracting the package, navigate to the correct folder for your chosen demo project. Below are the paths for each available demo:

### AI Projects (from `n6-ai-getstarted` package)
#### Image Classification
```
<your_path>/en.n6-ai-getstarted-v1.0.0/application_code/image_classification/STM32N6/STM32CubeIDE
```

#### Object Detection
```
<your_path>/en.n6-ai-getstarted-v1.0.0/application_code/object_detection/STM32N6/STM32CubeIDE
```

#### Instance Segmentation
```
<your_path>/en.n6-ai-getstarted-v1.0.0/application_code/instance_segmentation/STM32N6/STM32CubeIDE
```

#### Pose Estimation
```
<your_path>/en.n6-ai-getstarted-v1.0.0/application_code/pose_estimation/STM32N6/STM32CubeIDE
```

#### Semantic Segmentation
```
<your_path>/en.n6-ai-getstarted-v1.0.0/application_code/semantic_segmentation/STM32N6/STM32CubeIDE
```

### UVC Object Detection (from `n6-ai-h264-uvc` package)
```
<your_path>/x-cube-n6-ai-h264-usb-uvc-v1.0.0/STM32CubeIDE
```

> **Note:** Replace `<your_path>` with the actual location where you extracted the package.

Proceed to the next step to open and configure the selected project in **STM32CubeIDE**.


---

## Step 4: Configure IoTConnect Integration

### 1. Clone and Add IoTConnect FreeRTOS Library
Clone the IoTConnect FreeRTOS library and copy it into your project directory:
```sh
git clone https://github.com/avnet-iotconnect/iotc-freertos-da16k-atcmd-lib.git
```
Move the `iotc-freertos-da16k-atcmd-lib` folder into:
```
<project_root>/STM32CubeIDE/Middlewares/
```

### 2. Exclude `da16k_platform_ra6mx.c` from the Build
In **STM32CubeIDE**:
- Open the **Project Explorer**.
- Navigate to `iotc-freertos-da16k-atcmd-lib/src`.
- Right-click `da16k_platform_ra6mx.c` > **Resource Configurations** > **Exclude from Build**.
- Select all configurations and click **OK**.

### 3. Modify `da16k_comm.h`
Open `iotc_da16k_pmod/da16k_comm.h` and comment out line 19:
```c
// #error "Please define DA16K_CONFIG_FILE!"
```

### 4. Copy HAL Driver Files
Copy the following files:
- `stm32n6xx_hal_dma.c`
- `stm32n6xx_hal_uart.c`
- `stm32n6xx_hal_uart_ex.c`

From:
```
/STM32N6/STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Src/
```
To:
```
<project_root>/STM32CubeIDE/Drivers/STM32N6xx_HAL_Driver/Src/
```

### 5. Add Include Paths in STM32CubeIDE
- Right-click your project in **Project Explorer** > **Properties**.
- Navigate to **C/C++ Build** > **Settings** > **MCU GCC Compiler** > **Include Paths**.
- Click **Add**, then add:
  - `${ProjDirPath}/STM32CubeIDE/Middlewares/iotc-freertos-da16k-atcmd-lib`
  - `${ProjDirPath}/STM32CubeIDE/Drivers/STM32N6xx_HAL_Driver/Inc`
  - `${ProjDirPath}/STM32CubeIDE/Application/Inc`
- Click **Apply and Close**.

### 6. Replace Project-Specific Files
For each project, replace the necessary files from the cloned IoTConnect repo:

**For AI Projects (`image_classification`, `object_detection`, etc.):**
- Replace `main.c` with:
  ```
  ???
  ```
- Add `da16k_uart.c` to:
  ```
  ??? c/
  ```

**For UVC Object Detection:**
- Replace `main.c` with:
  ```
    ???
  ```
- Add `da16k_uart.c` and `app.c` to:
  ```
  <project_root>/STM32CubeIDE/Application/Src/
  ```

This completes the IoTConnect integration setup. Proceed to **Step 5** for flashing and testing the application!


---

## Step 5: Build and Flash the Firmware

1. Click **Project > Build Project** or press **Ctrl+B**.
2. Wait for the build to complete.
3. Connect the STM32 board via **USB**.
4. Open **STM32CubeProgrammer**.
5. Click **Connect** to detect the board.
6. Select the `.bin` or `.hex` file from the `Debug/` or `Release/` folder.
7. Click **Start Programming**.
8. Reset the board to start execution.

---

## Step 6: Setting Up IoTConnect

1. Create an **IoTConnect Device Template**:

   - Download the [Device Template](n6uvc_template.JSON).
   - Import the template in IoTConnect under **Device > Templates**.

2. Create an **IoTConnect Device**:

   - Navigate to **Devices > Create Device**.
   - Use the **DUID** saved from earlier.
   - Select the correct template from the dropdown.
   - Download the device credentials.
   - Obtain **CPID** and **ENV** values from IoTConnect settings.

3. Flash and Configure the DA16K PMOD:

   - Follow the [QuickStart Guide](https://github.com/avnet-iotconnect/iotc-dialog-da16k-sdk/blob/main/doc/QUICKSTART.md).

---

## Step 7: Debugging and Running the Application

1. In **STM32CubeIDE**, click **Run > Debug Configurations**.
2. Select **STM32 Cortex-M C/C++ Application**.
3. Click **Debug** to start debugging.
4. Use **breakpoints** and **console output** to troubleshoot issues.

To monitor logs, use a serial terminal (e.g., PuTTY, Tera Term) at **115200 baud rate**.

---

## Troubleshooting

- **Build Errors:** Ensure all source files and include paths are correctly set in **Project Properties**.
- **Board Not Detected in STM32CubeProgrammer:** Try reconnecting the USB cable or switching USB ports.
- **Flashing Issues:** Ensure the board is in **DFU mode** by holding the **BOOT0** button while resetting.

---

## Additional Resources

- [STM32CubeIDE User Guide](https://www.st.com/resource/en/user_manual/um2609-getting-started-with-stm32cubeide-stmicroelectronics.pdf)
- [STM32N6 Series Overview](https://www.st.com/en/microcontrollers-microprocessors/stm32n6-series.html)
- [IoTConnect Documentation](https://developer.iotconnect.io/)

---

## License

This project is licensed under the **STMicroelectronics License Agreement**. See the [LICENSE](../LICENSE) file for details.

---

## Contributors

- STMicroelectronics
- Avnet IoTConnect Team

For support, open an issue on [GitHub](https://github.com/avnet-iotconnect/iotc-stm32-n6-demos/issues).

