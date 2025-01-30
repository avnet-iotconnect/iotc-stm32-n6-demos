# Getting Started with Image Classification on STM32N6

## Overview
This guide will help you set up, build, and run the Image Classification demo on an STM32N6 microcontroller using STM32CubeIDE. Follow the steps below to clone the repository, configure the project, and compile the firmware.

## Prerequisites
Before starting, ensure you have the following installed on your system:

- **STM32CubeIDE** (latest version) - [Download here](https://www.st.com/en/development-tools/stm32cubeide.html)
- **STM32CubeProgrammer** (for flashing firmware) - [Download here](https://www.st.com/en/development-tools/stm32cubeprog.html)
- **Git** (for cloning the repository) - [Download here](https://git-scm.com/)
- **STM32 N6 Development Board** (such as STM32N6570-DK)
- **USB Cable** (for power and debugging)

---
## Step 1: Clone the Repository
Open a terminal or command prompt and run:

```sh
git clone https://github.com/avnet-iotconnect/iotc-stm32-n6-demos.git
```

Navigate to the Image Classification project directory:

```sh
cd iotc-stm32-n6-demos/Projects/image_classification/STM32N6/STM32CubeIDE
```

---
## Step 2: Open the Project in STM32CubeIDE
1. Launch **STM32CubeIDE**.
2. Click on **File > Import**.
3. Select **Existing Projects into Workspace** and click **Next**.
4. Click **Browse** and navigate to the cloned repository:
   ```
   iotc-stm32-n6-demos/Projects/image_classification/STM32N6/STM32CubeIDE
   ```
5. Select the **image_classification** project and click **Finish**.

---
## Step 3: Configure Build Settings
1. Go to **Project > Properties**.
2. Select **C/C++ Build > Settings**.
3. Under **MCU GCC Compiler > Optimization**, ensure **-O2** or **-Os** is selected for an optimized build.
4. Check the **Include Paths** under **MCU GCC Compiler > Includes** and make sure the necessary paths (e.g., `Drivers/`, `Core/`, `Middlewares/`) are included.
5. Click **Apply and Close**.

---
## Step 4: Build the Project
To compile the firmware:

1. Click **Project > Build Project** or press **Ctrl+B**.
2. Wait for the build process to complete. The output binary (`.elf`, `.bin`, or `.hex`) will be generated inside the `Debug` or `Release` folder.

If the build is successful, you should see:
```sh
Build Finished. 0 errors, 0 warnings.
```

---
## Step 5: Flash the Firmware
To flash the compiled binary to your STM32N6 board:

1. Connect the board via **USB**.
2. Open **STM32CubeProgrammer**.
3. Click **Connect** to detect the board.
4. Click **Browse** and select the `.bin` or `.hex` file from the `Debug/` or `Release/` folder.
5. Click **Start Programming**.
6. Wait for the flashing process to complete.

Once done, reset the board to start execution.

---
## Step 6: Debugging the Application
1. In **STM32CubeIDE**, click **Run > Debug Configurations**.
2. Select **STM32 Cortex-M C/C++ Application**.
3. Click **Debug** to start debugging.
4. Use **breakpoints** and the **console output** to troubleshoot issues.

---
## Step 7: Running the Application
After flashing the firmware:
- The application will run automatically.
- If using a display, you should see image classification results on the screen.
- If connected via UART, you can monitor logs using a serial terminal (e.g., PuTTY, Tera Term) at **115200 baud rate**.

---
## Troubleshooting
- If **STM32CubeIDE fails to build**, check that all source files and include paths are correctly set in **Project Properties**.
- If the board is **not detected in STM32CubeProgrammer**, try reconnecting the USB cable or using another port.
- If **flashing fails**, ensure the board is in **DFU mode** by holding the **BOOT0** button while resetting.

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

