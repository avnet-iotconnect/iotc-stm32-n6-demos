# Getting Started with STM32N6 AI Projects and /IOTCONNECT

## Overview

This guide will help you set up, build, and run AI-based demos on the STM32N6 microcontroller using STM32CubeIDE. The available projects include:

- **Image Classification** - Identifies objects or patterns in an image and assigns them to predefined categories.
- **Object Detection** - Detects and localizes multiple objects within an image, providing bounding boxes and labels.
- **Instance Segmentation** - Distinguishes between individual objects in an image by segmenting them at the pixel level.
- **Pose Estimation** - Detects and tracks human body key points, such as joints, for motion analysis.
- **Semantic Segmentation** - Classifies each pixel in an image into categories for scene understanding.
- **UVC Object Detection** - Performs object detection using a USB video camera (UVC) and encodes the output using H.264, streaming the processed video to a PC for real-time viewing.
- **Hand Landmark Detection** - Detects and tracks the position of hands and fingers in real-time.
- **Person Detection** - Identifies and tracks people in a given scene.
- **Pose Estimation with Advanced Tracking** - Expands upon basic pose estimation for improved movement tracking.

Each project follows a similar setup process with slight variations for specific requirements. This guide will help you choose the right project and configure it with /IOTCONNECT.

---

## Prerequisites

Before starting, ensure you have the following installed on your system:

- **STM32CubeIDE** (latest version) - [Download here](https://www.st.com/en/development-tools/stm32cubeide.html)
- **STM32CubeProgrammer** (for flashing firmware) - [Download here](https://www.st.com/en/development-tools/stm32cubeprog.html)
- **Git** (for cloning the repository) - [Download here](https://git-scm.com/)
- **STM32 N6 Development Board** ([Avnet](https://www.avnet.com/shop/us/products/stmicroelectronics/stm32n6570-dk-3074457345660283716?krypto=ioIhy%2BDNj7l%2BrLC6GjpppJ%2F5ZuSnZAnX2S%2Bk8OwfPfSxs301gtDA5pkEkdc1a0g2P5MjM0kmKKvjJQNMwbLNCIx8YMBTXuX7ByIlCM%2FwmkdkYzWiUUoiCow%2Fay4rjZjV), [Newark](https://www.newark.com/stmicroelectronics/stm32n6570-dk/discovery-board-32bit-arm-cortex/dp/20AM4437))
- **USB Cable** (for power and debugging)

### Additional Software for UVC Object Detection

- Request and download the **n6-ai-h264-uvc** software package from the [ST AI Software Ecosystem](https://www.st.com/en/development-tools/stm32n6-ai.html#st-get-software).

---

## Step 1: Select and Download the Required AI Projects

Depending on your project choice, follow the corresponding steps:

### Option 1: Getting Started AI Projects

These projects are ideal for AI applications running directly on the STM32N6 microcontroller:

1. Sign in to your **STMicroelectronics** account.
2. Download the **n6-ai-getstarted** package from the [ST AI software page](https://www.st.com/en/development-tools/stm32n6-ai.html#st-get-software).
3. Extract the package to a local directory. The following AI demo projects will be available:
   - **Image Classification**
   - **Object Detection**
   - **Instance Segmentation**
   - **Pose Estimation**
   - **Semantic Segmentation**

### Option 2: Advanced AI Projects

These projects are specialized AI applications that enhance object detection and tracking capabilities:

1. Download the additional AI packages from the **ST AI Software Ecosystem**.
2. Extract the package to a local directory. The following AI demo projects will be available:
   - **n6-ai-h264-uvc** - Streams object detection results via USB UVC with H.264 encoding.
   - **n6-ai-hand-land** - Detects and tracks hand and finger movements.
   - **n6-ai-pdetect** - Detects people in various environments.
   - **n6-ai-pose-estim** - Advanced pose estimation for improved tracking.

---

## Step 1.5 (Optional): Install ST Edge AI Tools & Generate the Model
> **Only needed if your chosen project does NOT already include a pre-compiled model (e.g., `network.c`, `network_ecblobs.h`) in the `st_ai_output` folder.**  
> **Skip this step if pre-compiled model files are already provided.**

1. **Install ST Edge AI Tools**
   - Go to the [ST STM32N6 AI Tools page](https://www.st.com/en/development-tools/stm32n6-ai.html#st-get-software) and download the **ST Edge AI** package (e.g., `v2.0.0`) for your OS (Linux `.deb` or `.sh`, etc.).
   - Install it according to ST’s instructions (e.g., `sudo dpkg -i steai-2.0.0.deb`).

2. **Add the Tools to Your PATH**
   - Typically the tools install to `/opt/ST/STEdgeAI/2.0/Utilities/linux`.
   - Append this folder to your user’s `PATH` by adding the following line to `~/.bashrc` (or `~/.zshrc`):
     ```bash
     export PATH="$PATH:/opt/ST/STEdgeAI/2.0/Utilities/linux"
     source ~/.bashrc
     ```

3. **Fix Permissions (If Needed)**
   - Grant read and directory-traverse access so your normal user can read `.mdesc` and other files:
     ```bash
     sudo chmod -R a+rX /opt/ST/STEdgeAI/2.0
     ```
   - Make sure executables (like `atonn` or `stedgeai`) are also executable:
     ```bash
     sudo chmod +x /opt/ST/STEdgeAI/2.0/Utilities/linux/atonn
     sudo chmod +x /opt/ST/STEdgeAI/2.0/Utilities/linux/stedgeai
     ```

4. **Generate the Model (If Your Script Requires It)**
   - Inside your project’s `Model` folder, you may find a script like `generate-n6-model.sh`. Run it as a normal user:
     ```bash
     ./generate-n6-model.sh
     ```
   - If everything is correct, the ST Edge AI compiler (`atonn`) will produce:
     - `network.c`
     - `network_ecblobs.h`
     - Additional binary/headers as needed
   - **Troubleshooting**:
     - If you see “permission denied,” confirm you’ve done the permission fixes above.
     - If you see “unsupported layer” (e.g., `ConvTranspose`), you’ll need a different model architecture or a newer AI tool version.

Once these steps complete successfully, you’ll have the optimized model files in the `st_ai_output` (or equivalent) folder. You can then move on to the normal project build steps.

---

## Step 2: Clone the /IOTCONNECT Repository
Clone the /IOTCONNECT FreeRTOS library and copy it into your project directory:
```sh
git clone https://github.com/avnet-iotconnect/iotc-freertos-da16k-atcmd-lib.git
```
Move the `iotc-freertos-da16k-atcmd-lib` folder into:
```
<project_root>/STM32CubeIDE/Middlewares/
```

## Step 3: Open Project in STM32CubeIDE

Open STM32CubeIDE and import your chosen project:

**Basic AI Projects (`n6-ai-getstarted`):**

- **Image Classification**
```bash
<your_path>/en.n6-ai-getstarted-v1.0.0/application_code/image_classification/STM32N6/STM32CubeIDE
```

- **Object Detection**
```bash
<your_path>/en.n6-ai-getstarted-v1.0.0/application_code/object_detection/STM32N6/STM32CubeIDE
```

- **Instance Segmentation**
```bash
<your_path>/en.n6-ai-getstarted-v1.0.0/application_code/instance_segmentation/STM32N6/STM32CubeIDE
```

- **Pose Estimation**
```bash
<your_path>/en.n6-ai-getstarted-v1.0.0/application_code/pose_estimation/STM32N6/STM32CubeIDE
```

- **Semantic Segmentation**
```bash
<your_path>/en.n6-ai-getstarted-v1.0.0/application_code/semantic_segmentation/STM32N6/STM32CubeIDE
```

**Advanced AI Projects:**

- **UVC Object Detection**
```bash
<your_path>/x-cube-n6-ai-h264-usb-uvc-v1.0.0/STM32CubeIDE
```

- **Hand Landmark Detection**
```bash
<your_path>/n6-ai-hand-land/STM32CubeIDE
```

- **Person Detection**
```bash
<your_path>/n6-ai-pdetect/STM32CubeIDE
```

- **Pose Estimation with Advanced Tracking**
```bash
<your_path>/n6-ai-pose-estim/STM32CubeIDE
```

Replace `<your_path>` with the actual extraction path.

---

## Step 4: Configure /IOTCONNECT Integration

In STM32CubeIDE:

- **Exclude** `da16k_platform_ra6mx.c` (Right-click → Resource Configurations → Exclude from Build).
- Edit `da16k_comm.h` to comment line 19:
```c
// #error "Please define DA16K_CONFIG_FILE!"
```

- Copy HAL driver files:
  - `stm32n6xx_hal_dma.c`
  - `stm32n6xx_hal_uart.c`
  - `stm32n6xx_hal_uart_ex.c`

From:
```bash
/STM32N6/STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Src/
```

To:
```bash
<project_root>/STM32CubeIDE/Drivers/STM32N6xx_HAL_Driver/Src/
```

- Add Include Paths (Project → Properties → C/C++ Build → Settings → Include Paths):

```bash
${ProjDirPath}/STM32CubeIDE/Middlewares/iotc-freertos-da16k-atcmd-lib
```

- Replace/Add /IOTCONNECT-specific files:
  - Replace `main.c` with:
```bash
iotc-stm32-n6-demos/Projects/<project_name>/STM32N6/Src/main.c
```

  - Add `da16k_uart.c` (and optionally `app.c` for advanced demos) to:
```bash
<project_root>/STM32CubeIDE/Application/Src/
```

---

## Step 5: Running from RAM or Flash

### Option 1: Running from RAM
- Run → Debug Configurations in STM32CubeIDE.
- Ensure Debugger configuration is set for RAM execution.
- Click **Debug** to start.

### Option 2: Signed Binary for Flash Execution
- Download [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html).
- Sign binary:
```bash
STM32_SigningTool_CLI -bin firmware.bin -nk -t ssbl -hv 2.3 -o firmware_sign.bin
```
- Flash with STM32CubeProgrammer to address `0x70100000`.

---

## Troubleshooting

- **Build errors:** Verify include paths and source files.
- **STM32CubeProgrammer connection issues:** Reconnect USB or ensure DFU mode (BOOT0 pressed during reset).
- **Permission issues:** Confirm permissions set (Step 1.5).

---
