## Quick guide

### Brief steps:
* Clone this repository.
* Download and unzip the [firmware packs](https://www.st.com/en/development-tools/stm32n6-ai.html#get-software) from ST. (n6-ai-getstarted and n6-ai-h264-uvc)

* for n6-ai-h264-uvc project,
  * Copy/replace this repo's *Projects/uvc/x-cube-n6-ai-h264-usb-uvc-v1.0.0* folder to the firmware pack *x-cube-n6-ai-h264-usb-uvc-v1.0.0* directory.
  * Copy this repo's *lib/iotc-freertos-da16k-atcmd-lib* folder to the firmware pack *x-cube-n6-ai-h264-usb-uvc-v1.0.0/Lib/* directory.
* for n6-ai-getstarted project,
  * Copy/replace this repo's Project/object_detection folder to the firmware pack *en.n6-ai-getstarted-v1.0.0/application_code/object_detection* directory.
  * Copy this repo's lib/iotc-freertos-da16k-atcmd-lib folder to the firmware pack *en.n6-ai-getstarted-v1.0.0/application_code/object_detection/STM32N6/Middlewares* directory.

* Create a workspace folder in a separate directory.
* Launch STM32CubeIDE(1.17.0) and import the project.
