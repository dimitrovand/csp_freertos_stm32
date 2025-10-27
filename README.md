# CSP with FreeRTOS on STM32H753
A simple project to build `libcsp` with `FreeRTOS` on an `STM32H753` microcontroller.
## How to Build
### 1. Prerequisites
Ensure the following tools are installed on your system:
```bash
sudo apt install gcc-arm-none-eabi cmake ninja
```
### 2. Clone the repository:
```bash
git clone --recursive <this repository url>
```

### 3. Build:
From within the project root directory:
```bash
cmake -S ./ -B build -G Ninja && ninja -C build
```
# Simulating with Renode
## Prerequisites
Make sure Renode in the project's root directory:
```bash
wget https://github.com/renode/renode/releases/download/v1.15.3/renode-1.15.3.linux-portable.tar.gz
mkdir renode && tar -xzf ./renode-1.15.3.linux-portable.tar.gz -C renode --strip-components=1
```
## Run
From within the project root directory, run the following command to start the simulation:
```bash
./renode/renode --disable-gui stm32h753.resc
```
This will create the emulated STM32H753 device and set up a virtual port that can be accessed.