# Wardriver

[![Arduino Build](https://github.com/lukeswitz/Wardriver/actions/workflows/build.yml/badge.svg)](https://github.com/lukeswitz/Wardriver/actions/workflows/build.yml)

Fork Details: 
- Duplication of networks
- Minor fixes and updates 

**Basic wardriving for the ESP8266 & ESP32, compatible with WiGLE.**

*This guide will help flash the firmware to your device*

## Prerequisites
- ESP8266 or ESP32  
- GPS module (nearly any will work)
- Docker and esptool:

**Install Docker:**
  
  `sudo apt update`

  `sudo apt upgrade`

  `sudo apt install docker.io`

**Install esptool**:
  
  - `pip install esptool` on Mac/Linux.

- Use the [official espressif releases] of esptool (https://github.com/espressif/esptool/releases) for other operating systems.

## Setup

1. Clone this repository to your local machine.

```bash
git clone https://github.com/lukeswitz/Wardriver.git
cd Wardriver/src
```

2. Build the Docker image.

```bash
make build
```

This will create a Docker image with all the necessary tools and libraries to compile the firmware.

## Flashing the Firmware

Before flashing the firmware, make sure your ESP8266 or ESP32 board is connected to your computer.

### 1. ESP32 ONLY: Enter DFU Mode

You can do this in 2 ways:

**Option 1:**

- Plug the board into your computer
- Hold down the 0 button
- Tap & Release the RST button

**Option 2:**

- Hold down the 0 button
- Plug in the board
- Release the 0 button

### Step 2: Find Serial Port & Erase Flash 

Find the Serial Port your board is connected to:

- Windows: Found via Device Manager
- MacOS: `ls /dev/cu*`
- Linux: `ls /dev/tty*`

Your device will appear as something like `ttyUSB0` or `ttyACM0`.

Erase the flash:

```bash
esptool --chip auto -p /dev/<your_serial_port> erase_flash
```

### Step 3: Flashing 
Decide what image you want to flash :
- Official Beta: `/build/v1.2-beta-WarDriver.bin`
- This pre-beta build (unstable): `/build/v1.2-alpha-WarDriver.bin`

Run the following command to flash the firmware to your board:

`esptool --chip auto -p <SERIAL_PORT> write_flash -z 0x0 <binary>.bin`

Replace `<binary>.bin` by dragging the file into the terminal or by manually specifying from the above paths. 

## Listing All Supported Boards

To list all the supported boards by esptool, you can check the help file:

```bash
esptool -h
```

## Troubleshooting

If you encounter any issues, make sure your board is properly connected and the correct port is specified in the `esptool` command. If the problem persists, please open an issue in this repository.

## Contributing

Contributions are welcome! Please open a pull request with your changes or improvements.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details. 

## Credits

All credit and thanks to Alex Lynd. Docs modded from the DNSDriveBy/Nugget flashing instructions. 
