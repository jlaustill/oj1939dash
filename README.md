# oj1939dash (Open Source J1939 Dash)

A modern dashboard solution for vehicles using the J1939 protocol, specifically designed for Cummins-powered vehicles. This project provides a clean, efficient interface to display vehicle data using a Nextion display.

## Features
### Working
* J1939 protocol support for Cummins engine data
* Nextion display output
* Persistent data storage using FRAM
* Real-time display of:
  * Engine RPM
  * Coolant Temperature
  * Oil Pressure
  * Fuel Temperature
  * Boost Pressure
  * Manifold Temperature
  * Transmission Temperature
  * Vehicle Speed
  * Gear Selection
  * And more...

## Hardware
This project uses:
* Teensy 4.1
* SN65HVD230 CAN Board
* Nextion Display
* FRAM Memory Module

### Pins
* 0: Cummins Bus RX
* 1: Cummins Bus TX
* 7: Nextion RX (RX2)
* 8: Nextion TX (TX2)

## Configuration
Configuration is handled through `Configuration.h`, allowing you to enable/disable features as needed.

## Building
This project uses PlatformIO for building and deployment. To build:
1. Install PlatformIO
2. Clone this repository
3. Open in your preferred IDE with PlatformIO support
4. Build and upload to your Teensy

## License
This project is open source and available under the MIT License.