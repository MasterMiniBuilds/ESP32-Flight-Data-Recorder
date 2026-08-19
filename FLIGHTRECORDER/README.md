# ESP32 Flight Recorder

A portable, standalone aviation flight data recorder built around an ESP32. The system records flight data from multiple sensors to a microSD card and provides status information through a small OLED display.

The project is designed for post-flight analysis. It records data during a flight. It does not control the aircraft or provide autopilot functionality.

## Features

- ESP32-based flight data recorder
- GPS position, altitude, speed, course and satellite data
- MPU6050 accelerometer and gyroscope
- BME280 temperature, pressure and barometric altitude
- MicroSD card flight-data recording
- CSV output for computer-side analysis
- 128×64 OLED display
- Physical start/stop recording button
- Module health monitoring
- Automatic BME280 reference-pressure calibration
- Recording continues if an individual sensor temporarily fails
- End-of-flight status screen
- Designed for portable operation
- No audio recording
- No aircraft control or autopilot functionality

## Hardware

The current project uses:

- ESP32-WROOM development board
- 128×64 I²C OLED display
- MPU6050 accelerometer/gyroscope
- BME280 temperature/pressure sensor
- u-blox NEO-6M GPS
- MicroSD card module
- MicroSD card
- Momentary pushbutton
- Portable battery supply

## Pin Configuration

| Component | Function | ESP32 Pin |
|---|---|---:|
| OLED | SDA | GPIO 25 |
| OLED | SCL | GPIO 26 |
| BME280 | SDA | GPIO 21 |
| BME280 | SCL | GPIO 22 |
| MPU6050 | SDA | GPIO 13 |
| MPU6050 | SCL | GPIO 14 |
| GPS | RX | GPIO 16 |
| GPS | TX | GPIO 17 |
| SD card | SCK | GPIO 18 |
| SD card | MISO | GPIO 19 |
| SD card | MOSI | GPIO 23 |
| SD card | CS | GPIO 32 |
| Start/Stop button | Input | GPIO 27 |

The OLED, BME280 and MPU6050 use separate I²C buses in the current implementation.

## Recording System

The recorder operates at different update rates depending on the task:

- **Sensor sampling:** 4 Hz
- **CSV recording:** 1 Hz
- **Module health checks:** every 2 seconds
- **Display updates:** every 3 seconds
- **Button debounce:** 60 ms
- **BME280 calibration:** 10 seconds

The recorder saves flight information as CSV data so it can be transferred to a computer after the flight.

## Flight Data

Depending on sensor availability, the recorded data includes:

### GPS

- Latitude
- Longitude
- GPS altitude
- Ground speed
- Course
- Number of satellites

### BME280

- Temperature
- Atmospheric pressure
- Calculated barometric altitude

### MPU6050

- X-axis acceleration
- Y-axis acceleration
- Z-axis acceleration
- X-axis gyroscope
- Y-axis gyroscope
- Z-axis gyroscope
- Total calculated G-force

The system does not invent replacement sensor values when a sensor is unavailable.

## Barometric Altitude

At startup, the BME280 performs a calibration period for approximately 10 seconds.

During calibration, pressure measurements are averaged to establish a reference pressure. This reference is then used to calculate relative barometric altitude.

If calibration cannot be completed, the recorder can fall back to a default reference pressure of:

**1013.25 hPa**

This allows recording to continue even if the BME280 has a problem during startup.

## OLED Display

The OLED cycles through several information screens.

### Module Status

Displays the current state of the main hardware modules, including:

- OLED
- MPU6050
- BME280
- GPS
- SD card

### Recording Status

Shows whether the flight recorder is currently recording and provides information about the recording system.

### Flight Data

Displays selected live flight information such as:

- GPS altitude
- Barometric altitude
- GPS speed
- Satellite count
- G-force

Additional temporary screens are used when starting or stopping a recording.

After a flight has been completed, the end-of-flight screen remains displayed until the system is powered off.

## Start/Stop Button

The physical button connected to GPIO 27 controls recording.

Pressing the button starts a new recording when the system is ready.

Pressing it again stops the recording and completes the flight.

The button uses software debouncing to prevent a single press from being detected multiple times.

## SD Card

Flight data is stored on the microSD card as CSV files.

The recorder continuously checks that the SD card is functioning and that data is successfully being written while recording.

A temporary sensor failure does not automatically stop the recording system.

## Computer-Side Analysis

The recorded CSV files are intended to be processed by a separate computer-side flight analysis program.

The planned analysis software can provide:

- Interactive flight map
- Altitude graphs
- Speed graphs
- G-force graphs
- Flight statistics
- GPS information
- Sensor information
- Raw CSV data access
- Professional flight-report presentation

The analysis program operates after the flight and does not control the ESP32 or aircraft.