# IMU-Based 3D Flight Visualization System

## Overview
This project implements a real-time 3D visualization of an aircraft model using an STM32L475 microcontroller equipped with accelerometer and gyroscope sensors (IMU). Sensor data is fused with a complementary filter to produce smooth roll, pitch, and yaw angles. Data is transmitted via UART to a Python 3D viewer built with PyQtGraph.

## Features
- Real-time acquisition of accelerometer and gyroscope data from STM32.
- Complementary filter for sensor fusion.
- Smooth 3D visualization of roll, pitch, and yaw.
- UART communication between STM32 and PC.
- Simple 3D airplane model rendered using PyQtGraph.

## Hardware
- STM32L475 IoT Discovery Board
- Built-in Accelerometer & Gyroscope

## Software
- STM32CubeIDE (C)
- Python 3, PyQtGraph

## How to Run
1. Flash the STM32 firmware.
2. Connect the board via UART to your PC (115200 baudrate).
3. Run `imu_3d_viewer.py` with Python 3.
4. Move the board and observe live 3D orientation changes.

## Screenshot
![IMG_5920](https://github.com/user-attachments/assets/b10cb6f5-cc19-436b-990d-3162b02103fb)
![IMG_5922](https://github.com/user-attachments/assets/278cfd0a-11b3-4a9d-b009-46eadb19437e)
![IMG_5921](https://github.com/user-attachments/assets/0d48d575-550a-40ba-9e98-2604cb2d10ca)


## License
MIT License
