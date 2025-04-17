# STM32 AI-Powered LiDAR Anomaly Detector

![Build Status](https://github.com/cjl4945/AI-with-lidar/actions/workflows/build.yml/badge.svg)

his project uses an STM32L476 microcontroller and a TF-Luna LiDAR sensor to detect anomalies in distance measurements using a lightweight AI-based or rule-based anomaly detection algorithm.

## Overview

The system continuously captures high-frequency distance readings from the TF-Luna LiDAR sensor over UART (500Hz) and applies anomaly detection logic to identify unusual patterns. This could include environmental changes, obstructions, or faulty sensor behavior.

The firmware is designed for real-time performance on low-power embedded hardware, and the development workflow includes GitHub-based CI/CD for continuous integration and artifact delivery.

## Features

- High-speed UART data acquisition from TF-Luna LiDAR
- Simple anomaly detection logic (configurable threshold or ML-based)
- UART-based telemetry output for logging and monitoring
- STM32 HAL and CubeMX configuration
- CMake-based project structure
- GitHub Actions CI for automated build and artifact upload

## Anomaly Detection

The system currently supports basic threshold detection. Future versions may integrate a trained anomaly detection model using TinyML frameworks such as TensorFlow Lite for Microcontrollers or custom lookup logic.

## Hardware

- STM32L476RG (Nucleo-64 development board)
- TF-Luna LiDAR (UART interface, 5V logic level)
- USB or external power (5V supply)

## Project Structure

. ├── Core/ # STM32 user and auto-generated code ├── Drivers/ # HAL and CMSIS drivers ├── build/ # CMake output (not committed) ├── .github/workflows/ # CI pipeline ├── .vscode/ # VSCode debugging tasks and config ├── CMakeLists.txt # Main build script ├── MyProject.ioc # STM32CubeMX project configuration └── README.md

## CI/CD

The project uses GitHub Actions to:
- Build the firmware on every push or pull request to the `main` branch
- Upload the `.hex` file as a downloadable artifact
- Enable further extension for testing or deployment automation

## Future Work

- Add TinyML anomaly detection model integration
- Enable real-time data visualization via USB or Python serial logger
- Add parameter tuning via serial command interface
- Integrate unit testing and static analysis into CI workflow

## License

This project is licensed under the MIT License.
