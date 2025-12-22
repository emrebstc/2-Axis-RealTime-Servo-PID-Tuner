***************************************************
## Dual-Axis Servo Control System with Real-Time PID Tuner and Telemetry

This project implements a high-precision, real-time control system for a dual-axis (X-Y / - Pan/Tilt) servo platform using the STM32F4 (ARM Cortex-M4) architecture and FreeRTOS. The system stabilizes the platform by processing feedback from a dual-channel ADC (utilizing the servos' internal potentiometers), applying a Low-Pass Filter (LPF) to reduce noise, and calculating precise duty cycles via a dedicated PID control loop.

Key features include a multi-threaded task architecture where control loops, sensor data acquisition, and communications are managed independently to deliver low-latency performance. The project also features a robust UART Command Interface, allowing users to dynamically update target setpoints from a terminal—via a custom MATLAB-based desktop application—while receiving real-time telemetry data, including current angles, setpoints, and PID coefficients.

## Core Highlights:

* *Remote Control*: Dedicated MATLAB desktop app and GUI for real-time setpoint tuning and performance monitoring created by me.

* *RTOS Integration*: Multi-threaded architecture for deterministic task execution and low latency.

* *Smart Telemetry*: Continuous UART feedback of system states (angles, PWM, PID gains).

* *Signal Integrity*: Integrated Low-Pass Filtering to ensure stable feedback in noisy environments.

***************************************************
[![Watch the video](https://img.youtube.com/vi/9InCc37I-D4/maxresdefault.jpg)](https://youtu.be/9InCc37I-D4)
![2](https://github.com/user-attachments/assets/126672b5-4563-4448-8c12-04af85c72059)
<img width="2558" height="1598" alt="Ekran görüntüsü 2025-12-22 180415" src="https://github.com/user-attachments/assets/364e26d0-7946-4df5-b734-74a6ab42a213" />
<img width="2558" height="1560" alt="Ekran görüntüsü 2025-12-22 181548" src="https://github.com/user-attachments/assets/7e3c4717-508c-4d0d-ab6e-4b18d1da1842" />
<img width="2558" height="1571" alt="Ekran görüntüsü 2025-12-22 181837" src="https://github.com/user-attachments/assets/3d470b48-1738-4a01-9459-910a8cf1f7c3" />
<img width="400" height="400" alt="splash" src="https://github.com/user-attachments/assets/6b9695b7-f362-4099-bebd-62a9337eeda8" />

