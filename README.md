# [cite_start]A-NAV: Advanced Autonomous Navigation & Position Estimation System [cite: 3]

## [cite_start]1. Project Overview & Strategic Vision [cite: 2]
[cite_start]A-NAV (Advanced-Navigation) is an embedded flight assistance and positioning system designed to eliminate the absolute dependence of Unmanned Aerial Vehicles (UAVs) on external GNSS/GPS satellites[cite: 3, 4]. [cite_start]By combining low-cost MEMS sensors with advanced mathematical filtering algorithms on ARM-based microcontrollers, the system provides high-reliability "Dead Reckoning" capabilities under intense Electronic Warfare (Jamming/Spoofing) conditions[cite: 4, 5].

## [cite_start]2. Operational Phases [cite: 6]
[cite_start]The flight software dynamically manages the UAV's navigational continuity across two major operational modes[cite: 7]:
* [cite_start]**GNSS Active Phase:** The system acquires live GPS data over UART, continuously calibrates internal sensor noise using a Kalman Filter structure, and executes high-precision path tracking[cite: 8].
* [cite_start]**Jamming (GPS-Denied) Phase:** Upon detecting signal degradation or jammer interference, the system switches to **Dead Reckoning** within milliseconds[cite: 9]. [cite_start]It utilizes real-time acceleration and angular velocity streams to dynamically estimate the aircraft's current position and velocity relative to its last known coordinates[cite: 10].

## [cite_start]3. Hardware Architecture & Hardware-Software Interfacing [cite: 11]
[cite_start]The project is built on the **STM32F401RET6** MCU platform running at a high-performance core clock frequency of **84 MHz** via an external HSE oscillator, enabling real-time floating-point calculations at a stable **100 Hz** execution rate[cite: 26, 1067].

### [cite_start]Peripheral Mapping & Configuration[cite: 12, 1067]:
* [cite_start]**Microcontroller (MCU):** STM32F401RET6 (ARM Cortex-M4 with FPU)[cite: 12, 1067].
* [cite_start]**Inertial Measurement Unit (IMU):** MPU6050 (6-axis accelerometer + gyroscope) interfaced over **I2C1** configured in **Fast Mode (400 kHz)** using pins **PB8 (SCL)** and **PB9 (SDA)**[cite: 12, 1067].
* [cite_start]**GNSS / Telemetry Interface:** Connected via **USART2** asynchronous serial communication protocol on pins **PA2 (TX)** and **PA3 (RX)**[cite: 12, 1067].
* [cite_start]**Actuation / Motor Control:** **TIM3** is configured to generate **4-Channel PWM** signals (CH1 on PC6, CH2 on PC7, CH3 on PB0, CH4 on PB1) dedicated to driving Electronic Speed Controllers (ESCs) and adjusting motor outputs dynamically[cite: 1067].
* [cite_start]**High-Speed Interfacing:** **SPI1** configured in Full-Duplex Master Mode for low-latency peripheral data transfer alongside an integrated **SSD1306 OLED Display** for localized instrumentation telemetry[cite: 1067].

## [cite_start]4. Advanced Engineering Solutions & Algorithmic Design [cite: 13]
* [cite_start]**Quaternion-Based Gravity Compensation:** Sensors inherently register the constant $9.81 m/s^2$ gravitational pull[cite: 16]. [cite_start]A-NAV utilizes a quaternion rotation matrix to track orientation in 3D space, computationally isolating and subtracting the gravity vector to extract pure linear motion acceleration[cite: 16, 17].
* [cite_start]**Unscented Kalman Filter (UKF) Integration:** To overcome the limitations of standard linear filters during aggressive non-linear tactical maneuvers, a UKF utilizes sigma-point modeling to predict highly accurate states from noisy raw sensor data streams[cite: 19, 20].
* [cite_start]**Dynamic Bias & Drift Modeling:** Sensor-inherent biases are monitored and estimated online during active flight tracking[cite: 22]. [cite_start]This real-time calibration nullifies numerical integration drift, which is the primary challenge in long-term Dead Reckoning systems[cite: 23].

## [cite_start]5. Development Ecosystem [cite: 11]
* [cite_start]**Embedded Software:** Developed in C using **STM32CubeIDE** and configured via **STM32CubeMX** ensuring optimized peripheral driver allocation[cite: 12, 1067].
* [cite_start]**Hardware Design:** Schematics and multi-layer PCB design layouts structured via **Altium Designer** with dedicated noise and vibration dampening techniques for avionic reliability[cite: 12, 384].
