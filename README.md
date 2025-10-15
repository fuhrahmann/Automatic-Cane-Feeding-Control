# Automatic Cane Feeding Control

<p align="center"> <img src="assets/CaneCarrier.png" alt="Cane Carrier" width="500"> </p>

## 📄 What is this?
This is my final project for achieving Bachelor of Applied Engineering for Automation Engineering Techology at Insitut Teknologi Sepuluh Nopember. This project focuses on implementing a Proportional-Integral-Derivative (PID) control system to regulate the speed of a cane carrier motor in a sugar factory based on the sugarcane bagasse level in the chute using cane carrier prototype. The goal is to minimize frequent sugarcane pile-ups caused by a mismatch between motor speed and material flow.
The system was developed using:

- Microcontroller: Arduino Uno R3
- Actuator: 3-Phase Induction Motor
- Sensor: 5 Infrared Proximity Sensor (for level detection)
- PID Tuning Method: Ziegler-Nichols

Test results show the system successfully minimized pile-ups with a steady-state error of 0.89 and 25% overshoot. The system only experienced a maximum level pile-up for 13 seconds out of a total 219 seconds of operation.

## 🛠️ System Architecture
The following diagram illustrates the overall system architecture:

<p align="center"> <img src="assets/SystemArchitecture.png" alt="System Architecture" width="600"> </p>

Component Description:

- Proximity Sensor: Monitors the bagasse level in the chute.
- Arduino Uno: Processes sensor data and runs the PID algorithm.
- VFD ATV12HU15M2: Regulates motor speed based on the PWM signal from the Arduino.
- 3-Phase Induction Motor: Drives the cane carrier.
- Power Supply & Buck Converter: Provides power for the entire system.

## 🔄 System Flowchart
The system workflow is defined by the following flowchart:

<p align="center"> <img src="assets/BlockDiagramControl.png" alt="Block Diagram Control" width="600"> <img src="assets/SystemFlowchart.png" alt="System Flowchart" width="500"> </p>

Flowchart Explanation:

- The system starts with component initialization.
- The proximity sensor reads the material level in the chute.
- Arduino compares the actual level with the setpoint.
- Based on the error, the PID algorithm calculates the PWM value.
- The PWM signal is sent to the VFD to adjust the motor speed.
- The process repeats in real-time until the push button for LI1 is set to be not connected.

## ⚡ Wiring Diagram
The electrical wiring for the cane carrier control system is shown below:

<p align="center"> <img src="assets/WiringDiagram.png" alt="Wiring Diagram" width="700"> </p>
Wiring Description:

- Arduino Pins 2-8: Connected to infrared proximity sensors.
- Arduino Pin 9 (PWM): Output to VFD (Analog Input AII).
- VFD Output (U, V, W): Drives the 3-phase induction motor.
- 12V Power Supply: Powers the Arduino via a buck converter (12V → 9V).
- Push Button: Connected to the VFD (LI1) for manual start/stop.