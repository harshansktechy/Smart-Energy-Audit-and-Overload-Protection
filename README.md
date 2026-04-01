OVERVIEW

Built a real time energy monitoring and motor guard system utilizing a PIC16F877A microcontroller and an ACS712 current sensor. Programmed automated safety protocols to instantly detect overcurrent events and safely disconnect loads, effectively preventing catastrophic hardware damage.

HARDWARE AND TECH STACK

Microcontroller: PIC16F877A
Sensors: ACS712 Current Sensor
Actuators and Components: Relay Module for load disconnection, 16x2 LCD Display for real time monitoring, AC Motor and Load
Language: Embedded C
IDE and Tools: MPLAB X IDE, XC8 Compiler

SYSTEM ARCHITECTURE AND WORKFLOW

The system operates on a continuous monitoring loop to ensure absolute hardware safety:

Step 1. Current Sensing: The ACS712 sensor continuously reads the analog current drawn by the load.
Step 2. Data Processing: The PIC16F877A internal ADC converts these analog readings into digital values to calculate real time power consumption.
Step 3. Safety Protocol: The microcontroller compares the live current draw against pre programmed safe thresholds.
Step 4. Trigger Mechanism: If an overcurrent or abnormal spike is detected, the microcontroller instantly signals the Relay Module to trip, completely isolating the load from the power supply.

CIRCUIT SCHEMATIC AND SIMULATION
https://github.com/user-attachments/assets/beafa1bf-69f4-4fd1-b911-bb60638a643f

