# Vented Balloon Controller Testing Guide

This guide outlines the process for testing a weather balloon's ascent control system using an Arduino program and a Python-based altitude simulator. It demonstrates how the controller manages vent operations and flight states in a simulated environment.

## Prerequisites

1.  Arduino IDE
2.  SimulIDE
3.  Python
4.  com0com (virtual serial port emulator)
5.  Git **If you don't have Git:** You can download a zip file containing the files

## Adjustable Parameters

Many parameters in the Python and Arduino codes can be tuned to adjust the behavior of the simulation. These can be adjusted by finding the file, and then changing the variables. The important parameters are described below:

**Python Simulation Parameters (`vent_test.py`):**

- `arduino_port`: COM port for communication with the Arduino. (e.g., `'COM9'`) (Note: It must one of the ports linked virtually by com0com)
- `initial_ascent_rate`: Initial ascent rate of the balloon in meters per second (m/s). (e.g., `6`)
- `deceleration_rate`: The rate at which the balloon decelerates when venting, in meters per second squared (m/s^2). (e.g., `0.0167`)

**Arduino Control Parameters (`vented_balloon_controller.ino`):**

- `TARGET_ALTITUDE`: The target altitude in meters at which venting should begin. (e.g., `100`)
- `FLOAT_DURATION`: The duration (in milliseconds) the balloon attempts to maintain a constant altitude before initiating full descent. (e.g., `30000`)
- `EMA_ALPHA`: The Exponential Moving Average (EMA) smoothing factor, which affects how much weight is given to recent ascent rate measurements. Lower values provide more smoothing. (e.g., `0.2`)
- `REQUIRED_AGREEMENTS`: The number of consecutive state proposals that must agree before a new flight state is adopted. Higher values make the system more resistant to noise. (e.g., `10`)
- `VENT_OPEN_POSITION`: Value of the servo to open the vent fully
- `VENT_CLOSE_POSITION`: Value of the servo to keep the vent closed fully

## Setup

1. (SKIP if you don't have Git) Clone the repository:

   ```
   git clone https://github.com/rkamindo/emu-apex/vented-balloon-controller.git
   cd vented-balloon-controller
   ```

2. (SKIP if you hvae Git) Download the zip file of the repository by navigating to the green button labeled Code in the Github repo and then clicking Download Zipfile

- Extract the contents
- Open the file in your code editor

3.  **(Optional) Set up a Python virtual environment:**

    ```
    python -m venv venv #This is optional but recommended
    source venv/bin/activate  # On Windows, use `venv\Scripts\activate`
    ```

4.  Install the required Python package:

    ```
    pip install pyserial
    ```

5.  Install com0com and create a pair of virtual COM ports (e.g., COM9 and COM10).

    <img width="336" alt="com0com" src="https://github.com/user-attachments/assets/7dd78769-2f04-4944-b7ca-3084dd03b462" />


7.  Open the Arduino sketch (`vented_balloon_controller.ino`) in the Arduino IDE
    - Ensure `TEST_MODE` is set to `true` and save the file.
    - Extract and compile (Go to Sketch > Extract Compiled Binary). The compiled binary will be in a newly created build folder in the project.

## Running the Test

1.  **Open SimulIDE and Load the Circuit:**

    - Open SimulIDE.
    - Load your `BalloonController.sim1` circuit (This is in the repository)
    - Right-click the Nano microcontroller
    - Click the Atmega328 processor
    - Select load firmware to load firmware
    - Navigate to vented_balloon_controller project, and to the build folder to find the files ending in .hex
    - Select the .hex file

2.  Ensure the serial port in the SimulIDE project is set to the COM port that was virtually linked to the Python test script by com0com (e.g., COM10). Verify that the TX and RX pins on the serial port are connected correctly and the button indicates that the serial monitor port is currently set to "Close".

    - After all the configurations, click the "Open Serial Monitor" button in SimulIDE

3.  Start the Python simulation script:

    ```
    python vent_test.py
    ```

    Ensure it connects successfully to the COM port (e.g., COM9).

4.  Start the simulation in SimulIDE.

5.  **Observe the Interaction:**

    - Watch the interaction between the Python script and the Arduino program in their respective console outputs. Note the following:
    - **Arduino Request for Altitude:** The Arduino program, running in `TEST_MODE`, simulates reading a sensor by sending a `REQUEST_ALTITUDE` message over the serial port. You'll see this in the SimulIDE serial monitor.
    - **Python Calculation and Response:** The Python program receives the `REQUEST_ALTITUDE` command, calculates the altitude based on the simulated ascent rate and venting, and sends the calculated altitude back to the Arduino over the serial port.
    - **Arduino Vent Control:** Based on the received altitude and its control logic, the Arduino program will send `VENT_OPEN` and `VENT_CLOSE` commands over the serial port. You'll see these in the SimulIDE serial monitor.

    * **Flight State and State Proposal Agreements:** Every two seconds the Arduino program will propose a flight state based on the Exponential Moving Average (EMA) ascent rate. The `updateFlightState()` function requires `REQUIRED_AGREEMENTS` (currently set to 10) consecutive agreements before transitioning to a new state. You'll see the "Current flight state:" printed, with a number corresponding to the state: 0 = FAST_ASCENT, 1 = SLOW_ASCENT, 2 = FLOAT, 3 = SLOW_DESCENT, 4 = FAST_DESCENT. The Arduino sketch includes a print statement that is triggered after 10 agreements that states "10 consecutive state proposals reached, new state".

## Troubleshooting

- If you encounter connection issues, double-check that the COM ports match in both the Python script and SimulIDE.
- Ensure that `TEST_MODE` is set to `true` in the Arduino sketch.
- If the Python script fails to start, verify that you've installed `pyserial`.

## Notes

- The Python script simulates the balloon's behavior, including ascent rate, altitude changes, venting
- The Arduino program controls the vent based on the simulated data it receives.
- Pay attention to the state transitions and vent control commands in the Arduino serial monitor output.

## Troubleshooting

- If you encounter connection issues, double-check that the COM ports match in both the Python script and SimulIDE.
- Ensure that `TEST_MODE` is set to `true` in the Arduino sketch.
- If the Python script fails to start, verify that you've installed `pyserial`.

## Notes

- The Python script simulates the balloon's behavior, including ascent rate and altitude changes.
- The Arduino program controls the vent based on the simulated data it receives.
- Pay attention to the state transitions and vent control commands in the output.
