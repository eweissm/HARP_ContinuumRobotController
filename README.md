# HARP Continuum Robot Control Suite

**Author:** Eric Weissman  
**Last Updated:** May 2025

This repository contains the full control stack for a soft continuum robot system (HARP) actuated using pneumatic artificial muscles. It includes Arduino firmware and multiple Python control scripts for modelless and model-based control approaches, using OptiTrack motion capture for feedback.

---

## System Overview

- **Robot:** Soft continuum robot (HARP)
- **Actuation:** Pneumatic muscle bundles
- **Sensors:** Motion capture via OptiTrack (NatNet SDK)
- **Control Loop Rate:** 100–200 Hz
- **Pressure Regulation:** Arduino + PWM control
- **Control Methods:**
  - **Modelless PID:** Jacobian-based Cartesian control
  - **Constant Curvature:** Physics-based curvature estimation
- **Interface:** Real-time visualization via PyQtGraph
- **Data Logging:** CSV export of pose, target, error, and pressures

---

## Main Components

### `ContinuumRobotArduino.ino`

**Platform:** Arduino  
**Role:** Low-level actuator interface

- Reads 3–9 pressure setpoints from serial (float values with sync byte `0xAA`)
- Converts pressure values to PWM voltages to drive analog regulators
- Reads actual pressures via analog sensors and sends back float values
- Max pressure constrained to 35 psi

---

### `ContinuumRobotControl_Modelless.py`

**Approach:** Modelless Cartesian PID control

- Tracks robot tip pose in X-Z plane using PID
- Uses Jacobian-based pressure mapping via `scipy.optimize.lsq_linear`
- Real-time visualization with PyQtGraph
- GUI for live PID gain tuning
- Tracks arbitrary SVG-defined paths or sine wave trajectories
- Logs control and performance data to CSV

---

### `ContinuumRobotControl_Modelless_BODE.py`

**Approach:** Same as above + frequency analysis

- Adds BODE plot generation and signal injection tools
- Useful for identifying system response characteristics

---

### `ContinuumRobotControl_ConstantCurvature.py`

**Approach:** Physics-based constant curvature control

- Computes curvature (κ) and angle (ϕ) from Cartesian pose
- Uses constant curvature kinematics and inverse muscle model
- Computes bending moments using Euler-Bernoulli beam theory
- Solves for muscle pressures using `lsq_linear`
- PID feedback applied to curvature and angle errors
- GUI for PID tuning + trajectory tracking

---

### `ContinuumRobotControl_ConstantCurvature_BODE.py`

**Approach:** Constant curvature + system ID

- Adds experimental tools for step response and BODE characterization
- Useful for validating the beam bending model and controller tuning

---

## Dependencies

**Python Packages:**
- `numpy`
- `scipy`
- `pyqtgraph`
- `PyQt5`
- `matplotlib`
- `svgpathtools`

**Custom Modules (must be in Python path):**
- `SerialCommunicator.py`
- `NatNetClient.py`
- `MoCapData.py`
- `DataDescriptions.py`

---

##  Usage

1. Upload `ContinuumRobotArduino.ino` to your Arduino board.
2. Connect the Arduino to your computer (`port="COM13"` by default).
3. Ensure OptiTrack Motive is running and streaming rigid body data.
4. Choose and run a Python control script:
    - `Modelless.py` for Cartesian PID
    - `ConstantCurvature.py` for physics-based control
5. Adjust PID gains via the GUI if needed.
6. Observe real-time pose tracking and pressure display.
7. Use `Ctrl+C` to safely stop the program and flush logs.

---

## Folder Structure

```plaintext
HARP_Robot_Controller/
├── ContinuumRobotArduino.ino
├── ContinuumRobotControl_Modelless.py
├── ContinuumRobotControl_Modelless_BODE.py
├── ContinuumRobotControl_ConstantCurvature.py
├── ContinuumRobotControl_ConstantCurvature_BODE.py
├── SerialCommunicator.py
├── NatNetClient.py
├── DataDescriptions.py
├── MoCapData.py
├── RobotLineArt/
│   └── ASUlogo.svg
├── RobotData/
│   └── robot_log_*.csv


## Notes
* Live trajectory tracking uses SVG paths; modify or replace ASUlogo.svg as needed.

* Model-based control assumes consistent backbone stiffness and muscle placement.

* For experimental dynamics, use BODE variants to assess frequency response.


Feel free to submit issues or feature requests!
