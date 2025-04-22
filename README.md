# HARP_ContinuumRobotController

# Instructions

- Upload ArduinoCodes/ContinuumRobotArduino.ino to the arduino
- Start Optitrack's Motive
- Label each vertebra as a rigid body in Motive
- Start Streaming
- On open the project in VS Code
  - Open ContinuumRobotControl.py. This is where the main controller is housed for the robot
  - Make sure you have the dependency pyserial
  - Update com port in the constructor for the communicator object
  - Add your MPC to the ControllerThread() def.
  - Change freq (frequency of the regulator pressure update as needed. Currently it is at 50 Hz. If you increase freq also change the serial communicator's frequency.
