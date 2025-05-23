"""
===============================================================================
Continuum Robot Real-Time Control and Visualization System
===============================================================================

By Eric Weissman
Last Modified: May 13, 2025

Description:
------------
This script implements a complete real-time control loop for a pneumatically actuated 
continuum robot. It integrates the following subsystems:

1. **Serial Communication**: Interfaces with an Arduino (or similar microcontroller) 
   via a custom `SerialCommunicator` class to send computed pressure setpoints and 
   optionally log feedback values.

2. **Motion Capture Integration**: Uses OptiTrack's NatNet SDK (via `NatNetClient`) 
   to stream rigid body position data for the robot in real-time.

3. **PID Control**: Employs a 2D PID controller (`PID2DController`) to minimize the 
   pose error between the robot's current tip position and a target trajectory 
   defined in the X-Z plane.

4. **Pressure Optimization**: Computes optimal pressure values using least-squares 
   with linear inequality constraints to solve an underdetermined Jacobian mapping.

5. **Trajectory Generation**: Loads and samples a 2D path from an SVG file (e.g., 
   a logo or character) to produce a time-varying target trajectory.

6. **Visualization**: Displays robot pose and target in real-time using PyQtGraph 
   with a live-updating plot, as well as a separate UI panel for PID gain tuning 
   with PyQt5 widgets.

7. **Data Logging**: Records pose, target, error, and actuator pressure data to a 
   timestamped CSV file for later analysis.

Structure:
----------
- `PID2DController`: Encapsulates a basic PID controller in 2D space with anti-windup.
- `ContinuumRobotController`: Orchestrates pose tracking and pressure computation.
- `run_control_loop`: Main control loop thread that synchronizes sensor input, 
  trajectory tracking, control computation, and communication output.
- `update`: Callback for the visualization plot update.
- `update_pid_gains`: Reconfigures the PID controller on-the-fly using GUI sliders.
- `start_natnet_client`: Initializes the OptiTrack client and connects to the server.

Usage:
------
- Ensure OptiTrack Motive and the NatNet server are running.
- Connect Arduino via the specified COM port.
- Place an SVG file in the expected directory for trajectory generation.
- Run this script. The robot will attempt to track the trajectory in real-time, 
  visualize progress, and log data to a CSV file.

Dependencies:
-------------
- `SerialCommunicator.py`, `NatNetClient.py`, `MoCapData.py`, `DataDescriptions.py`
- Python packages: numpy, scipy, pyqtgraph, PyQt5, svgpathtools, matplotlib, csv

Note:
-----
All threading operations are protected by locks to ensure safe access to shared data.
To terminate the program cleanly, press `Ctrl+C`.

===============================================================================
"""

#imports
from SerialCommunicator import SerialCommunicator
from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData
import sys
import time
import threading
from collections import deque
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
from scipy.optimize import lsq_linear
from svgpathtools import svg2paths
import csv
from datetime import datetime
from scipy.optimize import brentq
from scipy.optimize import least_squares

###################################
# PID Class
###################################


class PID2DController:
    def __init__(self, Kp, Ki, Kd, integral_limit=0.05):
        self.Kp = np.array(Kp)
        self.Ki = np.array(Ki)
        self.Kd = np.array(Kd)
        self.integral_error = np.zeros(2)
        self.prev_error = np.zeros(2)
        self.initialized = False
        self.integral_limit = integral_limit  # maximum magnitude of integral term

    def reset(self):
        self.integral_error = np.zeros(2)
        self.prev_error = np.zeros(2)
        self.initialized = False

    def compute(self, X, Xr, dt):
        x, y = X[:2]
        xr, yr = Xr[:2]
        error = np.array([xr - x, yr - y])

        if not self.initialized:
            self.prev_error = error
            self.initialized = True

        # Compute derivative term
        derivative_error = (error - self.prev_error) / dt if dt > 0 else np.zeros(2)

        # Update integral with anti-windup clamp
        self.integral_error += error * dt
        self.integral_error = np.clip(self.integral_error, -self.integral_limit, self.integral_limit)

        # PID output
        output = (self.Kp * error +
                  self.Ki * self.integral_error +
                  self.Kd * derivative_error)

        self.prev_error = error

        return np.array([output[0], output[1]]), error

###################################
# Controller Class
###################################
class ContinuumRobotController:
    def __init__(self, numRegulators=3, maxP = 30, RB_bufferSize=500):

        self.numRegulators= numRegulators
        self.PressureSetPoints = [0.0] * self.numRegulators
        self.maxP = maxP  # Max P for the muscles
        self.rigid_body_positions = deque(maxlen=RB_bufferSize) # buffer for rigid bodies in case time dependency matters
        self._running = False
        self._lock = threading.Lock()

        self.targetPose = np.array([0.0, 0.0])
        self.StartTime= 0

        self.pid = PID2DController(Kp=[100.0, 100.0], Ki=[1, 1], Kd=[0.05, 0.05], integral_limit = 100)

    def updateTarget(self, targetPose): # function to set the controller reference position
        with self._lock:
            self.targetPose= targetPose

    def getMetrics(self):
        with self._lock:
            latest_pose = self.rigid_body_positions[-1] if self.rigid_body_positions else np.zeros(3)
            Target = self.targetPose
            Pvals = self.PressureSetPoints
        return Target, latest_pose, Pvals

    def constrainP(self,p): # helper function to ensure pressures are within safe limits
        return min(self.maxP, max(0,p))

    def start(self):
        self.StartTime= time.time_ns()/1e9
        self._running = True
        self.start_natnet_client()# Run NatNet client in a background thread
        time.sleep(1) #lets wait a sec to let the buffers start to fill
        threading.Thread(target=self.ControllerThread, daemon=True).start()

    def RetrieveControlInput(self): #function to access the calculated pressures
        with self._lock:
            return self.PressureSetPoints

    def stop(self):
        self.client.stop()
        self._running = False

    ######################################################################
    # Controller Defs
    ######################################################################
    def CartesianToCurvature(self,X,BackboneLength):
        # converts cartesian projection of the end effector into curvature and phi (angle off x axis of the end effector).
        # X is a 2x1 vector pointing to EE cartiesian projection

        phi = math.atan2(X[1],X[0])
        r = np.sqrt(X[0]**2 + X[1]**2)

        if r <1e-6:
            phi=0
            curvature=0
        else:
            def f(k):
                if k == 0:
                    return np.inf  # avoid division by zero
                return (1/k) * (1 - np.cos(BackboneLength * k)) - r

            k_min = 1e-6
            k_max = np.pi / BackboneLength - 1e-6

            try:
                curvature = brentq(f, k_min, k_max)
            except ValueError:
                k= np.pi/BackboneLength
                MaxR = (1/k) * (1 - np.cos(BackboneLength * k))
                print(f"Warning: Could not find solution for curvature. Note that max radius from origin is r={MaxR}, while current radius is {r}")
                curvature = None  # No root found in the interval

        return phi, curvature

    def muscle_lengths_root_solver(self, L, k, phi, d, s):
        #eqs 17, 18, 19 in "Design and Kinematic Modeling of Constant Curvature Continuum Robots: A Review""

        L1, L2, L3 = L
        num = 2 * np.sqrt(L1 ** 2 + L2 ** 2 + L3 ** 2 - L1 * L2 - L1 * L3 - L2 * L3)
        den = d * (L1 + L2 + L3)

        f1 = (L1 + L2 + L3) / 3 - s
        f2 = np.atan2((np.sqrt(3) * (L2 + L3 - 2 * L1)) , (3 * (L2 - L3))) - (phi)
        f3 = num / den - k

        return [f1, f2, f3]

    def MuscleLengths(self, curvature, phi, BackboneLength, VertebraDiam):
        # Finds the muscles lengths from the resting Length using root solving least squares.
        if curvature<1e-6:
            return (np.array([BackboneLength,BackboneLength,BackboneLength]) )
        else:
            x0 = BackboneLength + np.array([1e-3, -1e-3, 0])  # initial guess
            fun = lambda L: self.muscle_lengths_root_solver(L, curvature, phi, VertebraDiam, BackboneLength)

            # Use least_squares with positivity constraints to find final lengths
            result = least_squares(fun, x0, bounds=(1e-6, np.inf))

            if result.success:
                return (np.array(result.x) ) # Calculate the change in length.
            else:
                raise RuntimeError(f"Solver failed: {result.message}")

    def PressuresFromMoment(self, M, backboneLength, curvature, phi, VertebraDiam, MuscleStiffness, ForcePressureSensitivity, MuscleRestLength):
        # Given a desired net moment vector on the end effector, tries to find the requires muscle Pressures
        # Calcs the muscle lengths, the jacobbian, solves QP problem

        NumMuscles = 2 #num muscles per bundles

        MuscleLength = self.MuscleLengths(curvature, phi, backboneLength, VertebraDiam)
        deltaL= MuscleLength - MuscleRestLength

        # each column is the coordinates d = -[d_y, d_x] of the muscle's start relative to the origin
        J = np.array([
            [-1, -np.sin(-5*math.pi/6), -np.sin(-math.pi/6)],
            [0, np.cos(-5*math.pi/6), np.cos(-math.pi/6)]
            ])
        # print(deltaL)
        # since M = Cross(F,d), M = J*(k*L+a*P) where k is muscle stiffness, L is muscle displacement vector, and a is the pressure sensitiviy, use bounded QP problem
        P = lsq_linear(J, (np.transpose(M)-np.matmul(NumMuscles*J, VertebraDiam* MuscleStiffness*deltaL))/(ForcePressureSensitivity*NumMuscles*VertebraDiam), bounds=(0, self.maxP))
        print(np.transpose(M),-np.matmul(NumMuscles*J, VertebraDiam*MuscleStiffness*deltaL), np.transpose(M)-np.matmul(NumMuscles*J, MuscleStiffness*deltaL)/(VertebraDiam*ForcePressureSensitivity*NumMuscles))
        
        return P.x

    def momentFromCurvature(self,curvature, phi, YoungsModulus, CoreDiameter):
        # using eulerBeamApproximation, calcs the moment needed to bend the core

        I = np.pi*CoreDiameter**4 / 32
        M = [curvature*YoungsModulus*I*np.sin(phi), curvature*YoungsModulus*I*np.cos(phi)]
        return M

    def ControllerThread(self): # Controller thread--> this is where you should put your MPC controller

        YoungsModulus= 11.469 * 1e9 #Pa
        CoreDiameter = 1/1000 #m
        BackboneLength = 175/1000 #m
        MuscleStiffness = 54.5035 #N/m
        ForcePressureSensitivity =.015 #N/psi
        MuscleRestLength=155/1000 #m
        VertebraDiam=20/1000 #m

        pid = self.pid

        prevTime = time.time_ns() / 1e9 - self.StartTime

        while self._running:
            with self._lock: # lock to avoid two threads trying to access the same data at the same time
                if self.rigid_body_positions:
                    latest_pose = self.rigid_body_positions[-1] # gets the lastest rigid body poses

            # measure times
            now = time.time_ns() / 1e9 - self.StartTime  # gets the time since the controller thread was started
            dt = now - prevTime
            prevTime = now

            # project to X-Z plane. X and X_desired are in m
            X = np.array([latest_pose[0], latest_pose[2]])
            X_desired = self.targetPose
            

            #convert to curvatures and phis
            phi, curvature = self.CartesianToCurvature(X, BackboneLength)
            phi_desired, curvature_desired = self.CartesianToCurvature(X_desired, BackboneLength)


            # PID for feedback, closing the loop on phi and curvatures
            u, error = pid.compute([phi, curvature], [phi_desired, curvature_desired], dt)
            
            PIDGAIN= 0

            # Use constant curvature and muscle model to find P with FF controller
            M = self.momentFromCurvature(curvature_desired + PIDGAIN*u[1], phi_desired+ PIDGAIN*u[0], YoungsModulus, CoreDiameter)
            P = self.PressuresFromMoment(M, BackboneLength, curvature_desired, phi_desired,
                                         VertebraDiam, MuscleStiffness, ForcePressureSensitivity, MuscleRestLength)

            time.sleep(.001) #lets not murder the CPU

            with self._lock:
                self.PressureSetPoints = [self.constrainP(P[0]),self.constrainP(P[1]),self.constrainP(P[2])] # assign regulator pressures
               


    ###################################
    #Optitrack Defs
    ###################################
    def receive_new_frame(self, data_dict):
        pass  # not needed for visualization

    def receive_rigid_body_frame(self,new_id, position, rotation): # when optitrack finds a new frame it runs this def appending the new RB to the deque

     
        with self._lock:
            self.rigid_body_positions.append([-position[0], position[1], position[2]])#make x axis to the right to match webster paper convention

    def start_natnet_client(self):
        optionsDict = {
        "clientAddress": "127.0.0.1",
        "serverAddress": "127.0.0.1",
        "use_multicast": True,
        "stream_type": 'd'
        }

        self.client = NatNetClient()
        self.client.set_client_address(optionsDict["clientAddress"])
        self.client.set_server_address(optionsDict["serverAddress"])
        self.client.set_use_multicast(optionsDict["use_multicast"])

        self.client.set_print_level(print_level=0)

        self.client.new_frame_listener = self.receive_new_frame
        self.client.rigid_body_listener = self.receive_rigid_body_frame

        is_running = self.client.run(optionsDict["stream_type"])
        if not is_running:
            print("ERROR: Could not start streaming client.")
            sys.exit(1)

        time.sleep(1)
        if not self.client.connected():
            print("ERROR: Could not connect to server.")
            sys.exit(2)

        print("NatNet client connected successfully.")


############################################
# Start of control sequence
###########################################

#start Serial Coms
communicator = SerialCommunicator(port="COM13",
                                baudrate=115200,
                                n_floats_to_arduino=3,
                                n_floats_from_arduino=3,
                                verbose=False,
                                buffer_size=1000,
                                direction = 'OneWay2Arduino',
                                logFreq=True,
                                CSVPath = None,
                                DesiredFreq=200)
communicator.start()

#start Controller thread
Controller =ContinuumRobotController(numRegulators=3,maxP=65)
Controller.updateTarget([0.0, 0.0, 0.0]) #start target position
Controller.start()

freq = 100 # desired frequency of how often the regulator pressures are changed


#######################################
## Real time visualization
######################################

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="Continuum Robot Real-Time Visualization")
plot = win.addPlot(title="X-Z Plane: Target vs Robot Pose")
plot.setXRange(-.10, .10)
plot.setYRange(-.10, .10)
plot.setLabel('left', 'Z')
plot.setLabel('bottom', 'X')
plot.addLegend()

pose_dot = plot.plot([], [], pen=None, symbol='o', symbolBrush='b', name='Pose')
target_dot = plot.plot([], [], pen=None, symbol='o', symbolBrush='r', name='Target')
p_label = pg.TextItem(text='', color='w', anchor=(0, 1))
plot.addItem(p_label)

def update():
    try:
        target, pose, pvals = Controller.getMetrics()
        x_t, z_t = target[0], target[1]
        x_r, z_r = pose[0], pose[2]
        pose_dot.setData([x_r], [z_r])
        target_dot.setData([x_t], [z_t])

        p_label.setText(f"P1: {pvals[0]:.1f}, P2: {pvals[1]:.1f}, P3: {pvals[2]:.1f}")
        p_label.setPos(-.095, .095)  # place label at top left of the plot

    except Exception as e:
        print(f"Update error: {e}")

# Run timer at 20 Hz
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)


# #tracking ASU
DefaultGains= [2.5,.25, .0, 10000]

#tracking circle
# DefaultGains= [100.0, 3000.0, .1, .07]

gain_win = QtWidgets.QWidget()
gain_layout = QtWidgets.QFormLayout()

# Create sliders or spin boxes for each gain
kp_slider = QtWidgets.QDoubleSpinBox()
kp_slider.setRange(0, 10000)
kp_slider.setSingleStep(.5)
kp_slider.setValue(DefaultGains[0])

ki_slider = QtWidgets.QDoubleSpinBox()
ki_slider.setRange(0, 10000)
ki_slider.setSingleStep(0.01)
ki_slider.setValue(DefaultGains[1])

kd_slider = QtWidgets.QDoubleSpinBox()
kd_slider.setRange(0, 10000)
kd_slider.setSingleStep(0.001)
kd_slider.setValue(DefaultGains[2])

kiLimit_slider = QtWidgets.QDoubleSpinBox()
kiLimit_slider.setRange(0, 10000)
kiLimit_slider.setSingleStep(.001)
kiLimit_slider.setValue(DefaultGains[3])

# Add to layout
gain_layout.addRow("Kp", kp_slider)
gain_layout.addRow("Ki", ki_slider)
gain_layout.addRow("Kd", kd_slider)
gain_layout.addRow("Ki_Limit", kiLimit_slider)

gain_win.setLayout(gain_layout)
gain_win.setWindowTitle("PID Gain Tuning")
gain_win.show()

def update_pid_gains():
    new_kp = kp_slider.value()
    new_ki = ki_slider.value()
    new_kd = kd_slider.value()
    new_kiLimit= kiLimit_slider.value()

    Controller.pid.Kp[:] = [new_kp, new_kp]
    Controller.pid.Ki[:] = [new_ki, new_ki]
    Controller.pid.Kd[:] = [new_kd, new_kd]
    Controller.pid.integral_limit = new_kiLimit

update_pid_gains()

kp_slider.valueChanged.connect(update_pid_gains)
ki_slider.valueChanged.connect(update_pid_gains)
kd_slider.valueChanged.connect(update_pid_gains)
kiLimit_slider.valueChanged.connect(update_pid_gains)

#######################################
## saving data
########################################
log_filename = f"C:/Users/eweissm1/Visual Studio Projects/HARP Continuum Robot/RobotData/robot_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
log_file = open(log_filename, mode='w', newline='')
csv_writer = csv.writer(log_file)
csv_writer.writerow(['time', 'pose_x', 'pose_z', 'target_x', 'target_z', 'error_x', 'error_z', 'P1', 'P2', 'P3', 'freq'])

#############################################
# Control loop thread
#############################################
def run_control_loop():
    lastTime = time.perf_counter()
    dt_history = deque(maxlen=100)
    loop_counter = 0
    tik = time.time()
    #############################################
    #Generate Path to track
    ############################################
    paths, attributes = svg2paths('C:/Users/eweissm1/Visual Studio Projects/HARP Continuum Robot/RobotLineArt/ASUlogo.svg')

    # Discretize the first path (you can loop through multiple if needed)
    path = paths[0]
    Scale = .06/200.0
    # Sample N points along the path
    N = 500
    maxt = 200
    points = [path.point(t) for t in np.linspace(0, 1, N)]
    x = [Scale*p.real-.03 for p in points]
    y = [Scale*p.imag-.03 for p in points]
    y = [-yi for yi in y]
    t = np.linspace(2, maxt, N) 
    

    
    frequencies = np.arange(0.1, 10.1, 0.1)  # Frequencies from 0.1 to 10.0 Hz

    try:
        for TrajectoryFrequency in frequencies:
            print(f"\n[Trajectory] Running at {TrajectoryFrequency:.1f} Hz")
            duration = 3.0 / TrajectoryFrequency  # Run for 3 full cycles
            start_time = time.time()

            # Reset timing variables
            lastTime = time.perf_counter()
            tik = time.time()
            loop_counter = 0
            dt_history = []

            while time.time() - start_time < duration:
                now = time.perf_counter()
                dt = now - lastTime

                if dt >= 1 / freq:
                    lastTime = now
                    loop_counter += 1
                    dt_history.append(dt)
            
                    idx = np.searchsorted(t, time.time() - tik, side="left")

                    Controller.updateTarget(np.array([
                        30.0 / 1000.0 * math.cos((time.time() - start_time) * 2 * math.pi * TrajectoryFrequency),
                        30.0 / 1000.0 * math.sin((time.time() - start_time) * 2 * math.pi * TrajectoryFrequency)
                    ]))

                    setpoint = Controller.RetrieveControlInput()
                    communicator.set_data_to_send(setpoint)
                    
                    target, pose, pvals = Controller.getMetrics()
                    pose_x, pose_z = pose[0], pose[2]
                    target_x, target_z = target[0], target[1]
                    error_x = target_x - pose_x
                    error_z = target_z - pose_z
                    timestamp = time.time() - tik

                    # Write to CSV
                    csv_writer.writerow([timestamp, pose_x, pose_z, target_x, target_z, error_x, error_z, *pvals, TrajectoryFrequency])
                    log_file.flush()  # Ensure data is written immediately

                    if loop_counter % 100 == 0:
                        avg_dt = sum(dt_history) / len(dt_history)
                        avg_freq = 1 / avg_dt if avg_dt > 0 else 0
                        print(f"[Loop Stats] Avg freq: {avg_freq:.2f} Hz")

                else:
                    time.sleep(0.0005)

    except KeyboardInterrupt:
        print("\n[Shutdown] KeyboardInterrupt received. Stopping controller and communicator...")
        Controller.stop()
        communicator.stop()
        log_file.close()
        print("[Shutdown] All threads successfully stopped.")


threading.Thread(target=run_control_loop, daemon=True).start()

win.show()
QtWidgets.QApplication.instance().exec_()



