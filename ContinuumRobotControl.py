"""
Continuum Robot Control Script
By Eric Weissman
4/22/2025

Description:
This script implements a real-time control loop for a pneumatic continuum robot using:
- NatNet motion capture data for pose estimation (via the OptiTrack system)
- Serial communication with an Arduino to control pressure regulators
- A threaded controller class to manage state estimation and compute pressure setpoints

Key Components:
- ContinuumRobotController: A threaded class that buffers OptiTrack pose data, tracks a target pose,
  and updates pressure setpoints accordingly.
- SerialCommunicator: Custom serial interface for sending pressure setpoints to an Arduino and (optionally)
  receiving feedback.

Features:
- Thread-safe access to shared data using Python locks.
- Uses high-precision timers via `time.perf_counter()` for stable loop timing.
- Tracks and prints average loop frequency every 100 iterations for performance monitoring.
- Gracefully shuts down all background threads and serial communication on `KeyboardInterrupt`.

Usage:
1. Connect Arduino to the correct COM port with matching baudrate and protocol.
2. Ensure OptiTrack streaming is enabled and broadcasting the correct rigid body data.
3. Run this script. The control loop will:
    - Read pose data from OptiTrack
    - Compute pressure setpoints
    - updates set pressures at a specified frequency (50 Hz)
    - Send them to the Arduino via serial at 100Hz
4. Stop the script with Ctrl+C.

Note:
- Make sure `SerialCommunicator`, `NatNetClient`, `DataDescriptions`, and `MoCapData` are in the Python path.
- Adjust frequency (`freq`), COM port, and pressure logic as needed for your application.
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

###################################
# Controller Class
###################################

class ContinuumRobotController:
    def __init__(self, numRegulators=3, maxP = 35, RB_bufferSize=500):

        self.numRegulators= numRegulators
        self.PressureSetPoints = [0.0] * self.numRegulators
        self.maxP = maxP  # Max P for the muscles
        self.rigid_body_positions = deque(maxlen=RB_bufferSize) # buffer for rigid bodies in case time dependency matters
        self._running = False
        self._lock = threading.Lock()

        self.targetPose = [0.0, 0.0, 0.0]
        self.StartTime= 0

    def updateTarget(self, targetPose): # function to set the controller reference position
        with self._lock:
            self.targetPose= targetPose
    
    def ControllerThread(self): # Controller thread--> this is where you should put your MPC controller
        while self._running:
            with self._lock: # lock to avoid two threads trying to access the same data at the same time
                if self.rigid_body_positions:
                    latest_pose = self.rigid_body_positions[-1] # gets the lastest rigid body poses
            now = time.time_ns()/1e9- self.StartTime # gets the time since the controller thread was started

            ######################################################
            # your controller goes here
            ######################################################
            time.sleep(.02)#simulate computer thinking --> REMOVE ME

            with self._lock:
                self.PressureSetPoints = [self.constrainP(0),self.constrainP(30),self.constrainP(30)] # assign regulator pressures

            

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

    ###################################
    #Optitrack Defs
    ###################################
    def receive_new_frame(self, data_dict):
        pass  # not needed for visualization

    def receive_rigid_body_frame(self,new_id, position, rotation): # when optitrack finds a new frame it runs this def appending the new RB to the deque
        with self._lock:
            self.rigid_body_positions.append(position)

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
communicator = SerialCommunicator(port="COM12",
                                baudrate=115200,
                                n_floats_to_arduino=3,
                                n_floats_from_arduino=3,
                                verbose=False,
                                buffer_size=1000,
                                direction = 'OneWay2Arduino',
                                logFreq=True,
                                CSVPath = None,
                                DesiredFreq=100)
communicator.start()

#start Controller thread
Controller =ContinuumRobotController(numRegulators=3,maxP=35)
Controller.updateTarget([0.0, 0.0, 0.0]) #start target position
Controller.start()

freq = 50 # desired frequency of how often the regulator pressures are changed

lastTime = time.perf_counter()
dt_history = deque(maxlen=100)
loop_counter = 0

###########################################
# MAIN LOOP
###########################################
try:
    while True:
        
        now = time.perf_counter()
        dt = now - lastTime

        if dt >= 1/freq: # Make sure we are operating at freq

            lastTime=now
            loop_counter += 1
            dt_history.append(dt)

            Controller.updateTarget([0.0, 0.0, 0.0]) #update target position
            
            setpoint = Controller.RetrieveControlInput() 
            communicator.set_data_to_send(setpoint) # update regulator setpoints based on most up-to-date input from the controller 
        
            if loop_counter % 100 == 0: # every 100 loops print average freq
                avg_dt = sum(dt_history) / len(dt_history)
                avg_freq = 1 / avg_dt if avg_dt > 0 else 0
                print(f"[Loop Stats] Avg freq: {avg_freq:.2f} Hz")
            
        else:
            time.sleep(.0005)
            
except KeyboardInterrupt:
    print("\n[Shutdown] KeyboardInterrupt received. Stopping controller and communicator...")
    Controller.stop()
    communicator.stop()  # Only if your SerialCommunicator supports a stop() method
    print("[Shutdown] All threads successfully stopped.")