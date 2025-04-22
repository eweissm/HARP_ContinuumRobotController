"""
Serial Communication Interface
By Eric Weissman
4/18/2025

Description:

SerialCommunicator: Bidirectional Serial Communication Interface for Arduino

This module defines the SerialCommunicator class, which facilitates real-time,
synchronized communication between a host computer (Python) and an Arduino via
a serial port. It supports three communication modes:
    1. TwoWay - Simultaneous sending and receiving of float arrays.
    2. oneWayFromArduino - Only receive data from Arduino.
    3. OneWay2Arduino - Only send data to Arduino.

Key Features:
- Sync byte protocol to ensure data alignment.
- Transmission of float arrays using `struct` for byte-level control.
- Threaded architecture for non-blocking send/receive operations.
- Optional logging of communication frequency.
- Data queue for easy access to received data.
- Configurable number of floats for transmission and reception.

Usage:
- Instantiate the class with serial port settings and desired mode.
- Use `start()` to begin communication.
- Use `set_data_to_send()` to update outgoing float data.
- Use `stop()` to end communication cleanly.

Example:
    communicator = SerialCommunicator(port="COM3", baudrate=115200, direction='TwoWay')
    communicator.start()
    ...
    communicator.set_data_to_send([0.1, 0.2, ..., 0.9]) # dont call this too often or you will make the CPU sad
    latest_buffer = communicator.get_buffer()
    ...
    communicator.stop()

Note:
Ensure the baudrate and COM port match the configuration on the Arduino.
"""

import serial
import struct
import threading
import time
from collections import deque
import csv
from queue import Queue


class SerialCommunicator:
    def __init__(self,
                 port,
                 baudrate,
                 n_floats_to_arduino=9,
                 n_floats_from_arduino=9,
                 direction = 'TwoWay',
                 verbose=True,
                 logFreq=False,
                 buffer_size=100,
                 CSVPath = None,
                 DesiredFreq = 10000):

        directionOption = ['TwoWay', 'oneWayFromArduino' , 'OneWay2Arduino'] # check to make sure valid direction has been selected
        if direction not in directionOption:
            raise Exception(f"Enter a valid direction from: {directionOption}")

        self.direction = direction #selected direction/ protocal for serial comms
        self.port = port  # COM port for the Arduino
        self.baudrate = baudrate # selected baud rate for serial comms. Must match arduino's
        self.n_floats_to_arduino = n_floats_to_arduino # num floats to send to arduino
        self.n_floats_from_arduino = n_floats_from_arduino # num floats to received from the arduino
        self.sync_byte = 0xAA # byte used to sync up binary messages being sent
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.buffer_size = buffer_size
        self.running = False # state of communication threads
        self.data_to_send = [0.0] * self.n_floats_to_arduino # message to send to the arduino
        self.verbose = verbose # option selecting to print sent and received data
        self.expected_bytes = self.n_floats_from_arduino * 4 # length of message from the arduino
        self.buffer = deque(maxlen=buffer_size)  # Constantly updating buffer which will store the received data from the arduino
        self.logFreq = logFreq # option to measure communication freq
        self.data_queue = Queue()
        self.CSVPath = CSVPath
        self.DesiredFreq = DesiredFreq

        # Deques to store recent timestamps for frequency calculation
        self.send_timestamps = deque(maxlen=100)
        self.recv_timestamps = deque(maxlen=100)
        self.send_counter = 0
        self.recv_counter = 0

        self._lock = threading.Lock()

    def start(self): #starts communication threads according to selected direction/ protocal

        time.sleep(1) # sleep for stability
        self.running = True

        match self.direction:
            case 'TwoWay':
                threading.Thread(target=self.listen_thread_twoWay, daemon=True).start()
                threading.Thread(target=self.write_thread_twoWay, daemon=True).start()
            case 'oneWayFromArduino':
                threading.Thread(target=self.listen_thread_oneWayFromArduino, daemon=True).start()
            case 'OneWay2Arduino':
                threading.Thread(target=self.write_thread_OneWay2Arduino, daemon=True).start()

        if self.CSVPath is not None:
            threading.Thread(target=self.data_writer, daemon=True).start()

    def get_buffer(self):  # Buffer accessor method
        return list(self.buffer)

    def data_writer(self):
        buffer = []

        with open(self.CSVPath, 'w', newline='') as f:
            writer = csv.writer(f)

            lastTime = time.perf_counter()

            while self.running:

                now = time.perf_counter()
                dt = now - lastTime

                if dt >= 1/self.DesiredFreq:
                    lastTime=now
                    try:
                        data = self.data_queue.get(timeout=1)
                        buffer.append(data)

                        if len(buffer) >= self.buffer_size:
                            writer.writerows(buffer)
                            buffer.clear()
                    except:
                        continue
                else:
                    time.sleep((1/self.DesiredFreq)/10)

    def log_frequency(self, timestamps, label, counter_name):
        # helper func to record and display the communication frequency

        if self.logFreq:
            now = time.time()
            timestamps.append(now)
            counter = getattr(self, counter_name)
            counter += 1
            if len(timestamps) == timestamps.maxlen and counter >= timestamps.maxlen:
                duration = timestamps[-1] - timestamps[0]
                freq = len(timestamps) / duration if duration > 0 else 0
                print(f"[{label}] Avg Frequency (last {len(timestamps)}): {freq:.2f} messages/sec")
                counter = 0
            setattr(self, counter_name, counter)

    def listen_thread_twoWay(self):
        # thread to control listening in the two way communication context.

        while self.running:
            if self.ser.in_waiting >= 1:  # if we have something to read

                sync = self.ser.read(1)  # read next byte
                if sync[0] == self.sync_byte:  # if the next byte is the sync bit
                    data_bytes = self.ser.read(self.expected_bytes)  # read the expected bytes

                    # process the data into floats
                    floats = [struct.unpack('<f', data_bytes[i * 4:i * 4 + 4])[0]
                              for i in range(self.n_floats_from_arduino)]

                    self.buffer.append(floats) #easy to access data queue

                    if self.CSVPath is not None:
                        self.data_queue.put(floats)

                    self.log_frequency(self.recv_timestamps, "Listener", "recv_counter")  # measure freq

                    if self.verbose:
                        print(f"[Listener] Received: {floats}")
            # self.pythonReady = True

    def write_thread_twoWay(self):
        # thread to control writing in the two way communication context.

        while self.running:
            with self._lock:
                # lets buils our message as b'sync byte floats'
                payload = bytearray()
                payload.append(self.sync_byte)
                for value in self.data_to_send:
                    payload.extend(struct.pack('<f', value))

                self.ser.write(payload)  # send message to the arduino

                self.log_frequency(self.send_timestamps, "Writer", "send_counter")  # measure freq

                if self.verbose:
                    print(f"[Writer] Sent: {self.data_to_send}")

    def listen_thread_oneWayFromArduino(self):
        while self.running:

            sync = self.ser.read(1)

            if sync and sync[0] == self.sync_byte:
                data_bytes = self.ser.read(self.expected_bytes)
                if len(data_bytes) == self.expected_bytes:
                    try:
                        floats = [struct.unpack('<f', data_bytes[i * 4:i * 4 + 4])[0]
                                  for i in range(self.n_floats_from_arduino)]

                        self.buffer.append(floats) #easy to access data queue

                        if self.CSVPath is not None:
                            self.data_queue.put(floats)

                        self.log_frequency(self.recv_timestamps, "Listener", "recv_counter")
                        if self.verbose:
                            print(f"[Listener] Received: {floats}")
                    except struct.error:
                        continue

    def write_thread_OneWay2Arduino(self):

        lastTime = time.perf_counter()

        while self.running:

            now = time.perf_counter()
            dt = now - lastTime

            if dt >= 1/self.DesiredFreq:

                lastTime=now

                with self._lock:
                    # lets builds our message as b'sync byte floats'
                    payload = bytearray()
                    payload.append(self.sync_byte)
                    for value in self.data_to_send:
                        payload.extend(struct.pack('<f', value))

                    self.ser.write(payload)  # send message to the arduino

                    self.log_frequency(self.send_timestamps, "Writer", "send_counter")  # measure freq

                    if self.verbose:
                        print(f"[Writer] Sent: {self.data_to_send}")

            else:
                    time.sleep((1/self.DesiredFreq)/10)

    def stop(self):
        self.running = False
        self.ser.close()

    def set_data_to_send(self, new_data):
        with self._lock:
            if len(new_data) == self.n_floats_to_arduino:
                self.data_to_send = new_data
            else:
                raise ValueError(f"Expected {self.n_floats_to_arduino} floats")


if __name__ == "__main__":
    communicator = SerialCommunicator(port="COM3", baudrate=115200, n_floats_to_arduino=9, n_floats_from_arduino=9, verbose=False, direction = 'TwoWay', logFreq=True)
    communicator.start()

    try:
        while True:
            time.sleep(1)
            # Dynamically change data if needed
            communicator.set_data_to_send([1.1, 2.2,3.3,4.4,5.5,6.6,7.7,8.8,9.9])

    except KeyboardInterrupt:
        communicator.stop()
        print("Communication stopped.")