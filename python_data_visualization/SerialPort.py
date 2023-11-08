import matplotlib.pyplot as plt
import serial
import time
import seaborn as sns
import numpy as np

class SerialPort:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud = baud_rate
        self.data = None

    def connect(self):
        self.data = serial.Serial(port=self.port, baudrate=self.baud)
        time.sleep(2)
        return self.data

    def read_data(self):
        return self.data.readline().decode('utf-8').strip()

class DataCollector:
    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.angles = []
        self.distances = []
        self.set_points = []
        self.pid_outputs = []
        self.errors = []
        self.cumulative_errors = []
        self.derivatives = []

    def collect(self):
        if self.serial_port.data.inWaiting() > 0:
            data_str = self.serial_port.read_data()
            if data_str.startswith("PLOT") and data_str.endswith("END"):
                _, angle,dist, sp, pid_output, error, cum_error, derivative, _ = data_str.split(',')
                self.angles.append(float(angle))
                self.distances.append(float(dist))
                self.set_points.append(float(sp))
                self.pid_outputs.append(float(pid_output))
                self.errors.append(float(error))
                self.cumulative_errors.append(float(cum_error))
                self.derivatives.append(float(derivative))
        return self.angles, self.distances, self.set_points, self.pid_outputs, self.errors, self.cumulative_errors, self.derivatives



