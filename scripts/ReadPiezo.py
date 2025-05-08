#!/usr/bin/env python3

import serial
import struct
import numpy as np
import time
import rospy
from sensor_controller.msg import Piezosensor, Thresholds
from sensor_controller.srv import Tare, TareResponse

class SensorController:
    def __init__(self):
        rospy.init_node('sensor_controller', anonymous=False)

        self.Pub = rospy.Publisher('/piezosensor', Piezosensor, queue_size=10)
        self.ThresholdPub = rospy.Publisher('/thresholds', Thresholds, queue_size=2)
        self.TareService = rospy.Service('/tare', Tare, self.tare)

        rate = rospy.get_param("~rate", 3000)
        self.n_sensors = rospy.get_param("~n_sensors", 8)
        self.start_tare = rospy.get_param("~start_tare", False)
        self.tare_window = rospy.get_param("~tare_window", 1000)
        rospy.loginfo(f"Number of sensors: {self.n_sensors}")

        self.tare_values = np.array([0 for i in range(self.n_sensors)])
        self.tare_base = np.array([0 for i in range(self.n_sensors)])
        self.tare_std = np.array([0 for i in range(self.n_sensors)])
        self.th_up = np.array([0 for i in range(self.n_sensors)])
        self.th_down = np.array([2**16 for i in range(self.n_sensors)])
        
        self.tare_counter = 0

        self.sensor = serial.Serial(stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        self.sensor.port = self.configure_port()
        
        self.sensor.baudrate = 1e6

        self.header = "3c3e00"
        self.n_bytes = self.n_sensors*2 + len(self.header)//2

        if self.n_sensors >= 4:
            str1 = rospy.get_param("~str1", "num1,17,18,19,20\r\n")
            self.start_bytes1 = bytes.fromhex(str1.encode('ascii').hex())

        if self.n_sensors >= 8:
            str2 = rospy.get_param("~str2", "num2,21,22,23,24\r\n")
            self.start_bytes2 = bytes.fromhex(str2.encode('ascii').hex())

        if self.n_sensors >= 12:
            str3 = rospy.get_param("~str3", "num3,25,26,27,28\r\n")
            self.start_bytes3 = bytes.fromhex(str3.encode('ascii').hex())
        
        if self.n_sensors >= 16:
            str4 = rospy.get_param("~str4", "num4,29,30,31,32\r\n")
            self.start_bytes4 = bytes.fromhex(str4.encode('ascii').hex())

        self.data = [0 for i in range(self.n_sensors)]
        self.rate = rospy.Rate(rate)


    def configure_port(self):
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        possible_ports = [p for p in ports if 'USB' in p.description]
        if len(possible_ports) >= 1:
            return possible_ports[0].device
        else:
            rospy.loginfo("No USB ports detected.")
            return None


    def start(self):
        if self.sensor.port:
            try:
                self.sensor.open()
                rospy.loginfo(f"Piezo Sensor connected to {self.sensor.port}")
            except serial.SerialException as e:
                rospy.loginfo(f"Failed to open serial port {self.sensor.port}: {e}")
        else:
            rospy.loginfo("No valid sensor port found.")

        if self.n_sensors >= 4:
            self.sensor.write(self.start_bytes1)
            self.find_header()
            time.sleep(0.2)  
        if self.n_sensors >= 8:
            self.sensor.write(self.start_bytes2)
            self.find_header()
            time.sleep(0.2)
        if self.n_sensors >= 12:
            self.sensor.write(self.start_bytes3)
            self.find_header()
            time.sleep(0.2)
        if self.n_sensors >= 16:
            self.sensor.write(self.start_bytes4)
            self.find_header()
            time.sleep(0.2)
        rospy.loginfo("Sensor controller started")

    
    def read(self):
        self.find_header()
        data = self.sensor.read(self.n_bytes-len(self.header)//2)
        data = data.hex()
        return data
    

    def find_header(self):
        header = self.sensor.read(len(self.header)//2)
        header = header.hex()
        while header != self.header:
            addheader = self.sensor.read(1)
            addheader = addheader.hex()
            header = header[2:] + addheader


    def extract_bytes(self, data, n_bytes = 2):
        n_values = 2*n_bytes
        #data = data[len(self.header):]
        data = [data[i:i+n_values] for i in range(0, len(data), n_values)]
        data = [int(value, 16) for value in data]
        return data

            
    def publish_data(self):
        msg = Piezosensor()
        msg.data = self.data
        msg.timestamp = rospy.Time.now()
        self.Pub.publish(msg)
        self.rate.sleep()


    def publish_thresholds(self):
        msg = Thresholds()
        msg.th_up = list(self.th_up)
        msg.th_down = list(self.th_down)
        self.ThresholdPub.publish(msg)
        self.rate.sleep()


    def tare(self, req):
        self.start_tare = True
        return TareResponse(True)
    
    
    def check_tare(self, data):
        if self.start_tare and self.tare_counter < self.tare_window:
            self.tare_base += np.array(data)
            self.tare_std += np.array(data)**2
            self.tare_counter += 1
        elif self.start_tare and self.tare_counter == self.tare_window:
            self.tare_values = self.tare_base//self.tare_window
            self.tare_std = np.sqrt(self.tare_std//self.tare_window - self.tare_values**2)
            self.th_up = self.tare_values + 2*(self.tare_std)
            self.th_down = self.tare_values - 2*(self.tare_std)
            self.start_tare = False
            self.tare_counter = 0
            self.tare_base = np.array([0 for i in range(self.n_sensors)])
            self.publish_thresholds()
            rospy.loginfo("Tare completed")
            # rospy.loginfo(f"Thresholds: {self.th_up} - {self.th_down}")
            # rospy.loginfo(f"Tare values: {self.tare_values}")


    def run(self):
        self.start()
        self.find_header()
        counter = 0
        while not rospy.is_shutdown():
            counter += 1
            data = self.read()
            data = self.extract_bytes(data)
            self.check_tare(data)
            correct_data = list(np.array(data))
            self.data = correct_data
        #self.collect_data(correct_data)
            self.publish_data()
            
            
            
        self.sensor.close()

        #self.stop()

if __name__ == '__main__':
    sensor_controller = SensorController()
    sensor_controller.run()
