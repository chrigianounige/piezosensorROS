#!/usr/bin/env python3

import serial
import struct
import numpy as np
import time
import rospy
from piezosensorROS.msg import Piezosensor, Thresholds
from piezosensorROS.srv import Tare, TareResponse

class SensorController:
    def __init__(self):
        rospy.init_node('controller', anonymous=False)

        # === ROS interfaces ===
        self.Pub = rospy.Publisher('/piezosensor', Piezosensor, queue_size=10)
        self.ThresholdPub = rospy.Publisher('/thresholds', Thresholds, queue_size=2)
        self.TareService = rospy.Service('/tare', Tare, self.tare)

        # === Parameters ===
        self.rate_hz = rospy.get_param("~rate", 3000)
        self.n_sensors = rospy.get_param("~n_sensors", 8)
        self.start_tare = rospy.get_param("~start_tare", False)
        self.tare_window = rospy.get_param("~tare_window", 1000)
        self.tare_method = rospy.get_param("~tare_method", "std")  # "std" or "percentiles"
        self.tare_coefficient = rospy.get_param("~tare_coefficient", 3.0)
        rospy.loginfo(f"Number of sensors: {self.n_sensors}")

        # === Buffers and thresholds ===
        self.tare_base = np.zeros(self.n_sensors)
        self.tare_std = np.zeros(self.n_sensors)
        self.tare_buffer = []
        self.th_up = np.zeros(self.n_sensors)
        self.th_down = np.ones(self.n_sensors) * (2 ** 16 - 1)
        self.tare_counter = 0

        # === Serial configuration ===
        self.sensor = serial.Serial(stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        self.sensor.port = self.configure_port()
        self.sensor.baudrate = int(1e6)
        self.sensor.timeout = 0.05
        self.sensor.write_timeout = 0.05

        # === Frame configuration ===
        self.header = b'\x3c\x3e\x00'
        self.n_bytes = self.n_sensors * 2 + len(self.header) // 2
        self.rate = rospy.Rate(self.rate_hz)

        # === Sensor group start commands ===
        if self.n_sensors >= 4:
            self.start_bytes1 = bytes.fromhex(rospy.get_param("~str1", "num1,17,18,19,20\r\n").encode('ascii').hex())
        if self.n_sensors >= 8:
            self.start_bytes2 = bytes.fromhex(rospy.get_param("~str2", "num2,21,22,23,24\r\n").encode('ascii').hex())
        if self.n_sensors >= 12:
            self.start_bytes3 = bytes.fromhex(rospy.get_param("~str3", "num3,25,26,27,28\r\n").encode('ascii').hex())
        if self.n_sensors >= 16:
            self.start_bytes4 = bytes.fromhex(rospy.get_param("~str4", "num4,29,30,31,32\r\n").encode('ascii').hex())

        self.data = np.zeros(self.n_sensors, dtype=np.int32)


    def configure_port(self):
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if "USB" in p.description or "ttyACM" in p.device or "ttyUSB" in p.device:
                rospy.loginfo(f"Detected possible sensor port: {p.device}")
                return p.device
        rospy.logwarn("No USB serial ports detected.")
        return None


    def start(self):
        if not self.sensor.port:
            rospy.logerr("No valid serial port configured. Aborting start.")
            return

        try:
            # Configure before opening
            self.sensor.open()
            rospy.loginfo(f"Piezo sensor connected on {self.sensor.port}")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port {self.sensor.port}: {e}")
            return

        time.sleep(0.5)
        self.sensor.reset_input_buffer()
        self.sensor.reset_output_buffer()

        # Send initialization sequences and confirm acknowledgment
        for start_bytes in [getattr(self, f"start_bytes{i}", None) for i in range(1, 5)]:
            if start_bytes is not None:
                self.sensor.write(start_bytes)
                if not self.wait_for_header(timeout=2.0):
                    rospy.logwarn(f"No valid header received after writing: {start_bytes}")
                else:
                    rospy.loginfo(f"Sensors initialized: {start_bytes[4:]}")
            time.sleep(0.5)
        rospy.loginfo("Sensor controller started successfully.")


    def wait_for_header(self, timeout=0.5):
        """
        Reads one byte at a time until the header sequence is found or timeout expires.
        Returns True if found, False otherwise.
        """
        start_time = time.time()
        header_len = len(self.header)
        buffer = bytearray()

        while (time.time() - start_time) < timeout:
            byte = self.sensor.read(1)
            if not byte:
                continue
            buffer += byte

            # Keep buffer length limited to header size
            if len(buffer) > header_len:
                buffer.pop(0)

            # Check if buffer matches header
            if bytes(buffer) == self.header:
                return True
            
        return False

    
    def read(self):
        """
        Reads one complete frame of data after detecting a header.
        Uses buffering to align packets and tolerate dropped bytes.
        """
        try:
            buffer = self.sensor.read_until(self.header)
            if self.header not in buffer:
                #rospy.logwarn("Header not found in stream.")
                return None
            payload = self.sensor.read(self.n_sensors * 2)
            if len(payload) < self.n_sensors * 2:
                rospy.logwarn("Incomplete data frame received.")
                return None
            return payload
        except serial.SerialException as e:
            rospy.logerr(f"Serial read error: {e}")
            return None


    def extract_bytes(self, data_bytes, n_bytes=2):
        """
        Efficient conversion of byte data to integer list.
        """
        if data_bytes is None:
            return [0] * self.n_sensors

        try:
            # Unpack all values in one go using struct
            fmt = f">{self.n_sensors}H"  # Big endian unsigned short
            values = list(struct.unpack(fmt, data_bytes))
            return values
        except struct.error as e:
            rospy.logwarn(f"Struct unpack failed: {e}")
            return [0] * self.n_sensors

            
    def publish_data(self):
        msg = Piezosensor()
        msg.data = self.data
        msg.th_up = self.th_up
        msg.th_down = self.th_down
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
        self.tare_coefficient = req.k
        self.start_tare = True
        self.tare_counter = 0
        self.tare_buffer = []
        self.tare_base = np.zeros(self.n_sensors)
        self.tare_std = np.zeros(self.n_sensors)
        rospy.loginfo(f"Tare started with coefficient {self.tare_coefficient}")
        return TareResponse(True)
    
    
    def compute_tare_std(self, data):
        if self.start_tare and self.tare_counter < self.tare_window:
            self.tare_base += np.array(data)
            self.tare_std += np.array(data)**2
            self.tare_counter += 1
        elif self.start_tare and self.tare_counter == self.tare_window:
            tare_values = self.tare_base//self.tare_window
            self.tare_std = np.sqrt(self.tare_std//self.tare_window - tare_values**2)
            self.th_up = tare_values + 2*(self.tare_std)
            self.th_down = tare_values - 2*(self.tare_std)
            self.start_tare = False
            self.tare_counter = 0
            self.tare_base = np.array([0 for i in range(self.n_sensors)])
            self.publish_thresholds()
            rospy.loginfo("Tare completed")
    

    def compute_tare_percentiles(self, data):
        if self.start_tare and self.tare_counter < self.tare_window:
            self.tare_buffer.append(data)
            self.tare_counter += 1
        elif self.start_tare and self.tare_counter == self.tare_window:
            tare_buffer_array = np.array(self.tare_buffer)
            tare_median = np.median(tare_buffer_array, axis=0)
            tare_p20 = np.percentile(tare_buffer_array, 20, axis=0)
            tare_p80 = np.percentile(tare_buffer_array, 80, axis=0)
            self.th_up = tare_median + self.tare_coefficient*(tare_p80 - tare_median)
            self.th_down = tare_median - self.tare_coefficient*(tare_median - tare_p20)
            self.start_tare = False
            self.tare_counter = 0
            self.publish_thresholds()
            rospy.loginfo("Tare completed")


    def run(self):
        self.start()
        rospy.loginfo("Starting main data acquisition loop...")
        while not rospy.is_shutdown():
            data_bytes = self.read()
            data = self.extract_bytes(data_bytes)

            if self.tare_method == "std":
                self.compute_tare_std(data)
            elif self.tare_method == "percentiles":
                self.compute_tare_percentiles(data)
            else:
                rospy.logwarn("Invalid tare method. Use 'std' or 'percentiles'.")
                self.start_tare = False

            self.data = data
            self.publish_data()

        if self.sensor.is_open:
            self.sensor.close()
            rospy.loginfo("Serial port closed cleanly.")
        

if __name__ == '__main__':
    sensor_controller = SensorController()
    sensor_controller.run()
