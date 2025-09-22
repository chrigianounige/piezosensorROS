#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_controller.msg import Piezosensor
import time
from pathlib import Path
import os

########################## TODO ##########################

class DataSaver:
    def __init__(self):
        rospy.init_node('data_saver', anonymous=False)
        self.buffer = []
        self.save_interval = 10000  # Salva ogni 10000 campioni
        self.data_dir = Path.home() / "piezosensor_data"
        self.file_counter = 0

        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)

        rospy.Subscriber('/piezosensor', Piezosensor, self.callback)

    def callback(self, msg):
        # Aggiungi i dati nel buffer
        self.buffer.append(msg.data)

        # Se il buffer raggiunge la dimensione definita, salva i dati
        if len(self.buffer) >= self.save_interval:
            self.save_data()

    def save_data(self):
        if len(self.buffer) > 0:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f"sensor_data_{timestamp}_{self.file_counter}.csv"
            filepath = os.path.join(self.data_dir, filename)
            
            # Salvataggio dei dati in formato CSV
            np.savetxt(filepath, self.buffer, delimiter=',')
            rospy.loginfo(f"Saved data to {filepath}")
            
            self.file_counter += 1

        self.buffer = []  # Reset the buffer after saving

    def run(self):
        rospy.spin()  # Keep the node running

if __name__ == '__main__':
    data_saver = DataSaver()
    data_saver.run()