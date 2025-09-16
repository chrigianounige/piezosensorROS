#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.lines import Line2D
from collections import deque
from matplotlib.animation import FuncAnimation
from piezosensorROS.msg import Piezosensor, Thresholds

class PlotterNode:
    def __init__(self):
        rospy.init_node('plotter_node', anonymous=True)

        # Parametri
        self.buffer_size = rospy.get_param("~buffer_size", 1000)
        self.n_sensors = rospy.get_param("~n_sensors", 8)
        self.data_buffer = [deque([0]*self.buffer_size, maxlen=self.buffer_size) for _ in range(self.n_sensors)]

        # Soglie
        self.th_up = np.array([0] * self.n_sensors)
        self.th_down = np.array([2**16] * self.n_sensors)

        # Setup figura
        self.fig, self.ax = plt.subplots()
        self.lines = [self.ax.plot([], [], label=f"Sensor {i+1}")[0] for i in range(self.n_sensors)]
        self.th_up_lines = []
        self.th_down_lines = []

        for i, line in enumerate(self.lines):
            color = line.get_color()
            self.th_up_lines.append(Line2D([], [], color=color, linestyle='--', alpha=0.7))
            self.th_down_lines.append(Line2D([], [], color=color, linestyle=':', alpha=0.7))
            self.ax.add_line(self.th_up_lines[-1])
            self.ax.add_line(self.th_down_lines[-1])

        self.ax.set_xlim(0, self.buffer_size)
        self.ax.set_ylim(0, 65535)
        self.ax.set_xlabel("Campioni")
        self.ax.set_ylabel("Valore Sensore")
        self.ax.set_title(f"Dati Sensori Piezoelettrici (Ultimi {self.buffer_size} Campioni)")
        self.ax.legend()

        # Subscriber ROS
        rospy.Subscriber('/piezosensors', Piezosensor, self.callback)
        rospy.Subscriber('/thresholds', Thresholds, self.threshold_callback)

        # Animazione
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False, cache_frame_data=False)

    def callback(self, msg):
        data = msg.data
        if len(data) != self.n_sensors:
            rospy.logwarn("Numero di sensori ricevuti non corrisponde!")
            return
        for i in range(self.n_sensors):
            self.data_buffer[i].append(data[i])

    def threshold_callback(self, msg):
        self.th_up = msg.th_up
        self.th_down = msg.th_down
        rospy.loginfo("Soglie aggiornate!")

    def update_plot(self, frame):
        for i in range(self.n_sensors):
            ydata = list(self.data_buffer[i])
            self.lines[i].set_data(range(len(ydata)), ydata)
            self.th_up_lines[i].set_data(range(self.buffer_size), [self.th_up[i]]*self.buffer_size)
            self.th_down_lines[i].set_data(range(self.buffer_size), [self.th_down[i]]*self.buffer_size)
        return self.lines + self.th_up_lines + self.th_down_lines

    def run(self):
        plt.show()  # GUI Matplotlib nel thread principale
        rospy.spin()  # ROS gestisce i callback

if __name__ == '__main__':
    plotter = PlotterNode()
    plotter.run()
