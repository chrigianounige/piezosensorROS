#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.lines import Line2D
from collections import deque
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
        self.th_down = np.array([2**16-1] * self.n_sensors)

        # Setup figura
        self.fig, self.ax = plt.subplots()
        self.lines = [self.ax.plot([], [], label=f"Sensor {i+1}")[0] for i in range(self.n_sensors)]
        self.th_up_lines = []
        self.th_down_lines = []

        x_data = range(self.buffer_size)
        for i, line in enumerate(self.lines):
            color = line.get_color()
            th_up_line = Line2D(x_data, [self.th_up[i]] * self.buffer_size, color=color, linestyle='--', alpha=0.7)
            th_down_line = Line2D(x_data, [self.th_down[i]] * self.buffer_size, color=color, linestyle=':', alpha=0.7)
            self.th_up_lines.append(th_up_line)
            self.th_down_lines.append(th_down_line)
            self.ax.add_line(th_up_line)
            self.ax.add_line(th_down_line)

        self.ax.set_xlim(0, self.buffer_size)
        self.ax.set_ylim(0, 65535)
        self.ax.set_xlabel("Campioni")
        self.ax.set_ylabel("Valore Sensore")
        self.ax.set_title(f"Dati Sensori Piezoelettrici (Ultimi {self.buffer_size} Campioni)")
        self.ax.legend()

        # Subscriber ROS
        rospy.Subscriber('/piezosensor', Piezosensor, self.callback)
        rospy.Subscriber('/thresholds', Thresholds, self.threshold_callback)


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


    def run(self):
        rate = rospy.Rate(100)  # Frequenza di aggiornamento
        while not rospy.is_shutdown():
            for i in range(self.n_sensors):
                self.lines[i].set_xdata(range(self.buffer_size))
                self.lines[i].set_ydata(list(self.data_buffer[i]))
                self.th_up_lines[i].set_xdata(range(self.buffer_size))
                self.th_down_lines[i].set_xdata(range(self.buffer_size))
                self.th_up_lines[i].set_ydata([self.th_up[i]] * self.buffer_size)
                self.th_down_lines[i].set_ydata([self.th_down[i]] * self.buffer_size)

            plt.pause(0.01)  # Aggiorna il grafico
            rate.sleep()

if __name__ == '__main__':
    plotter = PlotterNode()
    plotter.run()

        
