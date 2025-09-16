#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
from collections import deque
from matplotlib.lines import Line2D
from piezosensorROS.msg import Piezosensors, Thresholds # Metti il nome del progetto ROS

class PlotterNode:
    def __init__(self):
        rospy.init_node('plotter_node', anonymous=True)

        # Buffer per memorizzare gli ultimi buffer_size valori per ogni sensore
        self.buffer_size = rospy.get_param("~buffer_size", 1000)
        self.n_sensors = rospy.get_param("~n_sensors", 8)  # Cambia questo numero se hai più o meno sensori
        self.data_buffer = [deque([0]*self.buffer_size, maxlen=self.buffer_size) for _ in range(self.n_sensors)]

        # Inizializza le soglie
        self.th_up = np.array([0] * self.n_sensors)
        self.th_down = np.array([2**16] * self.n_sensors)


        # Setup della finestra di plot
        self.fig, self.ax = plt.subplots()
        self.lines = [self.ax.plot([], [], label=f"Sensor {i+1}")[0] for i in range(self.n_sensors)]
        # Linee delle soglie
        # Linee delle soglie (adesso sono Line2D)
        self.th_up_lines = []
        self.th_down_lines = []
        for i, line in enumerate(self.lines):
            color = line.get_color()
            # ora crea le due soglie con lo stesso colore
            self.th_up_lines.append(Line2D([], [], color=color, linestyle='--', alpha=0.7))
            self.th_down_lines.append(Line2D([], [], color=color, linestyle=':', alpha=0.7))

        for line in self.th_up_lines:
            self.ax.add_line(line)

        for line in self.th_down_lines:
            self.ax.add_line(line)

        self.ax.set_xlim(0, self.buffer_size)
        self.ax.set_ylim(0, 65535)  # Regola i limiti in base ai tuoi dati
        self.ax.set_xlabel("Campioni")
        self.ax.set_ylabel("Valore Sensore")
        self.ax.set_title(f"Dati Sensori Piezoelettrici (Ultimi {self.buffer_size} Campioni)")
        self.ax.legend()

        # Sottoscrizione al topic dei sensori
        rospy.Subscriber('/piezosensors', sensors, self.callback)
        rospy.Subscriber('/thresholds', thresholds, self.threshold_callback)

    def callback(self, msg):
        data = msg.data  # Lista di valori ricevuti

        if len(data) != self.n_sensors:
            rospy.logwarn("Numero di sensori ricevuti non corrisponde!")
            return

        # Aggiunge i nuovi dati al buffer
        for i in range(self.n_sensors):
            self.data_buffer[i].append(data[i])


    def threshold_callback(self, msg):
        """ Callback per aggiornare le soglie """
        self.th_up = msg.th_up
        self.th_down = msg.th_down
        rospy.loginfo("Soglie aggiornate!")


    def run(self):
        rate = rospy.Rate(100)  # Frequenza di aggiornamento
        while not rospy.is_shutdown():
            #self.ax.set_ylim(min(min(self.data_buffer)), max(max(self.data_buffer)))  # Adatta l'asse Y
            for i in range(self.n_sensors):
                self.lines[i].set_xdata(range(self.buffer_size))
                self.lines[i].set_ydata(list(self.data_buffer[i]))

                self.th_up_lines[i].set_xdata(range(self.buffer_size))
                self.th_up_lines[i].set_ydata([self.th_up[i]] * self.buffer_size)

                self.th_down_lines[i].set_xdata(range(self.buffer_size))
                self.th_down_lines[i].set_ydata([self.th_down[i]] * self.buffer_size)

            plt.pause(0.01)  # Aggiorna il grafico
            rate.sleep()

if __name__ == '__main__':
    plotter = PlotterNode()
    plotter.run()
