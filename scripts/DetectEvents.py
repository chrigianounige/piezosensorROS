#!/usr/bin/env python3

import rospy
import numpy as np
from collections import deque
from piezosensorROS.msg import Piezosensor, Thresholds


class EventDetector:
    def __init__(self):
        rospy.init_node('event_detector', anonymous=True)

        # Parametri
        self.n_sensors = rospy.get_param("~n_sensors", 8)
        self.min_number_of_active_sensors = rospy.get_param("~min_number_of_active_sensors", 3)
        self.data = [0 for i in range(self.n_sensors)]
        
        # Soglie
        self.th_up = np.array([0] * self.n_sensors)
        self.th_down = np.array([2**16-1] * self.n_sensors)

        # Subscriber ROS
        rospy.Subscriber('/piezosensor', Piezosensor, self.callback)
        rospy.Subscriber('/thresholds', Thresholds, self.threshold_callback)

    
    def callback(self, msg):
        data = msg.data
        if len(data) != self.n_sensors:
            rospy.logwarn("Numero di sensori ricevuti non corrisponde!")
            return
        for i in range(self.n_sensors):
            self.data = data

    
    def threshold_callback(self, msg):
        self.th_up = msg.th_up
        self.th_down = msg.th_down
        rospy.loginfo("Soglie aggiornate!")


    def check_event(self):
        index_active_sensors = [i for i in range(self.n_sensors) if self.data[i] > self.th_up[i] or self.data[i] < self.th_down[i]]
        active_sensors_count = len(index_active_sensors)
        return index_active_sensors, active_sensors_count


    def run(self):
    rate = rospy.Rate(3000)  # Event checking frequency
    possible_event_detected = False
    event_detected = False
    samples_counter = 0
    window_length = 25  # Number of cycles to confirm the event
    possible_event_buffer = [0] * self.n_sensors  # Track possible event state of each sensor
    event_buffer = [0] * self.n_sensors  # Track confirmed event state of each sensor

    while not rospy.is_shutdown():
        index_active_sensors, active_sensors_count = self.check_event()  # Check if sensors exceed thresholds

        # A possible event is detected (transition from no event to some active sensors)
        if not possible_event_detected and active_sensors_count:
            possible_event_detected = True
            samples_counter = 1
            for idx in index_active_sensors:
                possible_event_buffer[idx] = 1

        elif possible_event_detected:
            samples_counter += 1

            # Still within the confirmation window
            if samples_counter < window_length:
                if active_sensors_count:
                    for idx in index_active_sensors:
                        possible_event_buffer[idx] = 1
            else:
                # Window length reached → finalize decision
                possible_event_detected = False
                samples_counter = 0
                final_active_sensors = [i for i, v in enumerate(possible_event_buffer) if v]

                if not event_detected:
                    # Event press: confirm if enough sensors are active
                    if len(final_active_sensors) >= self.min_number_of_active_sensors:
                        event_detected = True
                        event_buffer = possible_event_buffer.copy()
                        rospy.loginfo(f"Event detected with {final_active_sensors} active sensors.")
                else:
                    # Event release: check how many previously active sensors are active again
                    num_of_actives = sum(event_buffer[i] and (i in final_active_sensors) for i in range(self.n_sensors))
                    if num_of_actives > int(0.5 * self.min_number_of_active_sensors):
                        event_detected = False
                        event_buffer = [0] * self.n_sensors
                        possible_event_buffer = [0] * self.n_sensors
                        rospy.loginfo("Event ended.")
        rate.sleep()
        
if __name__ == '__main__':
    event_detector = EventDetector()
    event_detector.run()    