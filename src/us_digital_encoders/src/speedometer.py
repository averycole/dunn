#!/usr/bin/env python

import rospy
import Tkinter as tk
import tkFont
from us_digital_encoders.msg import USDigitalEncoders
from std_msgs.msg import Float64

# Speedometer(ticks_per_metre, drive_ticks) where drive_ticks is 'left', 'right', or 'both'.
# left: USDigitialEncoders.ticks[0]
# right: USDigitialEncoders.ticks[1]
# both: USDigitialEncoders.ticks[0] and USDigitalEncoders[1] (take average of the two)

class Speedometer(object):

    def __init__(self, ticks_per_metre, drive_ticks='both', window=None):
        rospy.init_node('speedometer')
        self.ticks_per_metre = ticks_per_metre
        self.drive_ticks = drive_ticks
        self.window = window
        self.last_msg_time = None
        self.sub = rospy.Subscriber('encoders', USDigitalEncoders, self._callback)
        self.pub_mps = rospy.Publisher('speed/metres_per_second', Float64, queue_size=5)
        self.pub_kph = rospy.Publisher('speed/kilometres_per_hour', Float64, queue_size=5)

    def _callback(self, msg):
        ticks = self._get_ticks(msg)
        timestamp = msg.header.stamp.to_sec()
        if self.last_msg_time is None:
            self.last_msg_time = timestamp
        else:
            speed = self._ticks_to_speed(ticks, timestamp)
            if self.window is not None:
                self._set_window_text(speed)
            self.pub_mps.publish(speed)
            self.pub_kph.publish(speed * 3.6)
            self.last_msg_time = timestamp

    def _get_ticks(self, msg):
        if self.drive_ticks == 'left':
            ticks = float(msg.ticks[0])
        elif self.drive_ticks == 'right':
            ticks = float(msg.ticks[1])
        else:
            ticks = (msg.ticks[0] + msg.ticks[1]) / 2.0
        return ticks

    def _set_window_text(self, speed):
        self.window.speed = speed
    
    def _ticks_to_speed(self, ticks, timestamp):
        delta_metres = ticks / self.ticks_per_metre
        speed = delta_metres / (timestamp - self.last_msg_time)
        return speed

class SpeedometerWindow(tk.Frame):
    
    def __init__(self, parent, units="kph"):
        tk.Frame.__init__(self, parent, background="white")
        self.parent = parent
        self.units = units
        self.init_ui()
        self.speed = 0.0
        self.set_speed_text()

    def init_ui(self):
        self.parent.title("Speedometer")
        self.font = tkFont.Font(family='Courier', size='180', weight='bold')
        self.speed_text = tk.Text(self.parent, font=self.font)
        text = "0.0 km/h"
        self.speed_text.insert(tk.END, text)
        self.speed_text.pack(fill=tk.BOTH, expand=True)

    def set_speed_text(self):
        self.speed_text.delete(1.0, tk.END)
        text = "%.1f\nkm/h" % (abs(self.speed * 3.6))
        self.speed_text.insert(tk.END, text)
        self.after(500, self.set_speed_text)


if __name__ == '__main__':
    root = tk.Tk()
    root.geometry("800x600+300+300")
    app = SpeedometerWindow(root)
    speedometer = Speedometer(47363, drive_ticks='both', window=app)
    root.mainloop()
