#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

import tkinter as tk


class SliderController:
    def __init__(self, master, name, topic, min_value, max_value, color):
        self.pub = rospy.Publisher(topic, Float64, queue_size=10)

        self.label = tk.Label(master, text=name)
        self.label.pack()

        self.slider = tk.Scale(master, from_=min_value, to=max_value, orient=tk.HORIZONTAL, length=300, width=20, sliderlength=10, bg=color, fg="snow", command=self.slider_callback)
        self.slider.pack()

        # Create a small space between the slider and next label
        self.space = tk.Label(master, text=" ")
        self.space.pack()

    def slider_callback(self, value):
        # Convert the slider value to radians
        angle = float(value) * 3.14159 / 180.0

        self.pub.publish(Float64(angle))


class MasterController:
    def __init__(self):
        rospy.init_node('arm_controller')

        self.root = tk.Tk()
        self.root.title("Robot Controller")

        # Set the padding around window
        self.root.geometry("+50+50")

        # Create individual sliders for each joint
        _ = SliderController(self.root, "Controller 1", "/g05/Revolute_1_position_controller/command", -180, 180, "blue4")
        _ = SliderController(self.root, "Controller 2", "/g05/Revolute_2_position_controller/command", -180, 180, "blue4")
        _ = SliderController(self.root, "Controller 3", "/g05/Revolute_3_position_controller/command", -180, 180, "blue4")
        _ = SliderController(self.root, "Controller 4", "/g05/Revolute_4_position_controller/command", -180, 180, "blue4")
        _ = SliderController(self.root, "Controller 5", "/g05/Revolute_5_position_controller/command", -180, 180, "blue4")
        _ = SliderController(self.root, "Controller 6", "/g05/Revolute_6_position_controller/command", -180, 180, "blue4")

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    try:
        controller = MasterController()
        controller.run()
    except rospy.ROSInterruptException:
        pass