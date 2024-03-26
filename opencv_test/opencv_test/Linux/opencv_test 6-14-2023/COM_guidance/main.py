import tkinter as tk
import serial
import time
import mainLib
from mainLib import BasicController, send_command
import cv2
import numpy as np
import pyrealsense2.pyrealsense2 as rs
import time


ser = serial.Serial('/dev/ttyACM0', 115200)
threshold = 0.70 # Higher values match more precisely
scale_factors = [.15, .10] 
template_image = cv2.imread('template.png')

            
if __name__ == "__main__":
    root = tk.Tk()
    app = BasicController(root)
    root.mainloop()
    send_command("B")



 




