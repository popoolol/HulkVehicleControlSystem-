import tkinter as tk
import serial
import time
from lidar import get_distance

ser = serial.Serial('/dev/ttyACM0', 115200)

def send_command(command):
	
	ser.write(command.encode())


class BasicController:
	def __init__(self, master):
		self.master = master
		self.label = tk.Label(master, text="Basic Controller")
		self.label.pack()
	        
	        # Create a frame to hold the buttons
		self.button_frame = tk.Frame(master)
		self.button_frame.pack()
        
	        # Create the directional buttons
		self.forward_button = tk.Button(self.button_frame, text="Forward", command=self.move_forward, width = 15, height = 10)
		self.forward_button.pack(side=tk.TOP)
        
		self.backward_button = tk.Button(self.button_frame, text="Backward", command=self.move_backward, width = 15, height = 10)
		self.backward_button.pack(side=tk.BOTTOM)
        
		self.left_button = tk.Button(self.button_frame, text="Left", command=self.move_left, width = 15, height = 10)
		self.left_button.pack(side=tk.LEFT)
        
		self.right_button = tk.Button(self.button_frame, text="Right", command=self.move_right, width = 15, height = 10)
		self.right_button.pack(side=tk.RIGHT)
        
		self.ccw_button = tk.Button(self.button_frame, text="CCW", command=self.move_ccw, width = 15, height = 10)
		self.ccw_button.pack(side=tk.LEFT)
        
		self.cw_button = tk.Button(self.button_frame, text="CW", command=self.move_cw, width = 15, height = 10)
		self.cw_button.pack(side=tk.RIGHT)
        
		self.start_simple_path = tk.Button(self.button_frame, text="Start Simple Path", command=self.simple_path, width = 15, height = 10)
		self.start_simple_path.pack(side=tk.TOP)
	
	        # Create the stop button
		self.stop_button = tk.Button(master, text="Stop", command=self.stop, width = 50, height = 30)
		self.stop_button.pack(side=tk.BOTTOM)

		self.dummy = tk.Button(master, text="Start")
		self.dummy.pack(side=tk.TOP)
	
	def move_forward(self):
		print("Moving forward")
		send_command("A")

	def move_backward(self):
		print("Moving backward")
		send_command("C")
        
	def move_left(self):
		print("Moving left")
		send_command("E")

	def move_right(self):
		print("Moving right")
		send_command("D")

	def move_ccw(self):
		print("Turning counterclockwise")
		send_command("G")

	def move_cw(self):
		print("Turning clockwise")
		send_command("F")

	def stop(self):
		print("Stopping")
		send_command("B")

	def simple_path(self):
		while True:
			lidar_distance = get_distance()
			if(lidar_distance <= 15):
				print("stop moving")
				self.stop()
				break
			else:
				print("Moving forward")
				self.move_forward()

if __name__ == "__main__":
	root = tk.Tk()
	app = BasicController(root)
	root.mainloop()
 
