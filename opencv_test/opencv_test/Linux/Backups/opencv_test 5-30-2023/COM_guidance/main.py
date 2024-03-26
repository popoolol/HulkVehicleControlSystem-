import tkinter as tk
import serial
import time
from lidar import get_distance
import cv2
import numpy as np
import pyrealsense2.pyrealsense2 as rs
import time


ser = serial.Serial('/dev/ttyACM0', 115200)
threshold = 0.70 # Higher values match more precisely
scale_factors = [.15, .10, .05] 
template_image = cv2.imread('template.png')

def start_camera():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    return pipeline, config, 640  # width of the image frame


def start_camera():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    return pipeline, config, 640  # width of the image frame

def detect_green_blob(pipeline, config, width):
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        return 3  # No blob detected

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())

    # Convert BGR image to HSV
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

    # Define color range for green color in HSV
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])

    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv_image, lower_green, upper_green)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours and calculate center coordinates of the blobs
    for contour in contours:
        area = cv2.contourArea(contour)
        # Filter out small noise by comparing the area to a threshold
        if area > 50:  # Lower the threshold to detect smaller blobs
            # Calculate center coordinates of the contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                center_coordinates = (cx, cy)

                # Draw a circle at the center of the blob
                cv2.circle(color_image, center_coordinates, 5, (0, 255, 0), -1)

                # Display the coordinates as text values
                cv2.putText(color_image, f"({cx}, {cy})", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Compare the center of the blob to the center of the image
                image_center = width // 2
                if abs(image_center - cx) <= 5:
                    return 0  # Blob is in the center
                elif cx < image_center:
                    return 1  # Blob is to the left
                else:
                    return 2  # Blob is to the right

    # Show images
    cv2.imshow('Detected Green Blob', color_image)
    
    return 3  # No blob detected

def stop_camera(pipeline):
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
    

# Display the number of detected templates on the frame
    cv2.putText(color_image, f"Detected templates: {detected_templates}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow('Result', color_image)
    
    return 3  # No blob detected

def stop_camera(pipeline):
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()


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

        button_width = 5
        button_height = 2

        # Create the directional buttons
        self.forward_button = tk.Button(self.button_frame, text="Forward", command=self.move_forward, width = button_width, height = button_height)
        self.forward_button.pack(side=tk.TOP)

        self.backward_button = tk.Button(self.button_frame, text="Backward", command=self.move_backward, width = button_width, height = button_height)
        self.backward_button.pack(side=tk.BOTTOM)

        self.left_button = tk.Button(self.button_frame, text="Left", command=self.move_left, width = button_width, height = button_height)
        self.left_button.pack(side=tk.LEFT)

        self.right_button = tk.Button(self.button_frame, text="Right", command=self.move_right, width = button_width, height = button_height)
        self.right_button.pack(side=tk.RIGHT)

        self.ccw_button = tk.Button(self.button_frame, text="CCW", command=self.move_ccw, width = button_width, height = button_height)
        self.ccw_button.pack(side=tk.LEFT)

        self.cw_button = tk.Button(self.button_frame, text="CW", command=self.move_cw, width = button_width, height = button_height)
        self.cw_button.pack(side=tk.RIGHT)

        # rotate cw 90 test
        self.r90_button = tk.Button(master, text="rotateCW90", command=self.rotatecw90, width = button_width, height = button_height)
        self.r90_button.pack(side=tk.TOP)

        #rotate ccw 90 test
        self.r90_button = tk.Button(master, text="rotateCCW90", command=self.rotateccw90, width = button_width, height = button_height)
        self.r90_button.pack(side=tk.TOP)

        self.start_simple_path = tk.Button(self.button_frame, text="Start Simple Path", command=self.simple_path, width = button_width, height = button_height)
        self.start_simple_path.pack(side=tk.TOP)

        # Create the stop button
        self.stop_button = tk.Button(master, text="Stop", command=self.stop, width = 50, height = 30)
        self.stop_button.pack(side=tk.BOTTOM)

        # Create an emergency stop button
        self.emergency_stop_button = tk.Button(master, text="Emergency Stop", command=self.emergency_stop, bg='red', width = 50, height = 30)
        self.emergency_stop_button.pack(side=tk.BOTTOM)


    def emergency_stop(self):
        print("Emergency stopping")
        self.stop()
        self.master.destroy()  # or self.master.destroy(), depending on the desired behavior

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
    def rotatecw90(self):
        print("rotating")
        self.move_cw()
        time.sleep(3) # testing a delay of 2 seconds
        self.stop()
        print("stopping")
    def rotateccw90(self):
        print("rotating")
        self.move_ccw()
        time.sleep(3) # testing a delay of 2 seconds
        self.stop()
        print("stopping")


    def simple_path(self):
        try:
            pipeline, config, width = start_camera()
            strafe_check = 10 #how many times the robot will strafe. this improves accuracy
            
            while True:
                time.sleep(.3)
                
                lidar_distance = get_distance()
                if(lidar_distance <= 15):
                    print("stop moving")
                    self.stop()
                    break
        
        
        except KeyboardInterrupt:
            stop_camera(pipeline)
            
            

if __name__ == "__main__":
    root = tk.Tk()
    app = BasicController(root)
    root.mainloop()



 




