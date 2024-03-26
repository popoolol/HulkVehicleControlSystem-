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
scale_factors = [.15, .10] 
template_image = cv2.imread('template.png')



def initialize_camera():
    """
    Initialize the camera and return pipeline, align object, and template image.
    """
    # Initialize Intel RealSense
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable color and depth streams
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Create align object to align depth frames to color frame
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Start streaming
    profile = pipeline.start(config)

    template_image = cv2.imread('template.png')

    return pipeline, align, template_image


def detect_rotated_template(pipeline, align, template_image, threshold=0.70, scale = .10):
    """
    Detect a template rotated 90 degrees and return:
    - 0 if the template is within 5 pixels of the center of the camera
    - 1 if the template is to the left of the center
    - 2 if the template is to the right of the center
    """
    # Rotate the template image 90 degrees
    rotated_template = cv2.rotate(cv2.resize(template_image, None, fx=scale, fy=scale), cv2.ROTATE_90_CLOCKWISE)
    
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    
    # Find the best match
    result = cv2.matchTemplate(color_image, rotated_template, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    
    # If the match is strong enough
    if max_val >= threshold:
        # Determine the horizontal position of the match
        match_center = max_loc[0] + rotated_template.shape[1] / 2
        
        camera_center = color_image.shape[1] / 2
        if abs(match_center - camera_center) <= 5:
            return 0
        elif match_center < camera_center:
            return 1
        else:
            return 2
    else:
        return None

def calculate_skew(pipeline, align, template_image, scale_factors = [.15, .10], threshold = 0.70):
    """
    Calculate the skew of the camera and return it.
    """
    # Function for calculating skew angle
    def get_camera_skew(centers_and_depths):
        dx = 0.36195
        if (centers_and_depths[0][0] < centers_and_depths[1][0]):
            dz = centers_and_depths[1][1] - centers_and_depths[0][1]
        elif(centers_and_depths[0][0] > centers_and_depths[1][0]):
            dz = centers_and_depths[0][1] - centers_and_depths[1][1]
        else:
            dz = 0    
        skew_angle = np.arctan2(dx, dz) 
        return np.degrees(skew_angle) - 90

    # Capture frames and calculate skew
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    centers_and_depths = []
    boxes = []
    for scale in scale_factors:
        resized_template = cv2.resize(template_image, None, fx=scale, fy=scale)
        result = cv2.matchTemplate(color_image, resized_template, cv2.TM_CCOEFF_NORMED)
        locations = np.where(result >= threshold)

        for loc in zip(*locations[::-1]):
            top_left = loc
            bottom_right = (top_left[0] + resized_template.shape[1], top_left[1] + resized_template.shape[0])
            boxes.append([top_left[0], top_left[1], bottom_right[0], bottom_right[1]])

    boxes = np.array(boxes)
    indices = cv2.dnn.NMSBoxes(boxes.tolist(), [1]*len(boxes), threshold, 0.9)

    for i in indices:
        i = i.item(0)
        box = boxes[i]
        top_left = (box[0], box[1])
        bottom_right = (box[2], box[3])
        center_x = int((top_left[0] + bottom_right[0]) / 2)
        depth = depth_frame.get_distance(center_x, int((top_left[1] + bottom_right[1]) / 2))
        centers_and_depths.append((center_x, depth))

    if len(centers_and_depths) >= 2:
        skew_angle = get_camera_skew(centers_and_depths)
        return skew_angle
    else:
        return None

def close_camera(pipeline):
    """
    Stop the camera and close any open windows.
    """
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

        button_width = 8
        button_height = 4

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
        
        # Initialize the camera
        pipeline, align, template_image = initialize_camera()
            
            
        while True:
            time.sleep(.5)
            skew = calculate_skew(pipeline, align, template_image)
            skew = calculate_skew(pipeline, align, template_image)
            if skew is not None:
                print(f"Camera skew: {skew} degrees")

                while True:
                    time.sleep(.1)
                    skew = calculate_skew(pipeline, align, template_image)


                    if skew is not None:
                        print(f"Camera skew: {skew} degrees")
                    else: 
                        print("Skew templates are not in position")

                    
                    lidar_distance = get_distance()
                    print(f"lidar distance: {lidar_distance}")

                    if(lidar_distance <= 15):
                                print("stop moving")
                                self.stop()
                                break
                    
                    if skew == None and rotated_template_position == None:
                        break

                    elif lidar_distance < 80 or skew < 1 and skew > -1: 
                        rotated_template_position = detect_rotated_template(pipeline, align, template_image)
                        print ("skew is acceptable or not viewable")
                        print (f"Center template is in position {rotated_template_position}")

                        if rotated_template_position == None:
                            break
                        elif rotated_template_position == 0:
                            self.move_forward()

                            lidar_distance = get_distance()
                            if(lidar_distance <= 15):
                                print("stop moving")
                                self.stop()
                                break
                        elif rotated_template_position == 2:
                            self.move_right()
                            time.sleep(.1)
                            self.stop()
                        elif rotated_template_position == 1:
                            self.move_left()
                            time.sleep(.1)
                            self.stop()

                    elif skew > 1:
                        print("turning clockwise to account for skew")
                        self.move_cw()
                        time.sleep(.05)
                        self.stop()
                        time.sleep(.05)

                    elif skew < -1:
                        print("turning counter-clockwise to account for skew")
                        self.move_ccw()
                        time.sleep(.05)
                        self.stop()
                        time.sleep(.05)
                        
            else:
                print("Insufficient templates detected to calculate skew.")


            lidar_distance = get_distance()
            if(lidar_distance <= 15):
                print("stop moving")
                self.stop()
                break
        
        

        close_camera(pipeline)
            
            

if __name__ == "__main__":
    root = tk.Tk()
    app = BasicController(root)
    root.mainloop()
    send_command("B")



 




