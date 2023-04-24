import tkinter as tk
import pyrealsense2 as rs
import numpy as np
import cv2
import time
from PIL import Image
from PIL import ImageTk


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Chessboard Calibration")
        self.save_images_flag = False

        # Create a label for the first camera
        self.label1 = tk.Label(self.root, text="Camera 1", font=("TkDefaultFont", 12, "bold"))
        self.label1.pack()

        # Create a canvas for the first camera image
        self.canvas1 = tk.Canvas(self.root, width=848, height=480)
        self.canvas1.pack()

        # Create a label for the second camera
        self.label2 = tk.Label(self.root, text="Camera 2", font=("TkDefaultFont", 12, "bold"))
        self.label2.pack()

        # Create a canvas for the second camera image
        self.canvas2 = tk.Canvas(self.root, width=848, height=480)
        self.canvas2.pack()

        # Create a button to save images
        self.button = tk.Button(self.root, text="Save Images", font=("TkDefaultFont", 12, "bold"), command=self.save_images)
        self.button.pack()

        # Create a button to exit the program
        self.exit_button = tk.Button(self.root, text="Exit", font=("TkDefaultFont", 12, "bold"), command=self.close_window, bg="red", fg="white", activebackground="darkred", activeforeground="white", relief=tk.RAISED)
        self.exit_button.place(relx=0.9, rely=0.968)

        # Configure the RealSense cameras
        self.pipeline_1 = rs.pipeline()
        self.config_1 = rs.config()
        self.config_1.enable_device('233622079610')  # replace with the serial number of the first camera
        self.config_1.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

        self.pipeline_2 = rs.pipeline()
        self.config_2 = rs.config()
        self.config_2.enable_device('233622074753')  # replace with the serial number of the second camera
        self.config_2.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

        # Define the chessboard parameters
        self.chessboard_size = (10, 7)  # The size of the chessboard (number of internal corners)
        self.chessboard_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE  # Flags for cv2.findChessboardCorners()

        self.traffic_light1 = tk.Canvas(self.root, width=50, height=50, bg="red")
        self.traffic_light1.place(relx=0.01, rely=0.03)
        self.traffic_light2 = tk.Canvas(self.root, width=50, height=50, bg="red")
        self.traffic_light2.place(relx=0.01, rely=0.515)


    def save_images(self):
        self.save_images_flag = True

    def close_window(self):
        root.destroy()


    def start(self):
        # Start the RealSense cameras
        self.pipeline_1.start(self.config_1)
        self.pipeline_2.start(self.config_2)

        try:
            # Wait for 5 seconds
            print("Waiting for 2 seconds...")
            time.sleep(2)

            # Capture images
            index = 0
            while True:
                # Wait for a coherent pair of frames from both cameras
                frames_1 = self.pipeline_1.wait_for_frames()
                frames_2 = self.pipeline_2.wait_for_frames()

                color_frame_1 = frames_1.get_color_frame()
                color_frame_2 = frames_2.get_color_frame()

                # Convert the frames to numpy arrays
                color_image_1 = np.asanyarray(color_frame_1.get_data())
                color_image_2 = np.asanyarray(color_frame_2.get_data())
                color_image_1_copy = color_image_1.copy()
                color_image_2_copy = color_image_2.copy()

                # Find the chessboard corners in the images
                gray_image_1 = cv2.cvtColor(color_image_1_copy, cv2.COLOR_BGR2GRAY)
                gray_image_2 = cv2.cvtColor(color_image_2_copy, cv2.COLOR_BGR2GRAY)

                ret_1, corners_1 = cv2.findChessboardCorners(gray_image_1, self.chessboard_size, self.chessboard_flags)
                ret_2, corners_2 = cv2.findChessboardCorners(gray_image_2, self.chessboard_size, self.chessboard_flags)

                # Draw the chessboard corners on the images
                if ret_1 and ret_2:
                    cv2.drawChessboardCorners(color_image_1_copy, self.chessboard_size, corners_1, ret_1)
                    cv2.drawChessboardCorners(color_image_2_copy, self.chessboard_size, corners_2, ret_2)

                if ret_1:
                    cv2.drawChessboardCorners(color_image_1_copy, self.chessboard_size, corners_1, ret_1)
                    self.traffic_light1.config(bg="green")
                else:
                    self.traffic_light1.config(bg="red")
                if ret_2:
                    cv2.drawChessboardCorners(color_image_2_copy, self.chessboard_size, corners_2, ret_2)
                    self.traffic_light2.config(bg="green")
                else:
                    self.traffic_light2.config(bg="red")
                # Convert the OpenCV images to Tkinter images
                image1 = cv2.cvtColor(color_image_1_copy, cv2.COLOR_BGR2RGB)
                image1 = Image.fromarray(image1)
                photo1 = ImageTk.PhotoImage(image1)

                image2 = cv2.cvtColor(color_image_2_copy, cv2.COLOR_BGR2RGB)
                image2 = Image.fromarray(image2)
                photo2 = ImageTk.PhotoImage(image2)

                # Display the images on the canvas
                self.canvas1.create_image(0, 0, anchor=tk.NW, image=photo1)
                self.canvas2.create_image(0, 0, anchor=tk.NW, image=photo2)

                # Update the Tkinter window
                self.root.update()

                # Save the images when the button is pressed
                if self.save_images_flag:
                    cv2.imwrite(f"images/camera_1_{index}.jpg", color_image_1)
                    cv2.imwrite(f"images/camera_2_{index}.jpg", color_image_2)
                    index += 1
                    self.save_images_flag = False

        finally:
            # Stop the RealSense cameras
            self.pipeline_1.stop()
            self.pipeline_2.stop()

    


if __name__ == '__main__':
    root = tk.Tk()
    app = App(root)
    app.start()
    root.mainloop()


