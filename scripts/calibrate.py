import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Ask the user for the number of calibration images
num_images = int(input("Enter the number of calibration images: "))

calibration_object_size = 23.7 # in millimeters

# Define the calibration object
x_squares = 10
y_squares = 7
calibration_object = np.zeros((x_squares * y_squares, 3), np.float32)
calibration_object[:, :2] = np.mgrid[0:x_squares, 0:y_squares].T.reshape(-1, 2) * calibration_object_size

# Initialize the arrays to store the calibration points and the images
calibration_points = []
image_points_1 = []
image_points_2 = []

# Loop over the images and detect the calibration points

for image_idx in range(num_images):
    # Capture an image from each camera
    image_1 = cv2.imread(f'images/camera_1_{image_idx}.jpg')
    image_2 = cv2.imread(f'images/camera_2_{image_idx}.jpg')
    
    # Find the calibration points in both images
    gray_1 = cv2.cvtColor(image_1, cv2.COLOR_BGR2GRAY)
    gray_2 = cv2.cvtColor(image_2, cv2.COLOR_BGR2GRAY)
    
    ret_1, corners_1 = cv2.findChessboardCorners(gray_1, (x_squares, y_squares), None)
    ret_2, corners_2 = cv2.findChessboardCorners(gray_2, (x_squares, y_squares), None)
    
    

    # If the calibration points are found in both images, add them to the arrays
    if ret_1 == True and ret_2 == True:
        calibration_points.append(calibration_object)
        
                # Refine the calibration points to subpixel accuracy
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1000000, 1e-10)
        corners_1 = cv2.cornerSubPix(gray_1, corners_1, (11, 11), (-1, -1), criteria)
        corners_2 = cv2.cornerSubPix(gray_2, corners_2, (11, 11), (-1, -1), criteria)

        # Add the refined calibration points to the image points arrays
        image_points_1.append(corners_1.reshape(-1, 2))
        image_points_2.append(corners_2.reshape(-1, 2))

        # Draw the detected chessboard corners onto the images
        cv2.drawChessboardCorners(image_1, (10, 7), corners_1, ret_1)
        cv2.drawChessboardCorners(image_2, (10, 7), corners_2, ret_2)
        print('Processing: {:.2f} %'.format(image_idx / num_images * 100), end='\r')

        #Display the images with the detected corners and points
        # cv2.imshow(f'Camera 1 Image {image_idx}', image_1)
        # cv2.imshow(f'Camera 2 Image {image_idx}', image_2)
        # cv2.waitKey(0)

# Calibrate the cameras and obtain the intrinsic parameters
ret_1, camera_matrix_1, distortion_coeffs_1, rvecs_1, tvecs_1 = cv2.calibrateCamera(calibration_points, image_points_1, gray_1.shape[::-1], None, None)
ret_2, camera_matrix_2, distortion_coeffs_2, rvecs_2, tvecs_2 = cv2.calibrateCamera(calibration_points, image_points_2, gray_2.shape[::-1], None, None)
# print(camera_matrix_1)
# print(distortion_coeffs_1)
# print(camera_matrix_2)
# print(distortion_coeffs_2)
# Set the initial values of the extrinsic parameters
R = np.array([[1, 0, 0],
              [0, 0.5, -0.5],
              [0, 0.5, 0.5]])

T = np.array([[-12.5],
              [-670],
              [0]])

# Apply stereo calibration
flags = cv2.CALIB_FIX_INTRINSIC
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1000000, 1e-100)
ret, camera_matrix_1, distortion_coeffs_1, camera_matrix_2, distortion_coeffs_2, R, T, E, F = cv2.stereoCalibrate(
    calibration_points, image_points_1, image_points_2, camera_matrix_1, distortion_coeffs_1, camera_matrix_2, distortion_coeffs_2, gray_1.shape[::-1],
    criteria=criteria, flags=flags, R=R, T=T)


# Print the extrinsic parameters
def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    # Convert to degrees
    roll, pitch, yaw = np.rad2deg([x, y, z])
    print('roll, pitch, yaw',roll, pitch, yaw)
    return roll, pitch, yaw

def euler_angles_to_rotation_matrix(roll, pitch, yaw):
    roll = np.deg2rad(roll)
    pitch = np.deg2rad(pitch)
    yaw = np.deg2rad(yaw)

    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R

def plot_camera_origins(rotation_cam1, position_cam1, rotation_cam2, position_cam2):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    def plot_camera(ax, rotation, position, color):
        arrow_length = 500

        origin = np.array(position).flatten()
        x_axis = rotation[:, 0] * arrow_length + origin
        y_axis = rotation[:, 1] * arrow_length + origin
        z_axis = rotation[:, 2] * arrow_length + origin

        ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], color=[1,0,0], label="X-axis")
        ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], color=[0,1,0], linestyle="--", label="Y-axis")
        ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], color=[0,0,1], linestyle=":", label="Z-axis")

    plot_camera(ax, rotation_cam1, position_cam1, 'r')
    plot_camera(ax, rotation_cam2, position_cam2, 'b')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.legend()
    plt.show()



rotation_matrix_to_euler_angles(np.array(R))
dist = np.sqrt(np.power(T[0],2)+np.power(T[1],2)+np.power(T[2],2))
print('Distance between cameras: ',dist)
print('Arc between cameras: ',((np.arccos(R[1,1])*57.2957795 + np.arcsin(R[1,2])*57.2957795))/2)


# Calculate the midpoint between the cameras
midpoint = T / 2

# Calculate the position of each camera relative to the origin
position_cam1 = -midpoint
position_cam2 = midpoint

# Calculate the rotation of each camera relative to the origin
rotation_cam1 = np.eye(3)  # Identity matrix (3x3)
rotation_cam2 = R

# Print the rotation and translation of each camera with respect to the origin
print("Camera 1:")
print("Rotation:")
print(rotation_cam1)
rotation_matrix_to_euler_angles(rotation_cam1)
print("Translation:")
print(position_cam1)

print("Camera 2:")
print("Rotation:")
print(rotation_cam2)
print("Translation:")
print(position_cam2)
roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_cam2)
R1 = euler_angles_to_rotation_matrix(-roll/2, pitch/2, yaw/2)
R2 = euler_angles_to_rotation_matrix(roll/2, pitch/2, yaw/2)
plot_camera_origins(R1, [0,dist,0], R2, [0,-dist,0])


