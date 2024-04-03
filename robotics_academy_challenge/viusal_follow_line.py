from GUI import GUI
from HAL import HAL
import cv2
import numpy as np

# PID controller parameters
KP = -1 / 500
KD = -1 / 200

# Initial error values
error_current = 0.0
error_previous = 0.0

# Color ranges for red in HSV
LOWER_RED1 = np.array([0, 50, 50])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([170, 50, 50])
UPPER_RED2 = np.array([180, 255, 255])

def filter_color_in_range(image, lower_bound, upper_bound):
    """Filter a specified color range in an image."""
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    return mask

def calculate_centroid(mask):
    """Calculate the centroid of a given mask area."""
    moments = cv2.moments(mask)
    if moments['m00'] != 0:
        x = int(moments['m10'] / moments['m00'])
        y = int(moments['m01'] / moments['m00'])
        return x, y
    return None

def calculate_pid_error(set_point, current_point):
    """Compute the PID error."""
    global error_previous
    error = current_point - set_point
    p_error = KP * error
    d_error = KD * (error - error_previous)
    error_previous = error
    return p_error + d_error


while True:
    """Main loop to process image and apply PID control."""

    frame = HAL.getImage()

    # Combine masks for both red color ranges
    red_mask = filter_color_in_range(frame, LOWER_RED1, UPPER_RED1) + \
        filter_color_in_range(frame, LOWER_RED2, UPPER_RED2)

    highlighted_frame = cv2.bitwise_and(frame, frame, mask=red_mask)
    centroid = calculate_centroid(red_mask)

    if centroid:
        x_centroid, _ = centroid
        cv2.circle(highlighted_frame, (x_centroid, _), 20, (0, 255, 0), 5)
        frame_width = frame.shape[1]
        pid_error = calculate_pid_error(frame_width / 2, x_centroid)

        velocity = 3
        HAL.setV(velocity)
        HAL.setW(pid_error)

        GUI.showImage(highlighted_frame)
