#algoritma
'''
red degeri true olduğunda+
while kırmızı pixel 70 den kucuk
objenin merkezini bul
merkeze doğru ilerle
if kırmızı pixel 70 den buyukse
alçal- takeoff
bitir

'''




'''
on saniyede bir sonucu soyleme

import cv2
import numpy as np
import time

# create a video capture object to read from the camera
cap = cv2.VideoCapture(0)

# flag to indicate whether or not red has been found
red_found = False

# start time for measuring the 10-second interval
start_time = time.time()

while True:


    # calculate the percentage of red pixels in the mask
    num_pixels = mask.shape[0] * mask.shape[1]
    num_red_pixels = cv2.countNonZero(mask)
    red_percentage = num_red_pixels / num_pixels * 100

    # check if any red pixel is present in the mask
    if red_percentage >= 70 and not red_found:
        print("70 percent is red now")
        red_found = True

    # check if 10 seconds have passed since the last print
    if time.time() - start_time >= 10:
        print("Current percentage of red: {:.2f}%".format(red_percentage))
        start_time = time.time()

    # display the resulting image
    cv2.imshow('Frame', res)

    # wait for key press
    key = cv2.waitKey(1)

    # exit if the 'q' key is pressed
    if key & 0xFF == ord('q'):
        break

# release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
'''

import cv2
import numpy as np

# create a video capture object to read from the camera
cap = cv2.VideoCapture(0)

# flag to indicate whether or not red has been found
red_found = False

while True:
    # read a frame from the camera
    ret, frame = cap.read()

    # convert the color space from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define a range of red color in HSV color space
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    # combine the masks to get the final mask
    mask = cv2.bitwise_or(mask1, mask2)

    # bitwise AND the mask with the frame to show the red areas only
    res = cv2.bitwise_and(frame, frame, mask=mask)


    red_pixels = cv2.countNonZero(mask)
    total_pixels = mask.shape[0] * mask.shape[1]
    red_percent = red_pixels * 100 / total_pixels

    # check if any red pixel is present in the mask
    if red_percent >= 70 and not red_found:
        print("70 percent is red now")
        red_found = True

    # display the resulting image
    cv2.imshow('Frame', res)

    # wait for key press
    key = cv2.waitKey(1)

    # exit if the 'q' key is pressed
    if key & 0xFF == ord('q'):
        break

# release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()