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

    # check if any red pixel is present in the mask
    if cv2.countNonZero(mask) > 0 and not red_found:
        print("We found color red")
        red_found = True

    # display the resulting image
    cv2.imshow('Frame', res)

    if red_found == True:
        red_pixels = cv2.countNonZero(mask)
        total_pixels = mask.shape[0] * mask.shape[1]
        red_percent = red_pixels * 100 / total_pixels

        while red_percent < 70 :
            #find the center of the object
            #go forward 
            if red_percent >= 70 :
                print("70 percent is red now")
                master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, -2)

                msg = master.recv_match(type='COMMAND_ACK', blocking=True)
                print(msg)
                break
            
           

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
if len(contours) > 0:
'''