from pymavlink import mavutil
import time
import numpy as np
import cv2

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

cap = cv2.VideoCapture(0)

red_found = False


def arm():
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Vehicle armed!')


def disarm():
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)
    print("Waiting for the vehicle to disarm")
    master.motors_disarmed_wait()
    print('Vehicle disarmed!')


def go_forward(time_limit=5):
    start_time = time.time()
    
    while True:
        if time.time() - start_time > time_limit:
            print('Time limit reached.')
            break
        
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            int(0b110111111000), 20, 0, -20, 4, 0, 0, 0, 0, 0, 0, 0))
        
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        print(msg)


def turn_right():
    master.mav.command_long_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 90, 25, 1, 1, 0, 0, 0)

    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    time.sleep(1)


def turn_left():
    master.mav.command_long_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 90, 25, -1, 1, 0, 0, 0)

    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    time.sleep(0.1)


def full():
    master.mav.command_long_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,90, 15, 1, 1, 0, 0, 0)

    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    time.sleep(1)


def image_prossesing():
    while True:
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

        # wait for key press
        key = cv2.waitKey(1)

        # exit if the 'q' key is pressed
        if red_found == True:
            break


def searching():
    for i in range(4):
        full()
        time.sleep(0.1)
        #image_prossesing()


def positioning():
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



def main():
    arm()
    searching()
    st = time.time()
    ft = time.time()
    t = 0   
    x = 20 # random for now
    
    if x > t:
        st = time.time()
        go_forward()
        time.sleep(0.1)
        searching()
        time.sleep(0.1)
        if red_found == True:
            positioning()

        elif red_found == False:
            ft = time.time()
            t = ft - st

    while red_found == False:

        if x > t:
            go_forward()
            time.sleep(0.1)
            searching()
            time.sleep(0.1)

            if red_found == True:
                positioning()
                

            elif red_found == False:
                ft = time.time()
                t = ft - st
                continue

        elif t == x or t > x:
                    
                    turn_right()
                    st = time.time()
                    time.sleep(0.1)
                    go_forward()
                    time.sleep(0.1)
                    searching()
                    time.sleep(0.1)

                    if red_found == True:
                        positioning()
                        break

                    if red_found == False:
                        ft = time.time()
                        t = ft - st

        while red_found == False:

            if x/4 > t:
                go_forward()
                time.sleep(0.1)
                searching()
                time.sleep(0.1)

                if red_found == True:
                    positioning()
                    break

                if red_found == False:
                    ft = time.time()
                    t = ft - st
                    continue

            elif x/4 == t or t > x/4:
                turn_right()
                st = time.time()
                time.sleep(0.1)
                go_forward()
                time.sleep(0.1)
                searching()

                if red_found == True:
                    positioning()
                    break

                elif red_found == False:
                    ft = time.time()
                    t = ft - st

            while red_found == False:

                if x > t:
                    go_forward()
                    time.sleep(0.1)
                    searching()
                    time.sleep(0.1)

                    if red_found == True:
                        positioning()
                        break

                    if red_found == False:
                        ft = time.time()
                        t = ft - st
                        continue

                elif x == t or t > x:

                    turn_left()
                    st = time.time()
                    time.sleep(0.1)
                    go_forward()
                    time.sleep(0.1)
                    searching()
                    time.sleep(0.1)

                    if red_found == True:
                        positioning()

                    if red_found == False:
                        ft = time.time()
                        t = ft - st

                while red_found == False:

                        if x/4 > t:
                            go_forward()
                            time.sleep(1)
                            searching()
                            time.sleep(1)

                            if red_found == True:
                                positioning()
                            
                        
                            elif red_found == False:
                                ft = time.time()
                                t = ft - st
                                continue

                        elif x/4 == t or t > x/4:

                            turn_left()
                            st = time.time()
                            time.sleep(0.1)
                            go_forward()
                            time.sleep(0.1)
                            searching()
                            time.sleep(0.1)

                            if red_found == True:
                                positioning()

                            elif red_found == False:
                                ft = time.time()
                                t = ft - st

                        while red_found == False:

                            if x > t:
                                go_forward()
                                time.sleep(0.1)
                                searching()
                                time.sleep(0.1)

                                if red_found == True:
                                    positioning()
                         

                                elif red_found == False:
                                    ft = time.time()
                                    t = ft - st
                                    continue

                            elif t == x or t > x:
                                turn_left()
                                st = time.time()
                                go_forward()
                                time.sleep(0.1)
                                searching()
                                time.sleep(0.1)

                                if red_found == True:
                                    positioning()

                                elif red_found == False:
                                    print(" Mission Unsuccesful! ")
                                    break


cap.release()
cv2.destroyAllWindows()

if __name__ == "__main__":
    main()



    
