from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

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

go_forward()