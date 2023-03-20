from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

#arm the vehicle 
master.mav.command_long_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

def turn_left():
    master.mav.command_long_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 90, 25, -1, 1, 0, 0, 0)

    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    time.sleep(0.1)

turn_left()