from pymavlink import mavutil

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

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

disarm()