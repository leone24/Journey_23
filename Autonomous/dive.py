from pymavlink import mavutil

master = mavutil.mavlink_connection('udpin:localhost:14551')

master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))
#arm the vehicle
master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, -20)

msg = master.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)