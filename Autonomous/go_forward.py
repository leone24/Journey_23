# -*- coding: utf-8 -*-
from pymavlink import mavutil

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

master.arducopter_arm()

f_speed = 1

lat, lon, alt = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).lat / 1e7, master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).lon / 1e7, master.recv_match(type='GLOBAL_POSITION_INT', blocking=True).alt / 1000.0

go_forward  = mavutil.mavlink.MAVLink_command_long_message(
    0,   # target system ID (0 for broadcast)
    0,   # target component ID (0 for broadcast)
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0,
    f_speed,#param1
    0,
    0,
    0,
    lat,
    lon,
    alt
)
master.mav.send(go_forward)
