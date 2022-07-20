from pymavlink import mavutil
import time
# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('127.0.0.1:14550')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

mode = 'GUIDED'
mode_id = the_connection.mode_mapping()[mode]
the_connection.mav.set_mode_send(
    the_connection.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

konum_lat = the_connection.location().lat
konum_lng = the_connection.location().lng
konum_alt = the_connection.location().alt

while True:
    son_konum_alt = the_connection.location().alt
    if (son_konum_alt-konum_alt)<11 and 9<(son_konum_alt-konum_alt):
        break

print("Gereken yukseklige ulasildi")

the_connection.set_mode_rtl()
