import math
from pymavlink import mavutil

def get_mavlink_data(master, telemetry):
    
    msg = master.recv_match(type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'SYS_STATUS', 'GPS_RAW_INT', 'VFR_HUD'], blocking=False)
    
    if msg:
        msg_type = msg.get_type()
        
        if msg_type == 'ATTITUDE':
            telemetry['roll'] = math.degrees(msg.roll)
            telemetry['pitch'] = math.degrees(msg.pitch)
            telemetry['yaw'] = math.degrees(msg.yaw)

        if msg_type == 'GLOBAL_POSITION_INT':
            telemetry['lat'] = msg.lat / 1e7          # Convert from int to degrees
            telemetry['lon'] = msg.lon / 1e7          # Convert from int to degrees
            telemetry['alt'] = msg.relative_alt / 1000.0 # Convert mm to meters
            telemetry['heading'] = msg.hdg / 100.0    # Convert from cdeg to deg

        if msg_type == 'VFR_HUD':
            telemetry['ground_speed'] = msg.groundspeed
            telemetry['climb_rate'] = msg.climb     # Vertical speed (m/s)
            telemetry['airspeed'] = msg.airspeed
            telemetry['throttle'] = msg.throttle

        if msg_type == 'GPS_RAW_INT':
            telemetry['satellites'] = msg.satellites_visible
            telemetry['fix_type'] = msg.fix_type
            telemetry['h_acc'] = msg.eph / 100.0  # Horizontal accuracy in meters
            telemetry['v_acc'] = msg.epv / 100.0  # Vertical accuracy in meters

        if msg_type == 'SYS_STATUS':
            telemetry['battery'] = msg.voltage_battery / 1000.0 # mV to Volts
            telemetry['current_battery'] = msg.current_battery / 100.0 # cA to A
            telemetry['battery_remaining'] = msg.battery_remaining # Percentage

        if msg_type == 'HIGHRES_IMU':
            telemetry['accel_x'] = msg.xacc / 1000.0  # mg to g
            telemetry['accel_y'] = msg.yacc / 1000.0
            telemetry['accel_z'] = msg.zacc / 1000.0
            telemetry['gyro_x'] = msg.xgyro / 1000.0  # mdps to dps
            telemetry['gyro_y'] = msg.ygyro / 1000.0
            telemetry['gyro_z'] = msg.zgyro / 1000.0

        if msg_type == 'RC_CHANNELS':
            telemetry['rc_channels'] = msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw

    return telemetry