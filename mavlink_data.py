import time
import math
from pymavlink import mavutil

print("Connecting to mavlink-router...")
master = mavutil.mavlink_connection('udpin:localhost:14550')

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected!")

# Request all data streams (Standard way to ask for everything)
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1
)

# 1. Define the dictionary to hold all your data
telemetry = {
    'roll': 0, 'pitch': 0, 'yaw': 0,
    'lat': 0, 'lon': 0, 'alt': 0, 'heading': 0,
    'ground_speed': 0, 'climb_rate': 0, 
    'battery': 0,  'current_battery': 0, 'battery_remaining': 0,
    'heading': 0, 'airspeed': 0, 'throttle': 0,                        
    'satellites': 0, 'fix_type': 0, 'h_acc': 0, 'v_acc': 0,
    'accel_x': 0, 'accel_y': 0, 'accel_z': 0,
    'gyro_x': 0, 'gyro_y': 0, 'gyro_z': 0,
    'rc_channels': (0, 0, 0, 0, 0, 0, 0, 0)     
}

def get_mavlink_data():
    global telemetry
    
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

last_print = 0

while True:
    data = get_mavlink_data()
    now = time.time()
    if now - last_print >= 0.1:
        print(f" {data['roll']:.1f} {data['pitch']:.1f} {data['yaw']:.1f} | GPS: {data['lat']:.6f}, {data['lon']:.6f}, Alt: {data['alt']:.1f}m | Battery: {data['battery_remaining']}% ")
        last_print = now