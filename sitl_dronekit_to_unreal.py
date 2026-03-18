# Fix for the error: AttributeError: module 'collections' has no attribute 'MutableMapping'
from collections import abc
import collections
collections.MutableMapping = abc.MutableMapping

import time
import math
import json
import threading

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil as _mavutil
import tcp_relay

from flask import Flask, request, jsonify, Response, stream_with_context

# ── Configuration ─────────────────────────────────────────────────────────────
PWM_MIN = 1000
PWM_MAX = 2000
VEHICLE_CONNECTION_STRING = 'udpin:127.0.0.1:14540'
VEHICLE_BAUD = 57600
FLASK_HOST = '0.0.0.0'
FLASK_PORT = 5001
TCP_RELAY_HOST = '0.0.0.0'
TCP_RELAY_PORT = 1234
GUIDED_GOTO_DEFAULT_ALT_M = 20.0
GUIDED_GOTO_MIN_ALT_M = 1.0
GUIDED_GOTO_MAX_ALT_M = 500.0
GUIDED_MODE_FIXED_WING = 'fixed_wing'
GUIDED_MODE_VTOL = 'vtol'
GUIDED_MODE_DEFAULT = GUIDED_MODE_VTOL
RETURN_HOME_COMMAND_ALIASES = {'return_to_home', 'return_home', 'rth'}
RETURN_HOME_DEFAULT_ALT_M = GUIDED_GOTO_DEFAULT_ALT_M
RETURN_HOME_REQUIRE_VALID_HOME = True
LOG_RC_OVERRIDE_COMMAND_ONCE = True

# ── Shared States ─────────────────────────────────────────────────────────────
motor_state = {
    "vtol": [0.0, 0.0, 0.0, 0.0],
    "forward": 0.0,
    "is_vtol": 0.0,
    "is_fw": 0.0,
}

gimbal_state = {
    "pitch": 0.0,
    "yaw": 0.0,
}

GIMBAL_RC_CHANNEL_PITCH = 6
GIMBAL_RC_CHANNEL_YAW = 7
GIMBAL_SERVO_CHANNEL_PITCH = 9
GIMBAL_SERVO_CHANNEL_YAW = 10

live_telemetry = {
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "lat": 0.0, "lon": 0.0, "alt": 0.0,
    "heading": 0, "airspeed": 0.0,
    "armed": False, "mode": "UNKNOWN",
    "home_lat": None, "home_lon": None, "home_alt": None,
    "home_valid": False,
    "gimbal_pitch": 0.0,
    "gimbal_yaw": 0.0,
    "motors": motor_state
}

telemetry_lock = threading.Lock()
message_history = []
command_log_lock = threading.Lock()
rc_override_command_logged = False

# ── Helper Functions ──────────────────────────────────────────────────────────
def pwm_to_normalized(pwm):
    if pwm is None or pwm < 1050:
        return 0.0
    return max(0.0, min(1.0, (pwm - PWM_MIN) / (PWM_MAX - PWM_MIN)))

def _vehicle_mode_name():
    try: return vehicle.mode.name
    except: return "UNKNOWN"


def _servo_pwm_from_msg(msg, channel):
    return getattr(msg, f'servo{channel}_raw', 0)


def _normalize_guided_mode_name(guided_mode):
    normalized = str(guided_mode or '').strip().lower()
    if normalized in {GUIDED_MODE_FIXED_WING, GUIDED_MODE_VTOL}:
        return normalized
    return GUIDED_MODE_DEFAULT


def _get_home_location_snapshot():
    try:
        home = vehicle.home_location
    except Exception:
        home = None

    if home is None:
        return None

    lat = getattr(home, 'lat', None)
    lon = getattr(home, 'lon', None)
    alt = getattr(home, 'alt', None)

    if lat is None or lon is None:
        return None

    try:
        lat = float(lat)
        lon = float(lon)
    except Exception:
        return None

    if not (math.isfinite(lat) and math.isfinite(lon)):
        return None

    try:
        alt_val = float(alt) if alt is not None else None
    except Exception:
        alt_val = None

    if alt_val is not None and not math.isfinite(alt_val):
        alt_val = None

    return {
        'lat': lat,
        'lon': lon,
        'alt': alt_val,
    }


def _update_home_telemetry_locked():
    home = _get_home_location_snapshot()
    if home is None:
        live_telemetry['home_lat'] = None
        live_telemetry['home_lon'] = None
        live_telemetry['home_alt'] = None
        live_telemetry['home_valid'] = False
        return

    live_telemetry['home_lat'] = round(home['lat'], 7)
    live_telemetry['home_lon'] = round(home['lon'], 7)
    live_telemetry['home_alt'] = round(home['alt'], 2) if home['alt'] is not None else None
    live_telemetry['home_valid'] = True


def _send_return_to_home(guided_mode=None):
    home = _get_home_location_snapshot()
    if home is None and RETURN_HOME_REQUIRE_VALID_HOME:
        return False, 'Home location unavailable from SITL'

    if home is None:
        return False, 'Home location unavailable'

    lat = home['lat']
    lon = home['lon']
    home_alt = home['alt']

    current_alt = None
    try:
        current_alt = float(live_telemetry.get('alt'))
    except Exception:
        current_alt = None

    target_alt = home_alt
    if target_alt is None:
        target_alt = current_alt if current_alt is not None else RETURN_HOME_DEFAULT_ALT_M
    target_alt = max(GUIDED_GOTO_MIN_ALT_M, min(GUIDED_GOTO_MAX_ALT_M, float(target_alt)))

    guided_mode_name = _normalize_guided_mode_name(guided_mode)

    try:
        vehicle.mode = VehicleMode('GUIDED')
    except Exception:
        pass

    try:
        if guided_mode_name == GUIDED_MODE_FIXED_WING:
            master = _mavlink_master()
            master.mav.command_int_send(
                master.target_system,
                master.target_component,
                _mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                _mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                0,
                0,
                -1,
                0,
                0,
                math.nan,
                int(lat * 1e7),
                int(lon * 1e7),
                target_alt,
            )
        else:
            target = LocationGlobalRelative(lat, lon, target_alt)
            vehicle.simple_goto(target)
        print(
            '[Bridge] Return-to-home target sent '
            f'lat={lat:.7f}, lon={lon:.7f}, alt={target_alt:.1f}m, mode={guided_mode_name}'
        )
        return True, f'Returning to home ({guided_mode_name})'
    except Exception as e:
        return False, f'Failed to send return-home command: {e}'

# ── Connect to Vehicle & TCP ──────────────────────────────────────────────────
print("Connecting to Vehicle...")
vehicle = connect(VEHICLE_CONNECTION_STRING, wait_ready=True, baud=VEHICLE_BAUD)
print("Vehicle Connected!")

relay = tcp_relay.TCP_Relay(host=TCP_RELAY_HOST, port=TCP_RELAY_PORT)

# ── MAVLink Listeners ─────────────────────────────────────────────────────────

@vehicle.on_message('ATTITUDE')
def attitude_listener(self, name, msg):
    with telemetry_lock:
        live_telemetry["roll"] = round(math.degrees(msg.roll), 2)
        live_telemetry["pitch"] = round(math.degrees(msg.pitch), 2)
        live_telemetry["yaw"] = round(math.degrees(msg.yaw), 2)

@vehicle.on_message('VFR_HUD')
def vfr_hud_listener(self, name, msg):
    with telemetry_lock:
        live_telemetry["airspeed"] = msg.airspeed
        live_telemetry["heading"] = msg.heading
        live_telemetry["alt"] = msg.alt

@vehicle.on_message('GLOBAL_POSITION_INT')
def gps_listener(self, name, msg):
    with telemetry_lock:
        live_telemetry["lat"] = msg.lat / 1e7
        live_telemetry["lon"] = msg.lon / 1e7

@vehicle.on_message('HEARTBEAT')
def heartbeat_listener(self, name, msg):
    with telemetry_lock:
        live_telemetry["armed"] = vehicle.armed
        live_telemetry["mode"] = _vehicle_mode_name()
        _update_home_telemetry_locked()

@vehicle.on_message('SERVO_OUTPUT_RAW')
def servo_listener(self, name, msg):
    mode = _vehicle_mode_name()
    
    # Motors
    motor_state["vtol"][0] = pwm_to_normalized(msg.servo5_raw)
    motor_state["vtol"][1] = pwm_to_normalized(msg.servo6_raw)
    motor_state["vtol"][2] = pwm_to_normalized(msg.servo7_raw)
    motor_state["vtol"][3] = pwm_to_normalized(msg.servo8_raw)
    motor_state["forward"] = pwm_to_normalized(msg.servo3_raw)
    
    # Flight State
    if mode.startswith('Q'):
        motor_state["is_vtol"], motor_state["is_fw"] = 1.0, 0.0
    elif mode in ['FBWA', 'FBWB', 'CRUISE', 'AUTO']:
        motor_state["is_vtol"], motor_state["is_fw"] = 0.0, 1.0
    else:
        motor_state["is_vtol"], motor_state["is_fw"] = 1.0, 1.0

    # Gimbal
    pitch_pwm = _servo_pwm_from_msg(msg, GIMBAL_SERVO_CHANNEL_PITCH)
    if pitch_pwm > 0:
        gimbal_state["pitch"] = round(((pitch_pwm - 1500) / 500.0) * 90.0, 2)
    yaw_pwm = _servo_pwm_from_msg(msg, GIMBAL_SERVO_CHANNEL_YAW)
    if yaw_pwm > 0:
        gimbal_state["yaw"] = round(((yaw_pwm - 1500) / 500.0) * 180.0, 2)
        
    with telemetry_lock:
        live_telemetry["gimbal_pitch"] = gimbal_state["pitch"]
        live_telemetry["gimbal_yaw"] = gimbal_state["yaw"]

# ── Low-level MAVLink helpers (use vehicle._master to avoid DroneKit wrapping) ──

def _mavlink_master():
    """Return the raw pymavlink connection from DroneKit."""
    return vehicle._master


# All recognised ArduPilot flight mode names (lowercase → canonical)
_MODE_ALIASES = {
    'rtl': 'RTL', 'land': 'LAND', 'loiter': 'LOITER', 'guided': 'GUIDED',
    'auto': 'AUTO', 'stabilize': 'STABILIZE', 'stabilise': 'STABILIZE',
    'alt_hold': 'ALT_HOLD', 'althold': 'ALT_HOLD', 'acro': 'ACRO',
    'manual': 'MANUAL', 'fbwa': 'FBWA', 'fbwb': 'FBWB', 'cruise': 'CRUISE',
    'autotune': 'AUTOTUNE', 'poshold': 'POSHOLD', 'brake': 'BRAKE',
    'throw': 'THROW', 'avoid_adsb': 'AVOID_ADSB', 'guided_nogps': 'GUIDED_NOGPS',
    'qstabilize': 'QSTABILIZE', 'qhover': 'QHOVER', 'qloiter': 'QLOITER',
    'qland': 'QLAND', 'qrtl': 'QRTL', 'qautotune': 'QAUTOTUNE',
    'follow': 'FOLLOW', 'zigzag': 'ZIGZAG', 'systemid': 'SYSTEMID',
    'training': 'TRAINING', 'circle': 'CIRCLE', 'drift': 'DRIFT',
    'sport': 'SPORT', 'flip': 'FLIP', 'smartrtl': 'SMARTRTL',
}

PERSISTENT_RC_OVERRIDE_RATE_HZ = 10.0
_persistent_rc_overrides = {}
_persistent_rc_lock = threading.Lock()


def _dispatch_terminal_command(text):
    """Parse ArduPilot terminal text and dispatch via DroneKit/pymavlink.

    Supports every command you can type in the SITL terminal:
      mode <NAME>  |  <MODE_NAME>          - set flight mode
      arm throttle  |  arm                  - arm motors
      disarm                                - disarm motors
      rc <N> <PWM>                          - RC channel override (PWM 0 = release)
      param set <NAME> <VALUE>              - set a parameter
      param show <NAME>                     - print a parameter value
      param fetch                           - request all params from vehicle
      takeoff <alt>                         - MAV_CMD_NAV_TAKEOFF
      do_set_home                           - set home to current location
      wp list | wp clear | wp load <file>   - waypoint commands via SERIAL_CONTROL
      Everything else                       - sent via SERIAL_CONTROL shell
    """
    master = _mavlink_master()
    raw = text.strip()
    lower = raw.lower()
    parts = raw.split()
    if not parts:
        return 'error', 'Empty command'

    verb = parts[0].lower()

    # ── mode <NAME>  or bare mode name ──────────────────────────────────────
    if verb == 'mode' and len(parts) >= 2:
        mode_name = _MODE_ALIASES.get(parts[1].lower(), parts[1].upper())
        try:
            vehicle.mode = VehicleMode(mode_name)
            print(f'[Bridge] Mode set: {mode_name}')
            return 'ok', f'Mode set to {mode_name}'
        except Exception as e:
            return 'error', str(e)

    if verb in _MODE_ALIASES:
        mode_name = _MODE_ALIASES[verb]
        try:
            vehicle.mode = VehicleMode(mode_name)
            print(f'[Bridge] Mode set (bare): {mode_name}')
            return 'ok', f'Mode set to {mode_name}'
        except Exception as e:
            return 'error', str(e)

    # ── arm ──────────────────────────────────────────────────────────────────
    if verb == 'arm':
        try:
            vehicle.armed = True
            print('[Bridge] Arm command sent')
            return 'ok', 'Arm command sent'
        except Exception as e:
            return 'error', str(e)

    # ── disarm ───────────────────────────────────────────────────────────────
    if verb == 'disarm':
        try:
            vehicle.armed = False
            print('[Bridge] Disarm command sent')
            return 'ok', 'Disarm command sent'
        except Exception as e:
            return 'error', str(e)

    # ── rc <channel> <pwm> ───────────────────────────────────────────────────
    if verb == 'rc' and len(parts) >= 3:
        try:
            ch_num = int(parts[1])
            pwm    = int(parts[2])
            _set_persistent_rc_override(ch_num, pwm)
            print(f'[Bridge] RC override channel {ch_num} → {pwm}')
            return 'ok', f'RC channel {ch_num} override {pwm}'
        except Exception as e:
            return 'error', str(e)

    # ── param set <NAME> <VALUE> ─────────────────────────────────────────────
    if verb == 'param' and len(parts) >= 2:
        sub = parts[1].lower()
        if sub == 'set' and len(parts) >= 4:
            try:
                param_name  = parts[2].upper()
                param_value = float(parts[3])
                vehicle.parameters[param_name] = param_value
                print(f'[Bridge] Param set {param_name}={param_value}')
                return 'ok', f'{param_name} = {param_value}'
            except Exception as e:
                return 'error', str(e)
        if sub == 'show' and len(parts) >= 3:
            try:
                param_name = parts[2].upper()
                val = vehicle.parameters.get(param_name)
                result = f'{param_name} = {val}'
                print(f'[Bridge] {result}')
                return 'ok', result
            except Exception as e:
                return 'error', str(e)
        if sub == 'fetch':
            try:
                master.mav.param_request_list_send(
                    master.target_system, master.target_component)
                return 'ok', 'Param fetch requested'
            except Exception as e:
                return 'error', str(e)

    # ── takeoff <alt> ────────────────────────────────────────────────────────
    if verb == 'takeoff' and len(parts) >= 2:
        try:
            alt = float(parts[1])
            master.mav.command_long_send(
                master.target_system, master.target_component,
                _mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0, 0, 0, 0, 0, 0, 0, alt,
            )
            print(f'[Bridge] Takeoff to {alt}m')
            return 'ok', f'Takeoff {alt}m'
        except Exception as e:
            return 'error', str(e)

    # ── do_set_home ──────────────────────────────────────────────────────────
    if verb == 'do_set_home':
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                _mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0, 1, 0, 0, 0, 0, 0, 0,
            )
            print('[Bridge] Set home to current location')
            return 'ok', 'Set home'
        except Exception as e:
            return 'error', str(e)

    # ── Fallback: SERIAL_CONTROL shell relay ─────────────────────────────────
    # Used for wp, fence, rally, scripting, and any other command not above.
    try:
        if not raw.endswith('\n'):
            raw += '\n'
        encoded = raw.encode('utf-8')
        MAX_CHUNK = 70
        for i in range(0, len(encoded), MAX_CHUNK):
            chunk   = encoded[i:i + MAX_CHUNK]
            padded  = list(chunk) + [0] * (MAX_CHUNK - len(chunk))
            master.mav.serial_control_send(
                device=_mavutil.mavlink.SERIAL_CONTROL_DEV_SHELL,
                flags=(_mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND |
                       _mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE),
                timeout=0,
                baudrate=0,
                count=len(chunk),
                data=padded,
            )
            if len(encoded) > MAX_CHUNK:
                time.sleep(0.05)
        print(f'[Bridge] SERIAL_CONTROL shell: {raw.strip()!r}')
        return 'ok', f'Shell command sent: {raw.strip()}'
    except Exception as e:
        return 'error', f'SERIAL_CONTROL failed: {e}'


def _send_do_set_servo(servo_num, pwm):
    """Send MAV_CMD_DO_SET_SERVO (cmd 183) to set a servo output permanently.

    Directly writes the PWM value to the servo output register.
    ArduPilot holds the value until it receives a new command or reboots –
    no heartbeat or continuous refresh required.
    servo_num: 1-indexed servo/channel number.
    pwm:       Output pulse width in microseconds (1000–2000).
    """
    master = _mavlink_master()
    pwm = max(1000, min(2000, int(pwm)))
    master.mav.command_long_send(
        master.target_system, master.target_component,
        _mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        float(servo_num),  # param1 – servo instance/channel number
        float(pwm),        # param2 – PWM (µs)
        0, 0, 0, 0, 0,
    )
    print(f'[Bridge] DO_SET_SERVO: servo{servo_num}={pwm}µs')


def _send_rc_override_direct(channels_dict):
    """Send RC_CHANNELS_OVERRIDE directly via pymavlink.

    channels_dict maps channel number (str or int) to PWM value.
    PWM 0 or None means 'release / passthrough' for that channel (MAVLink standard).
    Channels not listed are left at 0 (passthrough).
    """
    master = _mavlink_master()
    ch = [0] * 18  # MAVLink v2 supports 18 channels; 0 = no override
    for key, pwm in channels_dict.items():
        idx = int(key) - 1
        if 0 <= idx < 18:
            if pwm is not None and int(pwm) != 0:
                ch[idx] = int(max(1000, min(2000, int(pwm))))
    # rc_channels_override_send signature differs by pymavlink version;
    # use keyword-safe positional call with 18 channels (MAVLink 2).
    try:
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7],
            ch[8], ch[9], ch[10], ch[11], ch[12], ch[13], ch[14], ch[15],
            ch[16], ch[17],
        )
    except TypeError:
        # Older pymavlink only has 8-channel variant
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7],
        )


def _set_persistent_rc_override(ch_num, pwm):
    pwm = max(0, min(2000, int(pwm)))
    key = str(ch_num)
    with _persistent_rc_lock:
        if pwm == 0:
            _persistent_rc_overrides.pop(key, None)
        else:
            _persistent_rc_overrides[key] = pwm
        overrides = dict(_persistent_rc_overrides)
    _send_rc_override_direct(overrides)
    _sync_gimbal_state_from_rc(ch_num, pwm)


def _rc_pwm_to_angle(channel, pwm):
    pwm = int(pwm)
    if pwm == 0:
        return 0.0
    if channel == GIMBAL_RC_CHANNEL_PITCH:
        return round(((pwm - 1500) / 500.0) * 90.0, 2)
    if channel == GIMBAL_RC_CHANNEL_YAW:
        return round(((pwm - 1500) / 500.0) * 180.0, 2)
    return 0.0


def _sync_gimbal_state_from_rc(ch_num, pwm):
    if ch_num not in (GIMBAL_RC_CHANNEL_PITCH, GIMBAL_RC_CHANNEL_YAW):
        return
    angle = _rc_pwm_to_angle(ch_num, pwm)
    with telemetry_lock:
        if ch_num == GIMBAL_RC_CHANNEL_PITCH:
            gimbal_state["pitch"] = angle
        elif ch_num == GIMBAL_RC_CHANNEL_YAW:
            gimbal_state["yaw"] = angle


def _rc_override_heartbeat():
    interval = 1.0 / max(1.0, PERSISTENT_RC_OVERRIDE_RATE_HZ)
    while True:
        time.sleep(interval)
        with _persistent_rc_lock:
            overrides = dict(_persistent_rc_overrides)
        if overrides:
            _send_rc_override_direct(overrides)


# ── Flask Web Server & CORS ───────────────────────────────────────────────────
flask_app = Flask(__name__)

# Force CORS Headers so web dashboards can read the data
@flask_app.after_request
def add_cors_headers(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
    return response

@flask_app.route('/status')
def get_status():
    with telemetry_lock:
        _update_home_telemetry_locked()
        return jsonify(live_telemetry)

@flask_app.route('/telemetry/stream')
def telemetry_stream():
    def generate():
        while True:
            with telemetry_lock:
                _update_home_telemetry_locked()
                data = json.dumps(live_telemetry)
            yield f"data: {data}\n\n"
            time.sleep(0.1) # 10Hz dashboard update rate
    return Response(stream_with_context(generate()), mimetype='text/event-stream')

@flask_app.route('/mavlink_messages')
def get_mavlink_messages():
    return jsonify(message_history[-50:]) # Return last 50 messages

@flask_app.route('/params')
def get_params():
    return jsonify({"note": "Params endpoint active"})

@flask_app.route('/command', methods=['POST', 'OPTIONS'])
def send_command():
    global rc_override_command_logged

    if request.method == 'OPTIONS':
        return jsonify({}), 200
    data = request.get_json(silent=True) or {}
    cmd = data.get('command', '')

    # Print incoming commands for routing visibility, but avoid flooding on
    # high-rate rc_override updates.
    if cmd == 'rc_override' and LOG_RC_OVERRIDE_COMMAND_ONCE:
        with command_log_lock:
            if not rc_override_command_logged:
                print(f"[Bridge /command] cmd='rc_override'  payload={json.dumps(data)[:300]} (further rc_override logs suppressed)")
                rc_override_command_logged = True
    else:
        print(f'[Bridge /command] cmd={cmd!r}  payload={json.dumps(data)[:300]}')

    # ── Terminal: parse and dispatch to DroneKit/pymavlink directly ──────────
    # Supports: mode, arm, disarm, rc, param set/show/fetch, takeoff,
    # do_set_home, and anything else via SERIAL_CONTROL shell fallback.
    if cmd == 'terminal':
        try:
            text = str(data.get('text', '')).strip()
            if not text:
                return jsonify({'status': 'error', 'message': 'Empty command'}), 400
            status, message = _dispatch_terminal_command(text)
            return jsonify({'status': status, 'message': message, 'command': text})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500

    # ── RC channel PWM overrides (gimbal / aux channels) ─────────────────────
    if cmd == 'rc_override':
        try:
            channels = data.get('channels', {})
            persistent = bool(data.get('persistent', False))
            if persistent:
                for key, pwm in (channels or {}).items():
                    _set_persistent_rc_override(int(key), int(pwm))
            else:
                _send_rc_override_direct(channels)
            return jsonify({'status': 'ok'})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500

    # ── Arm / disarm ──────────────────────────────────────────────────────────
    if cmd in ('arm', 'disarm'):
        try:
            vehicle.armed = (cmd == 'arm')
            return jsonify({'status': 'ok', 'armed': vehicle.armed})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500

    # ── Flight mode change ────────────────────────────────────────────────────
    if cmd == 'mode':
        try:
            mode_name = str(data.get('mode', '')).strip().upper()
            if not mode_name:
                return jsonify({'status': 'error', 'message': 'Missing mode'}), 400
            if mode_name == 'RTL':
                guided_mode = _normalize_guided_mode_name(data.get('guided_mode'))
                ok, message = _send_return_to_home(guided_mode)
                status_code = 200 if ok else 500
                payload = {
                    'status': 'ok' if ok else 'error',
                    'message': message,
                    'mode': mode_name,
                    'guided_mode': guided_mode,
                }
                if ok:
                    home = _get_home_location_snapshot()
                    if home is not None:
                        payload['home'] = home
                return jsonify(payload), status_code
            vehicle.mode = VehicleMode(mode_name)
            return jsonify({'status': 'ok', 'mode': mode_name})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500

    # ── Return to home (home location from SITL) ────────────────────────────
    if cmd in RETURN_HOME_COMMAND_ALIASES:
        guided_mode = _normalize_guided_mode_name(data.get('guided_mode'))
        ok, message = _send_return_to_home(guided_mode)
        status_code = 200 if ok else 500
        payload = {
            'status': 'ok' if ok else 'error',
            'message': message,
            'guided_mode': guided_mode,
        }
        if ok:
            home = _get_home_location_snapshot()
            if home is not None:
                payload['home'] = home
        return jsonify(payload), status_code

    # ── Guided goto ───────────────────────────────────────────────────────────
    if cmd == 'guided_goto':
        try:
            lat = float(data.get('lat', 0))
            lon = float(data.get('lon', 0))
            alt = float(data.get('altitude') or data.get('alt') or GUIDED_GOTO_DEFAULT_ALT_M)
            alt = max(GUIDED_GOTO_MIN_ALT_M, min(GUIDED_GOTO_MAX_ALT_M, alt))
            guided_mode = _normalize_guided_mode_name(data.get('guided_mode'))
            vehicle.mode = VehicleMode('GUIDED')

            if guided_mode == GUIDED_MODE_FIXED_WING:
                master = _mavlink_master()
                master.mav.command_int_send(
                    master.target_system,
                    master.target_component,
                    _mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    _mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                    0,
                    0,
                    -1,
                    0,
                    0,
                    math.nan,
                    int(lat * 1e7),
                    int(lon * 1e7),
                    alt,
                )
            else:
                target = LocationGlobalRelative(lat, lon, alt)
                vehicle.simple_goto(target)

            return jsonify({'status': 'ok', 'lat': lat, 'lon': lon, 'alt': alt, 'guided_mode': guided_mode})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500

    # ── Servo direct PWM (MAV_CMD_DO_SET_SERVO) ──────────────────────────────
    if cmd == 'servo_set':
        try:
            servo_num = int(data.get('servo_num', 0))
            pwm       = int(data.get('pwm', 1500))
            if servo_num < 1:
                return jsonify({'status': 'error', 'message': 'servo_num must be >= 1'}), 400
            _send_do_set_servo(servo_num, pwm)
            return jsonify({'status': 'ok', 'servo_num': servo_num, 'pwm': pwm})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500

    # ── Fallback ──────────────────────────────────────────────────────────────
    return jsonify({'status': 'received', 'command': cmd})

@flask_app.route('/sensor/rangefinder', methods=['POST', 'OPTIONS'])
def receive_rangefinder():
    if request.method == 'OPTIONS':
        return jsonify({}), 200
        
    data = request.get_json(silent=True) or {}
    distance_cm = data.get('distance_cm')

    if distance_cm is not None:
        try:
            distance_cm = int(distance_cm)
            master = _mavlink_master()
            
            # Send the MAVLink DISTANCE_SENSOR message directly to ArduPilot
            master.mav.distance_sensor_send(
                0,              # time_boot_ms (0 is fine for SITL)
                20,             # min_distance (cm) - Match your RNGFND1_MIN_CM param
                2000,           # max_distance (cm) - Match your RNGFND1_MAX_CM param
                distance_cm,    # current_distance (cm) - The value from Unreal!
                0,              # type (0 = Unknown, 2 = Laser, 4 = SITL)
                0,              # id (Sensor ID 0)
                25,             # orientation (25 = MAV_SENSOR_ROTATION_PITCH_270 / Downward)
                0               # covariance (0 = unknown)
            )
            return jsonify({'status': 'ok', 'distance_cm': distance_cm})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500

    return jsonify({'status': 'error', 'message': 'Missing distance_cm field'}), 400

def start_flask():
    import logging
    # Set to WARNING so you only see real errors, not web request spam
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.WARNING) 
    flask_app.run(host=FLASK_HOST, port=FLASK_PORT, debug=False, threaded=True)

threading.Thread(target=start_flask, daemon=True).start()
threading.Thread(target=_rc_override_heartbeat, daemon=True).start()

# ── Main Unreal Engine Loop ───────────────────────────────────────────────────
print("Starting Bridge Loop. Ready for Unreal Engine and Web Dashboard...")

while True:
    try:
        # Prevent crash if local_frame hasn't initialized yet (GPS Lock pending)
        if vehicle.location.local_frame and vehicle.location.local_frame.north is not None:
            n = vehicle.location.local_frame.north * 100
            e = vehicle.location.local_frame.east * 100
            d = vehicle.location.local_frame.down * -100
        else:
            n, e, d = 0.0, 0.0, 0.0

        roll = math.degrees(vehicle.attitude.roll)
        pitch = math.degrees(vehicle.attitude.pitch)
        yaw = math.degrees(vehicle.attitude.yaw)
        
        

        # We need 14 fields for Unreal Engine
        fields = [0.0] * 14

        # Transform (0-5)
        fields[0], fields[1], fields[2] = n, e, d
        fields[3], fields[4], fields[5] = roll, pitch, yaw

        # Motors (6-10)
        fields[6:10] = motor_state["vtol"]
        fields[10] = motor_state["forward"]

        # QuadPlane Flight State (11)
        if motor_state["is_vtol"] and motor_state["is_fw"]: fields[11] = 3.0
        elif motor_state["is_vtol"]: fields[11] = 1.0
        else: fields[11] = 2.0

        # Gimbal (12-13)
        fields[12] = gimbal_state["pitch"]
        fields[13] = gimbal_state["yaw"]

        # Send to Unreal
        relay.message = tcp_relay.create_fields_string(fields)
        
    except Exception as e:
        print(f"Bridge Loop Error: {e}")
        
    time.sleep(1 / 60)