import network
import socket
import struct
import time
import machine

# ============================== CONFIG ==============================
WIFI_SSID = "SWARM_BOT_03" # Change the number of your bot with the assigned team number.
WIFI_PASSWORD = "password" # Change your password within the "s.
WIFI_CHANNEL = 1  # Must be 1, 6, or 11. Change according to the given instructions.
WIFI_TX_POWER_DBM = 8  
UDP_PORT = 5005

DEAD_ZONE = 0.05       
GAIN = 3.0             
MAX_SPEED_PCT = 70     

COMMAND_TIMEOUT_MS = 600
OBSTACLE_DISTANCE_CM = 15.0

LOOP_PERIOD_MS = 40
ULTRASONIC_EDGE_TIMEOUT_US = 12000
ULTRASONIC_PULSE_TIMEOUT_US = 12000

# ---------- TB6612FNG motor-driver pins ----------
PWMA = machine.PWM(machine.Pin(33), freq=1000, duty_u16=0)
AIN1 = machine.Pin(26, machine.Pin.OUT)
AIN2 = machine.Pin(25, machine.Pin.OUT)
PWMB = machine.PWM(machine.Pin(13), freq=1000, duty_u16=0)
BIN1 = machine.Pin(14, machine.Pin.OUT)
BIN2 = machine.Pin(12, machine.Pin.OUT)
STBY = machine.Pin(27, machine.Pin.OUT)
STBY.value(1)
# ---------- HC-SR04 ultrasonic sensor pins ----------
TRIG = machine.Pin(32, machine.Pin.OUT)
ECHO = machine.Pin(35, machine.Pin.IN)


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def clamp_pwm(duty):
    return int(clamp(int(duty), 0, 65535))


def set_one_motor(pwm, in1, in2, speed):
    
    speed = clamp(speed, -1.0, 1.0)

    if speed > 0:
        in1.value(1)
        in2.value(0)
    elif speed < 0:
        in1.value(0)
        in2.value(1)
    else:
        in1.value(0)
        in2.value(0)

    duty = clamp_pwm(abs(speed) * 65535)
    pwm.duty_u16(duty)


def set_motors(left_speed, right_speed):
    set_one_motor(PWMA, AIN1, AIN2, left_speed)
    set_one_motor(PWMB, BIN1, BIN2, right_speed)


def stop_motors():
    set_motors(0.0, 0.0)


def apply_dead_zone(value):
    
    if -DEAD_ZONE < value < DEAD_ZONE:
        return 0.0

    if value > 0:
        return value - DEAD_ZONE
    return value + DEAD_ZONE


def component_to_control(component):
   
    value = apply_dead_zone(component) * GAIN
    speed_limit = clamp(MAX_SPEED_PCT, 0, 100) / 100.0
    return clamp(value, -speed_limit, speed_limit)


def read_osc_string(packet, offset):
   
    end = packet.find(b"\x00", offset)
    if end < 0:
        raise ValueError("OSC string has no null terminator")

    text = packet[offset:end]
    next_offset = (end + 4) & ~3

    if next_offset > len(packet):
        raise ValueError("OSC string padding exceeds packet")

    return text, next_offset


def parse_osc_quaternion(packet):
    
    try:
        offset = 0
        
        if packet.startswith(b"#bundle"):
            offset = 16 
            
            while offset + 4 <= len(packet):
                msg_size = struct.unpack(">i", packet[offset:offset+4])[0]
                offset += 4
                msg_end = offset + msg_size
                
                address, next_offset = read_osc_string(packet, offset)
                if address.endswith(b"/quaternion"):
                    type_tags, next_offset = read_osc_string(packet, next_offset)
                    offset = next_offset  
                    break                 
                else:
                    offset = msg_end      
            else:
                return None               
                
        else:
            
            address, offset = read_osc_string(packet, 0)
            if not address.endswith(b"/quaternion"):
                return None
            type_tags, offset = read_osc_string(packet, offset)

        if not type_tags.startswith(b","):
            return None

        arguments = []
        for tag in type_tags[1:]:
            if offset + 4 > len(packet):
                return None
            if tag == ord("f"):
                value = struct.unpack(">f", packet[offset:offset + 4])[0]
            elif tag == ord("i"):
                value = struct.unpack(">i", packet[offset:offset + 4])[0]
            else:
                return None
            arguments.append(value)
            offset += 4

        if len(arguments) < 4:
            return None

        x, y, z, w = arguments[0], arguments[1], arguments[2], arguments[3]

        for value in (x, y, z, w):
            if value != value or value > 1.0e6 or value < -1.0e6:
                return None

        return x, y, z, w

    except (ValueError, IndexError, struct.error):
        return None


def wait_for_echo_level(level, timeout_us):
    
    start = time.ticks_us()
    while ECHO.value() != level:
        if time.ticks_diff(time.ticks_us(), start) >= timeout_us:
            return False
    return True


def read_distance_cm():
    
    TRIG.value(0)
    time.sleep_us(2)
    TRIG.value(1)
    time.sleep_us(10)
    TRIG.value(0)

    if not wait_for_echo_level(1, ULTRASONIC_EDGE_TIMEOUT_US):
        return None

    pulse_start = time.ticks_us()
    if not wait_for_echo_level(0, ULTRASONIC_PULSE_TIMEOUT_US):
        return None

    pulse_us = time.ticks_diff(time.ticks_us(), pulse_start)
    return pulse_us / 58.0


def setup_access_point():
    if WIFI_CHANNEL not in (1, 6, 11):
        raise ValueError("WIFI_CHANNEL must be 1, 6, or 11")

    ap = network.WLAN(network.AP_IF)
    ap.active(True)

    ap.config(essid=WIFI_SSID, channel=WIFI_CHANNEL, authmode=3, password=WIFI_PASSWORD)

    if WIFI_TX_POWER_DBM is not None:
        try:
            ap.config(txpower=WIFI_TX_POWER_DBM)
            print("AP TX power set to", WIFI_TX_POWER_DBM, "dBm")
        except Exception as error:
            print("AP TX-power configuration unsupported; using firmware default:", error)

    print("AP active:", ap.active())
    print("SSID:", WIFI_SSID, "IP:", ap.ifconfig()[0], "channel:", WIFI_CHANNEL)

    return ap


def setup_udp_socket():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(("0.0.0.0", UDP_PORT))
    udp_socket.settimeout(0)
    print("Listening for OSC UDP on port", UDP_PORT)
    return udp_socket


def main():
    setup_access_point()
    udp_socket = setup_udp_socket()

    quaternion_x = 0.0
    quaternion_y = 0.0
    last_valid_command_ms = time.ticks_ms()

    while True:
        loop_started_ms = time.ticks_ms()

        for _ in range(8):
            try:
                packet, _sender = udp_socket.recvfrom(512)
            except OSError:
                break

            quaternion = parse_osc_quaternion(packet)
            #Before connecting to the ESP32, first uncomment the below print statement and verify that your getting an output.
            #print("Parser output:", quaternion) 
            if quaternion is not None:
                quaternion_x = quaternion[0]
                quaternion_y = quaternion[1]
                last_valid_command_ms = time.ticks_ms()
                

        distance_cm = read_distance_cm()
        obstacle_present = (
            distance_cm is None or distance_cm < OBSTACLE_DISTANCE_CM
        )

        command_age_ms = time.ticks_diff(
            time.ticks_ms(),
            last_valid_command_ms
        )
        command_timed_out = command_age_ms > COMMAND_TIMEOUT_MS

        if obstacle_present or command_timed_out:
            
            stop_motors()
        else:
            throttle = component_to_control(quaternion_y)
            steering = component_to_control(quaternion_x)

            left_speed = clamp(throttle + steering, -1.0, 1.0)
            right_speed = clamp(throttle - steering, -1.0, 1.0)
            set_motors(left_speed, right_speed)

        elapsed_ms = time.ticks_diff(time.ticks_ms(), loop_started_ms)
        remaining_ms = LOOP_PERIOD_MS - elapsed_ms
        if remaining_ms > 0:
            time.sleep_ms(remaining_ms)


try:
    main()
finally:
    stop_motors()
    STBY.value(0)