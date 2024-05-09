import typing
import threading
import serial
import nmea_encode_c_ext.nmea_encode as nmea_encode
import time
import serial.tools.list_ports
import socket

HOST = '192.168.68.43'
PORT = 10110 # standard NMEA port

METRO_M4_PIPE: str = '/dev/ttyACM0'

GAMEPAD_NUM_INPUTS: int = 18

for p in serial.tools.list_ports.comports():
    print(p)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("socket created")
try:
    s.connect((HOST, PORT))
    s.setblocking(0)
    print("socket connected")
except:
    print("socket connection failed")
    exit(1)

logitech_f310_map = {
    "ABS_X": 0, # i16, LSX
    "ABS_Y": 1, # i16, LSY
    "ABS_RX": 2, # i16, RSX
    "ABS_RY": 3, # i16, RSY
    "BTN_THUMBL": 4, # 0, 1, LSB
    "BTN_THUMBR": 5, # 0, 1, RSB
    "ABS_HAT0X": 6, # -1, 0, 1, DPadX
    "ABS_HAT0Y": 7, # -1, 0, 1, DPadY
    "BTN_SOUTH": 8, # 0, 1, A
    "BTN_EAST": 9, # 0, 1, B
    "BTN_NORTH": 10, # 0, 1, X
    "BTN_WEST": 11, # 0, 1, Y
    "BTN_TL": 12, # 0, 1, LB
    "BTN_TR": 13, # 0, 1, RB
    "ABS_Z": 14, # u8, LT
    "ABS_RZ": 15, # u8, RT
    "BTN_START": 16, # 0, 1, Start
    "BTN_SELECT": 17 # 0, 1, Back
}

arslv_transmission: list[list[str]] = [
    ["Left Vert Power", "0"],
    ["Right Vert Power", "0"],
    ["Front Left Power", "0"],
    ["Front Right Power", "0"],
    ["Back Left Power", "0"],
    ["Back Right Power", "0"],
    ["Depth", "0.0"],
    ["Pressure (mbar)", "0.0"],
    ["Yaw", "0.0"],
    ["Pitch", "0.0"],
    ["Roll", "0.0"],
    ["Slowmode", "0"],
    ["Depth Hold", "0"],
    ["Depth Hold Target", "0.0"],
    ["Stabilize", "0"],
    ["Yaw Absolute Current", "0.0"],
    ["Yaw Absolute Target", "0.0"],
    ["Yaw Relative Offset", "0.0"],
    ["Default Multiplier", "0.0"],
    ["Slow Multiplier", "0.0"]
]
gamepad_map = logitech_f310_map

gamepad_inputs: list[int] = [0] * GAMEPAD_NUM_INPUTS
gamepad_input_tuple: tuple[int, ...] = tuple(gamepad_inputs)

nmea_bytes = b''

ser = serial.Serial(METRO_M4_PIPE, 9600, write_timeout = 2)
ser.flush()

def monitor_socket_input():
    try:
        datalen: int = int(s.recv(4).decode('ASCII'))
    except:
        return         
    data: str = s.recv(datalen).decode('ASCII')
    chunks: list[str] = data.split(",");
    if(chunks[0] != "$CTCTL"):
        print("Invalid NMEA header")
        return
    for event in chunks[1:]:
        code, state = event.split(":")
        gamepad_inputs[gamepad_map[code]] = int(state)
    return

def transmit_serial():
    time.sleep(1)
    while(True):
        ser.write(nmea_bytes)
        time.sleep(0.1)

def main():
    global nmea_bytes
    transmission_thread = threading.Thread(target=transmit_serial)
    transmission_thread.daemon = True
    transmission_thread.start()
    while True:
        monitor_socket_input()
        if(ser.in_waiting > 0):
            try:
                arslv_data = ser.readline().decode('ASCII').rstrip().split(",")
                if arslv_data[0] != "$ARSLV":
                    print("Invalid NMEA header")
                    continue
                for i in range(1, len(arslv_data)-1):
                    arslv_transmission[i-1][1] = arslv_data[i]
                print(arslv_transmission)
            except UnicodeDecodeError:
                print("check transmission code")
        gamepad_input_tuple = tuple(gamepad_inputs)
        #print(gamepad_input_tuple)
        nmea_bytes = nmea_encode.nmea_encode(gamepad_input_tuple)

main()