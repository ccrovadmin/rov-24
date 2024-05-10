import typing
import glob
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

logitech_f310_map: dict[str, int] = {
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

arslv_transmission_map: list[str] = [
    "Left Vert Power",
    "Right Vert Power",
    "Front Left Power",
    "Front Right Power",
    "Back Left Power",
    "Back Right Power",
    "Depth",
    "Pressure (mbar)",
    "Yaw",
    "Pitch",
    "Roll",
    "Slowmode",
    "Depth Hold",
    "Depth Hold Target",
    "Stabilize",
    "Yaw Absolute Current",
    "Yaw Absolute Target",
    "Yaw Relative Offset",
    "Default Multiplier",
    "Slow Multiplier"
]

rov_data: dict[str, str] = {
    "Left Vert Power": "",
    "Right Vert Power": "",
    "Front Left Power": "",
    "Front Right Power": "",
    "Back Left Power": "",
    "Back Right Power": "",
    "Depth": "",
    "Pressure (mbar)": "",
    "Yaw": "",
    "Pitch": "",
    "Roll": "",
    "Slowmode": "",
    "Depth Hold": "",
    "Depth Hold Target": "",
    "Stabilize": "",
    "Yaw Absolute Current": "",
    "Yaw Absolute Target": "",
    "Yaw Relative Offset": "",
    "Default Multiplier": "",
    "Slow Multiplier": "",
    "External Temp (C)": ""
}

gamepad_map = logitech_f310_map

gamepad_inputs: list[int] = [0] * GAMEPAD_NUM_INPUTS
gamepad_input_tuple: tuple[int, ...] = tuple(gamepad_inputs)

nmea_bytes = b''

ser = serial.Serial(METRO_M4_PIPE, 9600, write_timeout = 2)
ser.flush()

# temperature code modified from
# https://learn.adafruit.com/adafruits-raspberry-pi-lesson-11-ds18b20-temperature-sensing/hardware


ds18b20s = glob.glob("/sys/bus/w1/devices/28*")[0] + "/w1_slave"

def monitor_temp():
    global external_temp_C
    time.sleep(1)
    while True:
        f = open(ds18b20s, 'r')
        lines = f.readlines()
        f.close()
        if lines[0].strip()[-3:] != 'YES':
            time.sleep(0.2)
            continue
        equals_pos = lines[1].find('t=')
        if equals_pos == -1:
            time.sleep(0.2)
            continue
        rov_data["External Temp (C)"] = lines[1][equals_pos+2:-1]
        #print(lines[1][equals_pos+2:-1])
        time.sleep(0.2)

def monitor_socket_input():
    try:
        datalen: int = int(s.recv(4).decode('ASCII'))
    except:
        return         
    data: str = s.recv(datalen).decode('ASCII')
    chunks: list[str] = data.split(",");
    if(chunks[0] != "$CTCTL"):
        print("Invalid NMEA header topside")
        return
    for event in chunks[1:]:
        code, state = event.split(":")
        gamepad_inputs[gamepad_map[code]] = int(state)
    return

def transmit_topside_socket():
    time.sleep(1)
    while(True):
        print(rov_data)
        nmea: str = "$RPCTL," + str(rov_data) + ",*FF" # dummy checksum
        len_str: str = str(len(nmea)).zfill(4)
        transmission = (len_str + nmea).encode('ASCII')
        s.send(transmission)
        time.sleep(0.2)

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
    temp_thread = threading.Thread(target=monitor_temp)
    temp_thread.daemon = True
    temp_thread.start()
    topside_thread = threading.Thread(target=transmit_topside_socket)
    topside_thread.daemon = True
    topside_thread.start()
    while True:
        monitor_socket_input()
        if(ser.in_waiting > 0):
            try:
                arslv_data = ser.readline().decode('ASCII').rstrip().split(",")
                if arslv_data[0] != "$ARSLV":
                    print("Invalid NMEA header arduino")
                    continue
                # first and last elements are header and checksum
                for i in range(1, len(arslv_data)-1):
                    rov_data[arslv_transmission_map[i-1]] = arslv_data[i]
            except UnicodeDecodeError:
                print("unicode decode error")
        gamepad_input_tuple = tuple(gamepad_inputs)
        #print(gamepad_input_tuple)
        nmea_bytes = nmea_encode.nmea_encode(gamepad_input_tuple)

main()