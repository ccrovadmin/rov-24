import socket
from inputs import get_gamepad
import typing

HOST = '192.168.1.75'
PORT = 10110 # standard NMEA port

NMEA_HEADER = "$CTCTL" # computer topside control

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("socket created")
s.bind((HOST, PORT))
print("socket bound")
s.listen()
print("listening for connection")
conn, addr = s.accept()
print("connection address:", addr)

def gamepad_to_nmea() -> bytes:
    events = get_gamepad()
    transmission: bytes = None
    nmea: str = NMEA_HEADER
    len_str: str = ""
    for event in events:
        if event.code != "SYN_REPORT":
            nmea += "," + event.code + ":" + str(event.state) 
    len_str = str(len(nmea)).zfill(4)
    transmission = (len_str + nmea).encode('ASCII')
    print("Data to send: ", transmission)
    return transmission

while True:
    conn.send(gamepad_to_nmea())