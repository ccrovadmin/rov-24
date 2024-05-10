import socket
from inputs import get_gamepad
import time
import threading
import typing
import ast

HOST = '192.168.68.43'
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
s.setblocking(0)

nmea_bytes: bytes = b''
rov_data: dict[str, str] = {}

def monitor_socket_input():
    global rov_data
    time.sleep(1)
    while True:
        try:
            datalen: int = int(conn.recv(4).decode('ASCII'))
            data: str = conn.recv(datalen).decode('ASCII')
            chunks: list[str] = data.split(",")
            rov_data_str: str = ",".join(chunks[1:-1])
            if chunks[0] != "$RPCTL":
                print("Invalid NMEA header")
                return
            rov_data = ast.literal_eval(rov_data_str)
        except ValueError:
            pass
        print(rov_data)
        time.sleep(0.15)


def gamepad_to_nmea() -> bytes:
    events = get_gamepad()
    transmission: bytes = None
    nmea: str = NMEA_HEADER
    len_str: str = ""
    for event in events:
        if(event.code != "SYN_REPORT"):
            nmea += "," + event.code + ":" + str(event.state) 
    len_str = str(len(nmea)).zfill(4)
    transmission = (len_str + nmea).encode('ASCII')
    #print("Data to send: ", transmission)
    return transmission

def main():
    monitor_socket_thread = threading.Thread(target=monitor_socket_input)
    monitor_socket_thread.daemon = True
    monitor_socket_thread.start()
    while True:
        conn.send(gamepad_to_nmea())
main()
    