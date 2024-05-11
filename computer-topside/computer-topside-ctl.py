import socket
from inputs import get_gamepad
import time
import threading
import typing
import ast
from tkinter import *

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

def mbar_to_depth(mbar: float) -> float:
    return (mbar*100 - 101300)/(997 * 9.80665)

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

def transmit_gamepad():
    while True:
        conn.send(gamepad_to_nmea())

def main():
    monitor_socket_thread = threading.Thread(target=monitor_socket_input)
    monitor_socket_thread.daemon = True
    monitor_socket_thread.start()
    transmit_gamepad_thread = threading.Thread(target=transmit_gamepad)
    transmit_gamepad_thread.daemon = True
    transmit_gamepad_thread.start()

main()

# ugly tkinter GUI

root = Tk()
root.title("ROV Data")
root.geometry("1400x300")

depth_hold_lbl = Label(root, text="Depth Hold: OFF", font="Arial 25")
depth_hold_lbl.grid(row=1, column=1)

yaw_hold_lbl = Label(root, text="Yaw Stabilize: OFF", font="Arial 25")
yaw_hold_lbl.grid(row=4, column=1)

slowmode_lbl = Label(root, text="Slowmode: OFF", font="Arial 25")
slowmode_lbl.grid(row=7, column=1)

depth_lbl = Label(root, text="Depth: 0.00", font="Arial 25")
depth_lbl.grid(row=1, column=3)

default_gain_lbl = Label(root, text="Default Gain: 0.7", font="Arial 25")
default_gain_lbl.grid(row=4, column=3)

slow_gain_lbl = Label(root, text="Slow Gain: 0.3", font="Arial 25")
slow_gain_lbl.grid(row=7, column=3)

int_temp_lbl = Label(root, text="Internal Temp: 0.00", font="Arial 25")
int_temp_lbl.grid(row=1, column=5)

int_pres_lbl = Label(root, text="Internal Pressure: 0.00", font="Arial 25")
int_pres_lbl.grid(row=4, column=5)

int_humid_lbl = Label(root, text="Internal Humidity: 0.00", font="Arial 25")
int_humid_lbl.grid(row=7, column=5)

ext_temp_lbl = Label(root, text="External Temp: 0.00", font="Arial 25")
ext_temp_lbl.grid(row=1, column=7)

col_count, row_count = root.grid_size()

for col in range(col_count):
    root.grid_columnconfigure(col, minsize=50)

for row in range(row_count):
    root.grid_rowconfigure(row, minsize=25)

def update_labels():
    if(rov_data == {}):
        return
    depth_hold_lbl.config(text="Depth Hold: " + ("ON {:0.2f}".format(mbar_to_depth(float(rov_data["Depth Hold Target"])))) if rov_data["Depth Hold"] == "1" else "OFF")
    yaw_hold_lbl.config(text="Yaw Stabilize: " + "ON" if rov_data["Stabilize"] == "1" else "OFF")
    slowmode_lbl.config(text="Slowmode: " + "ON" if rov_data["Slowmode"] == "1" else "OFF")
    depth_lbl.config(text="Depth: " + rov_data["Depth"])
    default_gain_lbl.config(text="Default Gain: " + rov_data["Default Multiplier"])
    slow_gain_lbl.config(text="Slow Gain: " + rov_data["Slow Multiplier"])
    int_temp_lbl.config(text="Internal Temp: " + rov_data["Internal Temp (C)"])
    int_pres_lbl.config(text="Internal Pressure: " + rov_data["Internal Pressure (mbar)"])
    int_humid_lbl.config(text="Internal Humidity: " + rov_data["Internal Humidity (%)"])
    ext_temp_lbl.config(text="External Temp: " + rov_data["External Temp (C)"])
    root.after(100, update_labels)

time.sleep(2)
update_labels()
root.mainloop()
