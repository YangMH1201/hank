#!/usr/bin/env python3
import serial
import threading
import time

# COM_PORT = "COM6"
COM_PORT = '/dev/ttyUSB1'
BAUD_RATES = 115200
ser = serial.Serial(COM_PORT, BAUD_RATES)

class UwbModule:
    def __init__(self):

        def serialthread():
            while self.serthread_alive:
                try:
                    while self.serial.in_waiting:
                        recv = self.serial.readline().decode()
                        if 'device' not in recv and 'added' not in recv:
                            #print("recv", recv)
                            self.uwb_data = recv
                        else:
                            print("IN ELSE")
                            print(recv)
                            self.uwb_data = None
                except Exception as err:
                    print(f"arduino UWB error: {err}.......{self.uwb_data}")
                    self.uwb_data = None

        self.serial = ser
        self.serthread_alive = True
        self.serthread = threading.Thread(target=serialthread)
        self.serthread.start()
        self.uwb_data = None

    def get_module_data(self):
        # self.uwb_data = None
        # self.uwb_data = 'dis'
        return self.uwb_data

    def close(self):
        self.serthread_alive = False
        self.serial.close()


uwb = UwbModule()
while True:
    print(f"{uwb.get_module_data()}")
    time.sleep(0.1)
