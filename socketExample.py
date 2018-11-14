#!/usr/bin/env python3

import subprocess
import threading
import socket
import time

usrInput=b""

def listen():
    monitorshell = subprocess.Popen("termite --exec=\"nc -l -p 8001\"",shell=True)

def sendData():
    while True:
        time.sleep(1)
        sock.send(bytes(usrInput))

t1 = threading.Thread(target=listen).start()
time.sleep(1)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('localhost', 8001))

t2 = threading.Thread(target=sendData).start()

while True:
    time.sleep(1)
    usrInput = input("Input: ").encode()

