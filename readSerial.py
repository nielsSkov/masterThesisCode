#!/usr/bin/env python3

import threading
import socket
import subprocess
import time
import serial
from pathlib import Path

def serialInSocketOutAndLog():
    log = 0
    while True:
        serRead = ser.readline()
        serReadUtf8 = serRead.decode('utf-8')
        serRead = serReadUtf8.encode()

        sock.send(bytes(serRead))
        
        if "startLogging" in serReadUtf8: 
            log = 1
            newState = 1
        elif "stopLogging" in serReadUtf8:
            log = 0
            newState = 1
        else:
            newState = 0
        
        if newState and log:
           logFile = open(logPathToFile,'a')
        elif newState and not log:
           logFile.close()

        if log and not newState:
            logFile.write(serReadUtf8)

def testNameSelect():
    global testName
    global logPathToFile

    testName      = input("Name of test for log filename: ")
    pathFromHome  = '/thesis/matlab/parameterEstimation/data/cartTest3/'
    home          = str(Path.home())
    logPathToFile = home+pathFromHome+testName+'.csv'

def serialOut():
    while True:
        usrInput = input("Input: ").encode()
        if usrInput is b"t":
            testNameSelect()
        else:
            ser.write(bytes(usrInput))

def listen():
    listenTermCmd = "termite --exec=\"nc -l -p 8001\""
    monitorshell  = subprocess.Popen( listenTermCmd, shell = True )


#clear terminal (same result as 'clear')
print("\033[H\033[J")

ser = serial.Serial('/dev/ttyACM0', 115200)

testNameSelect()

t1 = threading.Thread(target=listen).start()
time.sleep(2)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('localhost', 8001))

t2 = threading.Thread(target=serialInSocketOutAndLog).start()

t3 = threading.Thread(target=serialOut).start()

