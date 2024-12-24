import serial
import time

def write_ser(cmd):
    ard.write(cmd)
while 1:
    ard = serial.Serial("/dev/ttyUSB1",9600)

    cmd = input("give input ")
    write_ser(cmd.encode())