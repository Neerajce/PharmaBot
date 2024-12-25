
import serial
import time

def write_ser(cmd):
    ard.write(cmd)

while 1:
    ard = serial.Serial("/dev/ttyUSB1",9600)
    line = ard.readline().decode("utf-8").strip()
    print(line)
    # if(line[0] == 'e'):  
    #     #temp_one = line.replace('e','')
    #     temp  = line.split(',')
    #     print(temp)
    # rpm_motor_right = int(temp[0])
    # rpm_motor_left = int(temp[1])
    # print("rpm_motor_right is ",rpm_motor_right)
    # print("rpm_motor_left is ",rpm_motor_left)
    cmd = input("give input ")
    write_ser(cmd.encode())
