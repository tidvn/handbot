import serial
import time
import struct
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=.1)

def write_read(s0=0,s1=0,s2=0,s3=0,s4=0,s5=0):
    arduino.write(struct.pack('>BBBBBB',s0,s1,s2,s3,s4,s5))
    time.sleep(0.05)
    data = arduino.readline()
    data = data.decode("utf-8")
    return data
while True:
    # for i in range(0,90):
    #     value = write_read(0,0,0,0,0,i)
    #     print(i)
    #     time.sleep(0.1)
    # # value = write_read(0,0,0,0,0,0)
    # # print(value)
    value=write_read(90,90,90,90,90,90)
    print(value)
    time.sleep(1)
    value2=write_read(90,90,90,90,90,180)
    print(value2)
    time.sleep(1)