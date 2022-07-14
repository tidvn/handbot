import serial
import time
import struct
import serial.tools.list_ports

class SerialObject:
    def __init__(self, portNo=None, baudRate=9600):
        self.portNo = portNo
        self.baudRate = baudRate
        connected = False
        if self.portNo is None:
            ports = serial.tools.list_ports.comports()
            for p in ports:
                if len(ports) is not None:
                    print(f'{p.device} Connected')
                    self.ser = serial.Serial(p.device)
                    self.ser.baudrate = baudRate
                    connected = True
            if not connected:
                logging.warning("Arduino Not Found. Please enter COM Port Number instead.")
        else:
            try:
                self.ser = serial.Serial(self.portNo, self.baudRate)
                print("Serial Device Connected")
            except:
                logging.warning("Serial Device Not Connected")
    def sendData(self, s1=0,s2=0,s3=0,s4=0,s5=0,s6=0):
        try:
            self.ser.write(struct.pack('>BBBBBB',s1,s2,s3,s4,s5,s6))
            return True
        except:
            return False
                
    # def sendData(self, s1=0,s2=0,s3=0):
    #     try:
    #         self.ser.write(struct.pack('>BBB',s1,s2,s3))
    #         print("send data",s1,s2,s3)
    #         return True
    #     except:
    #         print("err")
    #         return False

    def getData(self):

        data = self.ser.readline()
        data = data.decode("utf-8")
        return data
# arduino = SerialObject()
# def main():
    
#     while True:
#         arduino.sendData(0,0,0)
#         # print(arduino.getData())

#         time.sleep(1)

#         arduino.sendData(30,30,30)
#         # arduino.getData()
#         time.sleep(1)
# if __name__ == "__main__":
#     main()
