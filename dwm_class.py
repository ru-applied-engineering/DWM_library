import serial
import time
import datetime

class DWM:
    line = -1
    x_pos = 0
    y_pos = 0
    z_pos = 0
    qf = 0

    def __init__(self):
        self.serialPort = serial.Serial(port="/dev/ttyS0", baudrate=115200)
        print("Connected to " +self.serialPort.name)
        
        self.serialPort.write("\r\r".encode())
        time.sleep(1)
        
        self.serialPort.write("lep\r".encode())
        
        time.sleep(1)

    # def __init__(self, Port, Baudrate, debug):
    #     self.serialPort = serial.Serial(port=Port, baudrate=Baudrate)
    #     if(debug):
    #         print("Connected to " +DWM.name)
        
    #     DWM.write("\r\r".encode())
    #     time.sleep(1)
        
    #     line = DWM.readline()
    #     if(not(line)):
    #         DWM.write("lep\r".encode())
        
    #     time.sleep(1)

    def get_pos(self, debug):
        try:
            for i in range(10):
                line = self.serialPort.readline()
                line = line.strip()
                # if(debug):
                #     print(line, ", ", len(line))
                if(line):
                    if len(line) >= 15:
                        parse=line.decode().split(",")
                        self.x_pos=parse[1]
                        self.y_pos=parse[2]
                        self.z_pos = parse[3]
                        self.qf=parse[4]
                        if(debug):
                            print(datetime.datetime.now().strftime("%H:%M:%S"),
                            ": x:", self.x_pos,
                            ", y:", self.y_pos,
                            ", z:", self.z_pos,
                            ", qf:", self.qf,
                            )
                        
                        break
                    else:
                        if(debug):
                            print("Position not calculated: ", line.decode(), " Length: ", len(line))
               
                if i == 10:
                    print("Could not get pos, tried 10 times")   
        
        except Exception as ex:
            print(ex)


dwm1 = DWM()

while 1:
    dwm1.get_pos(1)