import serial
import time
import datetime

class DWM:
    line = ""
    x_pos = 0
    y_pos = 0
    z_pos = 0
    qf = 0

    def __init__(self):
        self.serialPort = serial.Serial(port="/dev/ttyS0", baudrate=115200, timeout=10)
        print("Connected to " + self.serialPort.name)
        self.serialPort.write("\r\r".encode())
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
            self.serialPort.write("lep\r".encode())  # Telling DWM1001C node to send pos
            for i in range(10):
                print(i)
                line = self.serialPort.readline()  # Reading incoming data from node
                line = line.strip() # Taking \n and other symbols away
                
                if len(line) >= 15: # If the data is approriate length
                    numbers = [] # Create array for only numbers
                    for temp in line.split(): # Split the data up
                        if temp.isdigit(): # If the data is a number
                            numbers.append(int(temp)) # Putting it into number

                    self.x_pos = numbers[0] # Putting into approriate data
                    self.y_pos = numbers[1]
                    self.z_pos = numbers[2]
                    self.qf = numbers[3]

                    if(debug): # If debug mode is enabled
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
                
                # if(debug):
                #     print(line, ", ", len(line))
                if (i == 10):
                    print("Tried 10 time to get position, failed!")

                self.serialPort.write("lec \r".encode())  # Telling DWM1001C node to send pos
                
        except Exception as ex:
            print(ex)

    def get_orientation(self):
        self.serialPort.write("av\r".encode()) # Telling DWM1001C node to send pos


dwm1 = DWM()

while 1:
    dwm1.get_pos(1)