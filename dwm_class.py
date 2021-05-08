import serial
import time
import datetime

class DWM:
    x_pos = 0
    y_pos = 0
    z_pos = 0
    qf = 0

    x_acc = 0
    y_acc = 0
    z_acc = 0

    def __init__(self, Port = '/dev/ttyACM0', debug = 0):
        self.debug = debug

        self.serialPort = serial.Serial(port=Port, baudrate=115200, timeout=10)
        self.write_debug("Connected to " + self.serialPort.name)
        self.write_uart("\r", 0.1)

    def __str__(self):
        return "x: {}, y: {}, z: {}".format(self.x_pos, self.y_pos, self.z_pos)

    def write_uart(self, command, time_sleep): # Writing commands to uart
        self.serialPort.write("{}\r".format(command).encode())
        time.sleep(time_sleep)

    def write_debug(self, debug_message): # Writing debug message
        if (self.debug):
            print(debug_message)
            
    def connect_to_node(self, Port):  # Connecting to node
        self.serialPort = serial.Serial(port="/dev/Port", baudrate=115200, timeout=10)
        self.write_debug("Connected to " + self.serialPort.name)

        self.write_uart("\r", 0.1)

    def disconnect_from_node(self): # Disconnecting from node
        #self.write_uart("quit", 0.1)
        self.serialPort.close()
        self.write_debug("Connection closed")

    def reset_node(self):
        self.write_uart("reset" ,0.5)
        # Gera eitthvað meira, tengjast aftur??

    def get_pos(self): # Getting posistion
        try:
            for i in range(10):
                self.write_uart("apg\r", 0.1)

                trash = self.serialPort.readline() # There is always 0 reading in the beginning, throwing it out
                line = self.serialPort.readline()  # Reading incoming data from node
                line = line.strip() # Taking \n and other symbols away
                if len(line) >= 9:  # If the data is approriate length
                    line = line[5:]
                    axis = line.split()
                    axis_numbers = []  # Create array for only numbers
                    for temp in axis:
                        temp = temp.split(b':')
                        axis_numbers.append(int(temp[1]))

                    self.x_pos = axis_numbers[0] # Putting into approriate data
                    self.y_pos = axis_numbers[1]
                    self.z_pos = axis_numbers[2]
                    self.qf = axis_numbers[3]

                    # If debug mode is enabled print out numbers
                    # self.write_debug(datetime.datetime.now().strftime("%H:%M:%S") +
                    # ": x:" + self.x_pos +
                    # ", y:" + self.y_pos +
                    # ", z:" + self.z_pos +
                    # ", qf:"+ self.qf
                    # )
                    self.write_debug("x: {}, y: {}, z: {}".format(self.x_pos, self.y_pos, self.z_pos))

                    if (i == 10):
                        print("Failed to get position, tried 10 times!")

                    break

                else:
                    self.write_debug('Line: {} Length: {}'.format(line.decode(), len(line)))

                
        except Exception as ex:
            print(ex)


    def get_acc_data(self):
        try:
            if(self.close_serial_between):
                self.connect_to_node()

            for i in range(10):
                self.write_uart("av", 0.1)

                #trash = self.serialPort.readline() # There is always 0 reading in the beginning, throwing it out
                line = self.serialPort.readline()  # Reading incoming data from node
                #line = line.strip()  # Taking \n and other symbols away
                print(line)
                numbers = [0, 0, 0]  # Create array for only numbers
                if len(line) >= 15:  # If the data is approriate length
                    line = line[5:]
                    axis = line.split()
                    print(axis)
                    axis_numbers = []  # Create array for only numbers
                    for temp in axis:
                        print(temp)
                        temp = temp.split(b',')
                        axis_numbers.append(int(temp[0]))

                    self.x_acc = numbers[0] # Putting into approriate data
                    self.y_acc = numbers[1]
                    self.z_acc = numbers[2]

                    # If debug mode is enabled print out numbers
                    self.write_debug(datetime.datetime.now().strftime("%H:%M:%S") +
                    ": x:"+ self.x_pos+
                    ", y:"+ self.y_pos+
                    ", z:"+ self.z_pos+
                    ", qf:"+ self.qf
                    )

                    if(self.close_serial_between):
                        self.disconnect_from_node()

                    break

                else:
                    self.write_debug("Line:" + line.decode() + " Length: " + len(line))
                
                if (i == 10):
                    print("Failed to get position, tried 10 times!")

                
        except Exception as ex:
            print(ex)

    def pub_pos(self):
        print("Publishing pos on mosqitto")


    def sub_pos(self, topic):
        print("Subcribing pos on mosqitto", topic)
    
    def get_device_uptime(self):
        self.write_uart("ut\r",0)
        #parsa gögn

    def factory_reset_node(self):
        sure = input("Are you sure to factory reset?(y/n) ")
        loop = 1
        if (sure == "y"):
            self.write_uart("frst\r", 0)
        else:
            print("Aborting factory reset")

    def get_stationary_conf(self):
        print("Serial command: scg")

    def set_stationary_conf(self, config_number):
        print("Serial command: scs ", config_number)

    def get_pos_update_rate(self):
        print("Serial command: aurg")

    def set_pos_update_rate(self, pos_update_rate):
        print("Serial command: aurs", pos_update_rate)

    def set_pos_node(self,x,y,z):
        print("Serial command: aps", x,y,z)

    def get_transm_power(self):
        print("Serial command: utpg")

    def set_transm_power(self, transm_number):
        print("Serial command: utps", transm_number)

    def get_system_info(self):
        print("Serial command: si")
    
    def get_mode_info(self):
        print("Serial command: nmg")

    def show_anchor_list(self):
        print("Serial command: la")

    def show_bn_list(self):
        print("Serial command: lb")

    def set_network_id(self, network_id):
        print("Serial command: nis", network_id)

    def set_node_label(self, node_label):
        print("Serial command: nls", node_label)

    def display_stats(self):
        print("Serial command: stg")

    def clear_stats(self):
        print("Serial command: stc")
    


dwm1 = DWM(debug=1)


def updating_pos():
    global dwm1

    dwm1.get_pos()


while 1:
    updating_pos()
