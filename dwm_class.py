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

    def __init__(self, Close_Serial_Between, debug):
        self.debug = debug
        self.close_serial_between = Close_Serial_Between

        if(self.close_serial_between == 0):
            self.serialPort = serial.Serial(port="/dev/ttyS0", baudrate=115200, timeout=10)
            print("Connected to " + self.serialPort.name)
            self.write_uart("\r\r", 0.1)

    def write_uart(self, command, time_sleep): # Writing commands to uart
        self.serialPort.write(command.encode(), "\r".encode())
        time.sleep(time_sleep)

    def write_debug(self, debug_message): # Writing debug message
        if (self.debug):
            print(debug_message)
            
    def connect_to_node(self): # Connecting to node
        self.serialPort = serial.Serial(port="/dev/ttyS0", baudrate=115200, timeout=10)
        self.write_debug("Connected to " + self.serialPort.name)

        write_uart("\r\r", 0.1)

    def disconnect_from_node(self, debug): # Disconnecting from node
        self.write_uart("exit", 0.1)
        self.serialPort.close()
        if(self.debug):
            print("Connection closed")

    def reset_node(self):
        self.write_uart("reset")
        # Gera eitthvað meira, tengjast aftur??

    def get_pos(self, debug): # Getting posistion
        try:
            if(self.close_serial_between):
                self.connect_to_node()

            for i in range(10):
                self.write_uart("apg\r", 0)

                trash = self.serialPort.readline()
                line = self.serialPort.readline()  # Reading incoming data from node
                line = line.strip() # Taking \n and other symbols away
                numbers = [0, 0, 0, 0]  # Create array for only numbers
                number_index = 0
                if len(line) >= 15:  # If the data is approriate length
                    for axis in line.split(b' '):  # Split the data up into axis and quality factor
                        for temp in axis.split(b':'): # Splitting axis name from numbers
                            if temp.lstrip(b'-').replace(b'.',b'', 1).isdigit(): # If the data is a number
                                numbers[number_index] = int(temp)  # Putting it into number
                                number_index += 1

                    self.x_pos = numbers[0] # Putting into approriate data
                    self.y_pos = numbers[1]
                    self.z_pos = numbers[2]
                    self.qf = numbers[3]

                    # If debug mode is enabled print out numbers
                    self.write_debug(datetime.datetime.now().strftime("%H:%M:%S"),
                    ": x:", self.x_pos,
                    ", y:", self.y_pos,
                    ", z:", self.z_pos,
                    ", qf:", self.qf,
                    )

                    if(self.close_serial_between):
                        self.disconnect_from_node()

                    break

                else:
                    self.write_debug("Line:", line.decode(), " Length: ", len(line))
                
                if (i == 10):
                    print("Failed to get position, tried 10 times!")

                
        except Exception as ex:
            print(ex)


    def get_acc_data(self):
        self.write_uart("av\r")
        #parsa gögn

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
    

dwm1 = DWM(1, 1)

while 1:
    dwm1.get_pos(1)