import serial
import time
import datetime

class DWM:
    def __init__(self, Port = '/dev/ttyACM0', debug = 0):
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.z_pos = 0.0
        self.qf = 0.0

        self.x_acc = 0.0
        self.y_acc = 0.0
        self.z_acc = 0.0

        self.debug = debug
        self.connect_to_node(Port)

    def __str__(self):
        return "x: {}, y: {}, z: {} qf: {} ".format(self.x_pos, self.y_pos, self.z_pos, self.qf)

    def write_uart(self, command, time_sleep):  # Writing commands to uart
        self.serialPort.write(command.encode())
        time.sleep(time_sleep)

    def clear_uart(self):    
        self.serialPort.flushInput()    
        self.serialPort.flushOutput()

    def write_debug(self, debug_message): # Writing debug message
        if (self.debug):
            print(debug_message)
            
    def connect_to_node(self, Port):  # Connecting to node
        self.serialPort = serial.Serial(port=Port, baudrate=115200, timeout=10)
        self.write_debug("Connected to " + self.serialPort.name)
        self.write_uart("\r\r", 1)

    def disconnect_from_node(self): # Disconnecting from node
        #self.write_uart("quit", 0.1)
        self.serialPort.close()
        self.write_debug("Connection closed")

    def reset_node(self):
        self.write_uart("reset" ,0.5)
        # Gera eitthvað meira, tengjast aftur??

    def get_pos(self): # Getting posistion
        try:
            self.clear_uart()
            for i in range(10):
                #print("i: {}".format(i))
                self.write_uart("apg\r", 0.1)

                #trash = self.serialPort.readline() # There is always 0 reading in the beginning, throwing it out
                line = self.serialPort.readline()  # Reading incoming data from node
                line = line.strip()  # Taking \n and other symbols away
                #print("Line len: {}".format(len(line)))
                if len(line) >= 15:  # If the data is approriate length
                    line = line[5:]
                    axis = line.split()
                    axis_numbers = []  # Create array for only numbers
                    for temp in axis:
                        temp = temp.split(b':')
                        if(len(temp) >= 2):
                            axis_numbers.append(int(temp[1]))
                    if(len(axis_numbers) >= 4):
                        self.x_pos = axis_numbers[0] # Putting into approriate data
                        self.y_pos = axis_numbers[1]
                        self.z_pos = axis_numbers[2]
                        self.qf = axis_numbers[3]

                        self.write_debug("x: {}, y: {}, z: {} qf: {}".format(self.x_pos, self.y_pos, self.z_pos, self.qf))  # If debug mode is enabled print out numbers

                        break

                    if (i == 10):
                        print("Failed to get position, tried 10 times!")

                # else:
                #     self.write_debug('Line: {} Length: {}'.format(line.decode(), len(line)))

            self.clear_uart()
                
        except Exception as ex:
            print(ex)

    def get_pos_avg(self, n):
        try:
            x_pos_sum, y_pos_sum, z_pos_sum, qf_sum = 0.0,0.0,0.0,0.0
            self.clear_uart()
            self.write_uart("lep\r", 0.1)
            i = 0
            while i < n:
                line = self.serialPort.readline().strip()  # Reading incoming data from node & taking \n and other simbol
                if len(line) >= 15:  # If the data is approriate length
                    axis = line.split(b',')
                    if(len(axis) >= 5):
                        x_pos_sum += float(axis[1]) # Putting into approriate data
                        y_pos_sum += float(axis[2])
                        z_pos_sum += float(axis[3])
                        qf_sum += float(axis[4])

                        i += 1
                    else:
                        i -= 1

                # else:
                #     self.write_debug('Line: {} Length: {}'.format(line.decode(), len(line)))

            self.x_pos = 1000*(x_pos_sum / float(n))
            self.y_pos = 1000*(y_pos_sum / float(n))
            self.z_pos = 1000*(z_pos_sum / float(n))
            self.qf = qf_sum / float(n)

            self.clear_uart()
            time.sleep(0.1)
            self.write_uart("lep\r", 0.1)
            
            self.write_debug("x: {}, y: {}, z: {} qf: {}".format(self.x_pos, self.y_pos, self.z_pos, self.qf))  # If debug mode is enabled print out numbers
            self.clear_uart()
                
        except Exception as ex:
            print(ex)

    def get_acc_data(self):
        try:
            self.clear_uart()
            for i in range(10):
                self.write_uart("av\r", 0.1)

                #trash = self.serialPort.readline() # There is always 0 reading in the beginning, throwing it out
                trash = self.serialPort.readline()
                line = self.serialPort.readline()  # Reading incoming data from node
                line = line.strip()  # Taking \n and other symbols away
                if len(line) >= 15:  # If the data is approriate length
                    line = line[5:]
                    axis = line.split(b',')
                    axis_numbers = []
                    for temp in axis:
                        temp = temp.split(b'=')
                        if(len(temp) >= 2):
                            axis_numbers.append(int(temp[1]))
                    if(len(axis_numbers) >= 3):
                        self.x_pos = int(axis_numbers[0]) # Putting into approriate data
                        self.y_pos = int(axis_numbers[1])
                        self.z_pos = int(axis_numbers[2])

                        self.write_debug("ACC: x: {}, y: {}, z: {}".format(self.x_pos, self.y_pos, self.z_pos))  # If debug mode is enabled print out numbers

                        break

                    if (i == 10):
                        print("Failed to get position, tried 10 times!")

            self.clear_uart()
                
        except Exception as ex:
            print(ex)

    def pub_pos(self):
        print("Publishing pos on mosqitto")

    def sub_pos(self, topic):
        print("Subcribing pos on mosqitto", topic)
    
    def get_device_uptime(self):
        self.clear_uart()
        self.write_uart("ut\r", 0)
        trash = self.serialPort.readline()
        line = self.serialPort.readline()  # Reading incoming data from node
        line = line.strip()  # Taking \n and other symbols away
        if len(line) >= 15:  # If the data is approriate length
            data = line.split()
            uptime = data[data.index(b'uptime:') + 1]
            time = uptime.split(b':')
            hours = int(time[0])
            minutes = float(time[1])

            uptime_days = int(data[data.index(b'uptime:') + 2])

            self.write_debug("UPTIME: minutes: {}, hours: {}, days: {}".format(minutes, hours, uptime_days))  # If debug mode is enabled print out numbers

    def factory_reset_node(self):
        sure = input("Are you sure to factory reset?(y/n) ")
        if (sure == "y"):
            self.write_uart("frst\r", 0.1)
        else:
            print("Aborting factory reset")

    def get_stationary_conf(self):
        self.clear_uart()
        self.write_uart("scg\r", 0.1)

        trash = self.serialPort.readline()
        line = self.serialPort.readline()  # Reading incoming data from node
        line.strip()
        data = line.split(b'=')
        sensitivity = int(data[1])

        self.write_debug('Statinoary configuration: {}'.format(sensitivity))


    def set_stationary_conf(self, config_number):
        if (config_number == 0 or config_number == 1 or config_number == 2):
            self.clear_uart()
            self.write_uart("scs " + str(config_number) + "\r", 0.1)
        else:
            print("Not a valid stationary configuration, choose between 0, 1 and 2")
        
        trash = self.serialPort.readline() 
        line = self.serialPort.readline()  # Reading incoming data from node
        print(str(line[5:].strip()))
        self.clear_uart()


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



dwm = DWM(debug=1)

dwm.get_pos()
dwm.get_acc_data()
dwm.get_device_uptime()
dwm.set_stationary_conf(2)
dwm.get_stationary_conf()