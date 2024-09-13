#!/usr/bin/env python3

import rospy, time, serial, numpy, math
from dynamic_reconfigure.server     import Server
from localizer_dwm1001.cfg          import DWM1001_Tune_SerialConfig
from localizer_dwm1001.msg          import Anchor
from localizer_dwm1001.msg          import Tag
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

class DWM1001_API_COMMANDS:
        DOUBLE_ENTER    = b'\r\r'   # ASCII char for double Enter
        SINGLE_ENTER    = b'\r'     # ASCII char for single Enter
        HELP            = b'?'      # Display help
        QUIT            = b'quit'   # Quit API shell mode
        GC              = b'gc'     # Clears GPIO pin
        GG              = b'gg'     # Reads GPIO pin level
        GS              = b'gs'     # Sets GPIO as output and sets its value
        GT              = b'gt'     # Toggle GPIO(must be an output)
        F               = b'f'      # Show free memory on the heap
        PS              = b'ps'     # Show info about running threads
        PMS             = b'pms'    # Show power managements tasks. IDL means that task is idle. USE means that task is allocated in the power management
        RESET           = b'reset'  # reset the dev board
        UT              = b'ut'     # Show device uptime
        FRST            = b'frst'   # Factory reset
        TWI             = b'twi'    # General purpose I2C/TWI read
        AID             = b'aid'    # Read ACC device ID
        AV              = b'av'     # Rad ACC values
        LES             = b'les'    # Show distances to ranging anchors and the position if location engine is enabled
        LEC             = b'lec'    # Show measurement and position in CSV format
        LEP             = b'lep'    # Show position in CSV format.Sending this command multiple times will turn on/off this functionality.
        SI              = b'si'     # System Info
        NMG             = b'nmg'    # Get node mode info
        NMO             = b'nmo'    # Enable passive offline option and resets the node
        NMP             = b'nmp'    # Enable active offline option and resets the node.
        NMA             = b'nma'    # Configures node to as anchor, active and reset the node.
        NMI             = b'nmi'    # Configures node to as anchor initiator, active and reset the node.
        NMT             = b'nmt'    # Configures node to as tag, active and reset the node
        NMTL            = b'nmtl'   # Configures node to as tag, active, low power and resets the node.
        BPC             = b'bpc'    # Toggle UWB bandwidth / tx power compensation.
        LA              = b'la'     # Show anchor list
        STG             = b'stg'    # Display statistics
        STC             = b'stc'    # Clears statistics
        TLV             = b'tlv'    # Parses given tlv frame, see section 4 for valid TLV commands
        AURS            = b'aurs'   # Set position update rate. See section 4.3.3 for more detail.
        AURG            = b'aurg'   # Get position update rate. See section 4.3.4 for more details
        APG             = b'apg'    # Get position of the node.See section 3.4.2 for more detail
        APS             = b'aps'    # Set position of the node.See section 3.4.2for more detail
        ACAS            = b'acas'   # Configures node as anchor with given options
        ACTS            = b'acts'   # Configures node as tag with given options


x1=y1=x2=y2=0

global tag1_tmp
tag1_tmp = Tag(0,0,0)
global tag1
tag1 = Tag(0,0,0)

global tag2_tmp
tag2_tmp = Tag(0,0,0)
global tag2
tag2 = Tag(0,0,0)
# initialize the node
rospy.init_node('Localizer_DWM1001', anonymous=False)

# initialize ros rate 10hz
rate = rospy.Rate(1)

serialReadLine = ""
# For dynamic configuration
dynamicConfig_OPEN_PORT = {"open_port": False}
dynamicConfig_CLOSE_PORT = {"close_port": False}
dynamicConfig_SERIAL_PORT = {"serial_port": ""}

# initialize serial port connections
serialPortDWM1001 = serial.Serial(
    port       = str(rospy.get_param('~serial_port_name1', '/dev/ttyUSB_UWB_LEFT')),
    baudrate   = int(rospy.get_param('~serial_baud_rate', 115200)),
    parity     = serial.PARITY_ODD,
    stopbits   = serial.STOPBITS_TWO,
    bytesize   = serial.SEVENBITS
)
serialPortDWM1002 = serial.Serial(
    # port       = str(rospy.get_param('~serial_port_name2', '/dev/ttyUSB_UWB_RIGHT')),
    port       = str(rospy.get_param('~serial_port_name2', '/dev/ttyACM1')),
    baudrate   = int(rospy.get_param('~serial_baud_rate', 115200)),
    parity     = serial.PARITY_ODD,
    stopbits   = serial.STOPBITS_TWO,
    bytesize   = serial.SEVENBITS
)

class dwm1001_localizer:
    def main(self):
        """
        Initialize port and dwm1001 api

        :param:

        :returns: nones
        """

        global serialReadLine1
        # global serialReadLine2
        global x1, y1, x2, y2
        
        #TODO implemnt functionality dynamic configuration
        #updateDynamicConfiguration_SERIALPORT()
        # close the serial port in case the previous run didn't closed it properly
        serialPortDWM1001.close()
        serialPortDWM1002.close()
        
        # sleep for one sec
        time.sleep(1)
        # open serial port
        serialPortDWM1001.open()
        serialPortDWM1002.open()

        # check if the serial port is opened
        if(serialPortDWM1001.isOpen() ):
            rospy.loginfo("Port1 opened: "+ str(serialPortDWM1001.name) )
            rospy.loginfo("Port2 opened: "+ str(serialPortDWM1002.name) )
            # start sending commands to the board so we can initialize the board
            self.initializeDWM1001API()
            self.initializeDWM1002API()
            # give some time to DWM1001 to wake up
            time.sleep(2)
            # send command lec, so we can get positions is CSV format
            serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
            serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            serialPortDWM1002.write(DWM1001_API_COMMANDS.LEC)
            serialPortDWM1002.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rospy.loginfo("Reading DWM10011 coordinates")
            rospy.loginfo("Reading DWM10012 coordinates")
        else:
            rospy.loginfo("Can't open port: "+ str(serialPortDWM1001.name))
            rospy.loginfo("Can't open port: "+ str(serialPortDWM1002.name))

        
        try:

            while not rospy.is_shutdown():
                # just read everything from serial port
                serialReadLine1 = serialPortDWM1001.read_until()
                serialReadLine2 = serialPortDWM1002.read_until()
                
                try:
                    self.pubblishCoordinatesIntoTopics1(self.splitByComma(serialReadLine1))
                    self.pubblishCoordinatesIntoTopics2(self.splitByComma(serialReadLine2))

                except IndexError:
                    rospy.loginfo("Found index error in the network array!DO SOMETHING!")



        except KeyboardInterrupt:
            rospy.loginfo("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
            serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            serialPortDWM1002.write(DWM1001_API_COMMANDS.RESET)
            serialPortDWM1002.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        finally:
            rospy.loginfo("Quitting, and sending reset command to dev board 1")
            rospy.loginfo("Quitting, and sending reset command to dev board 2")
            serialPortDWM1001.reset_input_buffer()
            serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            serialPortDWM1002.reset_input_buffer()
            serialPortDWM1002.write(DWM1001_API_COMMANDS.RESET)
            serialPortDWM1002.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rate.sleep()
            # if "reset".encode() in serialReadLine:

            serialPortDWM1001.close()
            serialPortDWM1002.close()
            if not serialPortDWM1001.isOpen():
                rospy.loginfo("succesfully closed port 1")
            if not serialPortDWM1002.isOpen():
                rospy.loginfo("succesfully closed port 2")
           
            
                

    def splitByComma(self, dataFromUSB):
        """
        Split network data such us coordinates of anchor and tag, by comma ','

        :param dataFromUSB:  Array from serial port containing all informations, tag xyz and anchor xyz

        :returns: arrayFromUSBFormatted

        """

        arrayFromUSBFormatted = [x.strip() for x in dataFromUSB.strip().split(','.encode())]

        return arrayFromUSBFormatted

    def pubblishCoordinatesIntoTopics1(self, networkDataArray):
        """
        Publish anchors and tag in topics using Tag and Anchor Object

        :param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz

        :returns: none

        """
        global x1,y1,tag1_tmp,tag1
        # loop trough the array given by the serial port
        for network1 in networkDataArray:

            if 'POS'.encode() in network1:

                # construct the object for the tag
                try:
                    tag1_tmp = tag1
                    tag1 = Tag(float(networkDataArray[networkDataArray.index(network1) + 1]),
                            float(networkDataArray[networkDataArray.index(network1) + 2]),
                            float(networkDataArray[networkDataArray.index(network1) + 3]),)
                    # tag1_tmp = tag1
                except:
                    rospy.loginfo("ERROR")
                    tag1 = tag1_tmp
                    # print('loi du lieu tag')
                # publish tag
                pub_anchor = rospy.Publisher('/dwm1001/tag1', Tag, queue_size=1)
                pub_anchor.publish(tag1)
                x1=tag1.x
                y1=tag1.y
                rospy.loginfo("Tag1: "
                              + " x: "
                              + str(tag1.x)
                              + " y: "
                              + str(tag1.y)
                              + " z: "
                              + str(tag1.z))
                

    def pubblishCoordinatesIntoTopics2(self, networkDataArray):
        """
        Publish anchors and tag in topics using Tag and Anchor Object

        :param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz

        :returns: none

        """
        global x2,y2,tag2_tmp,tag2
        # loop trough the array given by the serial port
        for network1 in networkDataArray:

            if 'POS'.encode() in network1:

                # construct the object for the tag
                try:
                    tag2_tmp = tag2
                    tag2 = Tag(float(networkDataArray[networkDataArray.index(network1) + 1]),
                            float(networkDataArray[networkDataArray.index(network1) + 2]),
                            float(networkDataArray[networkDataArray.index(network1) + 3]),)
                    
                except:
                    rospy.loginfo("ERROR")
                    tag2 = tag2_tmp
                    # print('loi du lieu tag')
                
                # publish tag
                pub_anchor = rospy.Publisher('/dwm1001/tag2', Tag, queue_size=1)
                pub_anchor.publish(tag2)
                x2=tag2.x
                y2=tag2.y
                rospy.loginfo("Tag2: "
                              + " x: "
                              + str(tag2.x)
                              + " y: "
                              + str(tag2.y)
                              + " z: "
                              + str(tag2.z))
                



    def updateDynamicConfiguration_SERIALPORT(self):

        """
        Update dynamic configuration of ROS

        :param:

        :returns: none

        """

        global dynamicConfig_SERIAL_PORT

        # intialize dynamic configuration
        dynamicConfigServer = Server(DWM1001_Tune_SerialConfig, self.callbackDynamicConfig)
        # set close port to true
        dynamicConfig_CLOSE_PORT.update({"close_port": True})
        # set the open port to false
        dynamicConfig_OPEN_PORT.update({"open_port" : False})
        # update the server
        dynamicConfigServer.update_configuration(dynamicConfig_OPEN_PORT)
        dynamicConfigServer.update_configuration(dynamicConfig_CLOSE_PORT)
        # update the server with opened port
        dynamicConfig_CLOSE_PORT.update({"close_port": False})
        # update the server with close port
        dynamicConfig_OPEN_PORT.update({"open_port": True})
        # update name of serial port in dynamic configuration
        dynamicConfig_SERIAL_PORT = {"serial_port": str(serialPortDWM1001.name)}
        # now update server configuration
        dynamicConfigServer.update_configuration(dynamicConfig_OPEN_PORT)
        dynamicConfigServer.update_configuration(dynamicConfig_OPEN_PORT)
        dynamicConfigServer.update_configuration(dynamicConfig_CLOSE_PORT)
        dynamicConfigServer.update_configuration(dynamicConfig_SERIAL_PORT)

    def initializeDWM1001API(self):
        """
        Initialize dwm1001 api, by sending sending bytes

        :param:

        :returns: none

        """
        # reset incase previuos run didn't close properly
        serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        time.sleep(0.2)
        serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        time.sleep(0.2)
        # send ENTER two times in order to access api
        serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half a second
        time.sleep(0.5)
        serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half second
        time.sleep(0.5)
        # send a third one - just in case
        serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

    def initializeDWM1002API(self):
        """
        Initialize dwm1001 api, by sending sending bytes

        :param:

        :returns: none

        """
        # reset incase previuos run didn't close properly
        serialPortDWM1002.write(DWM1001_API_COMMANDS.RESET)
        time.sleep(0.2)
        serialPortDWM1002.write(DWM1001_API_COMMANDS.RESET)
        time.sleep(0.2)
        # send ENTER two times in order to access api
        serialPortDWM1002.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half a second
        time.sleep(0.5)
        serialPortDWM1002.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half second
        time.sleep(0.5)
        # send a third one - just in case
        serialPortDWM1002.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
    def callbackDynamicConfig(self, config, level):
        """
        Map each button from dynamic configuration gui with specific action

        :param config:  array contains value of the gui

        :returns: config

        """
        global serialReadLine
        #rospy.loginfo("""Reconfigure Request: {dwm1001_network_info}, {open_port},\
        #      {serial_port}, {close_port}""".format(**config))

        if config["quit_dwm1001_api"]:
            rospy.loginfo("Not implement it yet")
            config["quit_dwm1001_api"] = False

        if config["close_port"]:
            rospy.loginfo("Close port not implement it yet")
            config["close_port"] = False

        if config["exit"]:
            rospy.loginfo("Not implement it yet")
            config["exit"] = False

        return config

def start():
    dwm1001 = dwm1001_localizer()
    dwm1001.main()


if __name__ == '__main__':
    try:
        start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

