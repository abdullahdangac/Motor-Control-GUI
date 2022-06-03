import sys
import serial
import serial.tools.list_ports
from PyQt5 import QtWidgets, QtCore


"""class SerialThread(QtCore.QThread):  # for data read from serial port 
    message = QtCore.pyqtSignal(str)
    def __init__(self,parent = None):
        super(SerialThread, self).__init__(parent)
        self.serialPort = serial.Serial()
        self.stopflag = False
        
    def stop(self):
        self.stopflag = True
        
    def run(self):
        while True:
            if (self.stopflag):
                self.stopflag = False
                break
            elif(self.serialPort.isOpen()): 
                try:                        
                    self.data = self.serialPort.readline()
                except:
                    print("ERROR\n")
                self.message.emit(str(self.data.decode()))"""


class SerialCom:
    def __init__(self):
        self.serialConnection = None
        
    def connect(self, portName, baudRate):
        try:
            if self.serialConnection is None:
                self.serialConnection = serial.Serial(portName, baudRate, timeout=4)
                print('Connected to ' + str(portName) + ' at ' + str(baudRate) + ' BAUD')
            else:
                if self.serialConnection.isOpen():
                    self.serialConnection.close()
                    print("Disconnected")
                    print("Try Again Now")
                else:
                    self.serialConnection.open()
                    print("Connected")
        except:
            print("Failed to connect with " + str(portName) + ' at ' + str(baudRate) + ' BAUD')
            
    def sendSerialData(self, data):
        self.serialConnection.write(data.encode())  # utf-8
        
    def disconnect(self):
        self.serialConnection.close()
        print("Disconnected")


class GUI:
    def __init__(self):
        self.serialReference = SerialCom()
        self.ui = QtWidgets.QApplication(sys.argv)
        self.window = QtWidgets.QWidget()
        self.init_window()
        
    def init_window(self):
        self.window.setWindowTitle("Motor Control Panel")
        self.window.setGeometry(450, 150, 600, 300)
        
        """
            Com Port
        """
        port_label = QtWidgets.QLabel(self.window)
        port_label.setText("Port")
        port_label.move(20, 22)
        
        self.port_ComboBox = QtWidgets.QComboBox(self.window)
        self.port_ComboBox.move(50, 20)
        ports = ["COM1", "COM2", "COM3", "COM4"]
        for i in ports:
            self.port_ComboBox.addItem(str(i))
            
        """
            BaudRate
        """
        baud_label = QtWidgets.QLabel(self.window)
        baud_label.setText("BaudRate")
        baud_label.move(147, 22)
        
        self.baud_ComboBox = QtWidgets.QComboBox(self.window)
        self.baud_ComboBox.move(210, 20)
        baud = ["300", "1200", "2400", "4800", "9600", "19200", "38400", "57600", "74880", "115200"]
        for i in baud:
            self.baud_ComboBox.addItem(i)  # baud dizisinin içerisindeki değerler eklendi.
        self.baud_ComboBox.setCurrentText(baud[4]) # pencere ilk açıldığında baudrate 9600 olsun.
        
        """
            Connect Info Label
        """
        self.isConnect_label = QtWidgets.QLabel(self.window)
        self.isConnect_label.setText('<font color=red>Disconnected</font>')
        self.isConnect_label.move(400, 22)
        
        """
            Connect Button
        """
        self.connect_button = QtWidgets.QPushButton(self.window)
        self.connect_button.setText("Connect")
        self.connect_button.move(55, 60)
        self.connect_button.clicked.connect(self.connect)
        
        """
            Disonnect Button
        """
        self.disconnect_button = QtWidgets.QPushButton(self.window)
        self.disconnect_button.setText("Disconnect")
        self.disconnect_button.move(155, 60)
        self.disconnect_button.clicked.connect(self.disconnect)
        
        """
            Kp
        """
        Kp_label = QtWidgets.QLabel(self.window)
        Kp_label.setText("Kp")
        Kp_label.move(20, 121)
        
        self.Kp_entry = QtWidgets.QLineEdit(self.window)
        self.Kp_entry.move(40, 120)
        
        setKp_button = QtWidgets.QPushButton(self.window)
        setKp_button.setText("Set")
        setKp_button.move(190, 117)
        setKp_button.clicked.connect(self.send_Kp)
        
        """ 
            Ki 
        """
        Ki_label = QtWidgets.QLabel(self.window)
        Ki_label.setText("Ki")
        Ki_label.move(20, 153)
        
        self.Ki_entry = QtWidgets.QLineEdit(self.window)
        self.Ki_entry.move(40, 150)
        
        setKp_button = QtWidgets.QPushButton(self.window)
        setKp_button.setText("Set")
        setKp_button.move(190, 147)
        setKp_button.clicked.connect(self.send_Ki)
        
        """
            Kd
        """
        Kd_label = QtWidgets.QLabel(self.window)
        Kd_label.setText("Kd")
        Kd_label.move(20, 185)
        
        self.Kd_entry = QtWidgets.QLineEdit(self.window)
        self.Kd_entry.move(40, 180)
        
        setKp_button = QtWidgets.QPushButton(self.window)
        setKp_button.setText("Set")
        setKp_button.move(190, 177)
        setKp_button.clicked.connect(self.send_Kd)
    
        """
            Start/Stop Button
        """
        startStop_button = QtWidgets.QPushButton(self.window)
        startStop_button.setText("Start/Stop")
        startStop_button.move(400, 250)
        startStop_button.clicked.connect(self.send_startStop)
        
        
        """
            Set Point (Speed)
        """
        setPoint_label = QtWidgets.QLabel(self.window)
        setPoint_label.setText("Set Motor Speed")
        setPoint_label.move(58, 230)
        
        self.setPoint_entry = QtWidgets.QLineEdit(self.window)
        self.setPoint_entry.move(40, 250)
    
        setPoint_button = QtWidgets.QPushButton(self.window)
        setPoint_button.setText("Set Speed")
        setPoint_button.move(190, 247)
        setPoint_button.clicked.connect(self.send_setPoint)
    
    def connect(self):
        self.serialReference.connect(self.port_ComboBox.currentText(), self.baud_ComboBox.currentText())
        
        if self.serialReference.serialConnection.isOpen():
            self.isConnect_label.setText('<font color=green>Connected</font>')
            self.connect_button.setEnabled(False)      
            self.port_ComboBox.setEnabled(False)
            self.baud_ComboBox.setEnabled(False)
        
    def disconnect(self):
        self.serialReference.disconnect()
        
        if self.serialReference.serialConnection.isOpen() == False:
                self.isConnect_label.setText('<font color=red>Disconnected</font>')
                self.connect_button.setEnabled(True)
                self.port_ComboBox.setEnabled(True)
                self.baud_ComboBox.setEnabled(True)
    
    def send_Kp(self):
        self.serialReference.sendSerialData('P' + self.Kp_entry.text() + '%')
        print("Set Kp")
        
    def send_Ki(self):
        self.serialReference.sendSerialData('I' + self.Ki_entry.text() + '%')
        print("Set Ki")
        
    def send_Kd(self):
        self.serialReference.sendSerialData('D' + self.Kd_entry.text() + '%')
        print("Set Kd")
        
    def send_setPoint(self):
        self.serialReference.sendSerialData('S' + self.setPoint_entry.text() + '%')
        print(self.setPoint_entry.text())
        print("Set Speed")
        
    def send_startStop(self):
        self.serialReference.sendSerialData('R')
        print("Start/Stop")
    

if __name__ == '__main__':
    gui = GUI()
    gui.window.show()
    sys.exit(gui.ui.exec_())

