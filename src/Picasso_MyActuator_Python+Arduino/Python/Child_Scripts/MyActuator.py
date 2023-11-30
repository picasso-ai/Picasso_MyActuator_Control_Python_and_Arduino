import numpy as np
import serial
import time


class MYACTUATOR(object):
    def __init__(self, ComPort) -> None:
       #Motor Variables -------
       self.ComPort = ComPort
       self.current_int16 = 0
       self.current = 0
       self.speed_int16 = 0xffff
       self.speed = 0
       self.position_int16 = 0xffff
       self.position = 0
       self.current_sent_int16 = 0xffff
       self.current_sent = 0
       self.current_sent_highbyte = 0xff
       self.current_sent_lowbyte = 0xff
       #Motor Variables -------
       
       #Serial Variables -------
       self.buffer = 0x00
       self.buffer_len = 0x00
       #Serial Variables -------

       #Serial Begin -------------
       #self.Serial_IMU = serial.Serial(ComPort, 230400, timeout=0.007, parity=serial.PARITY_NONE)
       self.Serial_Motor = serial.Serial(ComPort, 115200, timeout=0.007, parity=serial.PARITY_NONE)
        
       print('Serial Open Success')
       #Serial END---------------

    def ToUint(self, x, x_min, x_max, nbits):
        span = x_max - x_min

        if (x < x_min):
            x = x_min

        if (x > x_max):
            x = x_max
        toUint=((x - x_min) * ((float)((1 << nbits) - 1) / span))
        return toUint
        

    def ToFloat(self,x_int, x_min, x_max, nbits):
        span = x_max - x_min
        offset_value = x_min
        toFloat= x_int * span / float((((1 << nbits) - 1))) + offset_value
        return toFloat

    def read(self):
        self.buffer = self.Serial_Motor.read(9)
        self.buffer_len = len(self.buffer)

    def decode(self):
        if len(self.buffer)==9 and self.buffer[0]==0x30 and self.buffer[1]==0x31 :
            self.current_int16 =(self.buffer[2] <<8) | (self.buffer[3])
            self.current = self.ToFloat(self.current_int16,-2.6, 2.6,16)
            self.speed_int16 =(self.buffer[4] <<8 | (self.buffer[5]))
            self.speed = self.ToFloat(self.speed_int16, -25, 25, 16)
            self.position_int16 =(self.buffer[6] <<8 | (self.buffer[7]))
            self.position = self.ToFloat(self.position_int16, 0,360,16)

            #Debug Prints
            print("Position: " + str(round(self.position)))
            print("Speed: " + str(round(self.speed,2)))
            print("Current: " + str(round(self.current,2)))
            print()
            #Debug Prints

    def sendCurrent(self,current_sent): 
        self.current_sent_int16 = self.ToUint(self.current_sent, -5, 5,16)
        #self.current_sent_highbyte = 0x00 & (self.current_int16 >> 8)#SOMETHING function of torque_int16
        #self.current_sent_lowbyte = 0x00 & (self.current_int16 << 8)#SOMETHING function of torque_int16
        self.current_sent_highbyte = np.uint8(np.uint16(self.current_sent_int16) >> 8)
        self.current_sent_lowbyte = np.uint8(np.uint16(self.current_sent_int16) << 8)
        self.Serial_Motor.write(bytes([0xff,0xee,0xdd,self.current_sent_lowbyte,self.current_sent_highbyte,0xbb]))
        #print("Current Sent: " + str(current_sent))