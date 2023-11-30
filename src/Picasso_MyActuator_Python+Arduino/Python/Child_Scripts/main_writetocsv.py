import argparse
import MyActuator as MyActuator
import time
import numpy as np

# Parse command-line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--file_name', type=str, default='C:/Users/taylo/OneDrive/Documents/Desktop/GradSchool/Lab/tMotor Controllers/Data/tMotor.csv', help='Name of the CSV file')
parser.add_argument('--amplitude', type=float, default=1.0, help='Amplitude value')
parser.add_argumant('--ComPort', type=str, default='Com4', help='ComPort for serial converter')
args = parser.parse_args()
# Parse command-line arguments

#Motor Setup
motor = MyActuator.MYACTUATOR(args.ComPort)
#Motor Setup

#Time Control
now=0
t_pr1=0
Delta_T1= 0.1
start = time.time()
#Time Control

while(True):
    #Recieve Data
    motor.read();
    motor.decode();
    #Recieve Data

    now=(time.time()-start)

    if (now - t_pr1 > Delta_T1):
        t_pr1 = now

        #Send Current
        current = args.amplitude * np.sin(now)
        motor.sendCurrent(current)
        #Send Current

        # Store Data in CSV
        with open(args.file_name, 'a') as file:
            file.write(f'{now},{current},{motor.current}\n')
        # Store Data in CSV

        
  
    
