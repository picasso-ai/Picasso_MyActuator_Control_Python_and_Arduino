#Setup
import PySimpleGUI as sg
import serial
from serial.tools import list_ports
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
matplotlib.use('TkAgg')
#Setup

#Variables
ports = []
ComPort = 'PLACEHOLDER' #Variable for GUI
#Variables

#Create Figures

    #M1 Torque Plot
fig_m1TorquePlot = matplotlib.figure.Figure(figsize=(3, 2.25), dpi=100)
t = np.arange(0, 3, .01)
fig_m1TorquePlot.add_subplot(111).plot(t, 2 * np.sin(2 * np.pi * t))
def draw_figure(canvas, figure):
   tkcanvas = FigureCanvasTkAgg(figure, canvas)
   tkcanvas.draw()
   tkcanvas.get_tk_widget().pack(side='top', fill='both', expand=1)
   return tkcanvas
    #M1 Torque Plot

    #M2 Torque Plot
fig_m2TorquePlot = matplotlib.figure.Figure(figsize=(3, 2.25), dpi=100)
t2 = np.arange(0, 3, .01)
fig_m1TorquePlot.add_subplot(111).plot(t2, 2 * np.sin(2 * np.pi * t2))
def draw_figure(canvas, figure):
   tkcanvas2 = FigureCanvasTkAgg(figure, canvas)
   tkcanvas2.draw()
   tkcanvas2.get_tk_widget().pack(side='top', fill='both', expand=1)
   return tkcanvas2
    #M2 Torque Plot

    #Position and Speed Plot
fig_PositionandSpeedPlot = matplotlib.figure.Figure(figsize=(4,1), dpi=100)
t3 = np.arange(0, 3, .01)
fig_PositionandSpeedPlot.add_subplot(111).plot(t3, 2*np.sin(2 * np.pi * t3))
def draw_figure(canvas, figure):
   tkcanvas3 = FigureCanvasTkAgg(figure, canvas)
   tkcanvas3.draw()
   tkcanvas3.get_tk_widget().pack(side='top', fill='both', expand=1)
   return tkcanvas3
    #Position and Speed Plot


#Create Figures

#Aesthetic Layout
setup_layout = [
    [sg.Text("ComPort: "), sg.Combo(values = ports, default_value = 'Hit Refresh', tooltip="USB to Serial Converter ComPort", k = '_dropdownkey_', size = (25, 25), auto_size_text = True), sg.Button("Refresh Port List"), sg.Button("Connect")], 
    [sg.Text("Filename: "), sg.InputText(default_text = 'Data/Default.csv', tooltip="Filename for Saving Data", k = "_filenamekey_")]
]

signal_layout_sub1 = [
    [sg.Radio("Sine", "RADIO1", default= True)],
    [sg.Radio("Sawtooth", "RADIO1", default= False)],
    [sg.Radio("Box", "RADIO1", default=False)],
    [sg.Text("Amplitude: "), sg.Input(default_text= '0.0', k = '_amplitudekey', size = (5,25))],
]

signal_layout_sub2 = [
    [sg.Radio("Custom", "RADIO1", default=False)],
    [sg.Text('Custom Signal: ')],
    [sg.Text("torque/current ="), sg.InputText(default_text = "10*t^3", tooltip="(Function of 't' (time))", size= (20,25))]
]

signal_layout_main = [
    [sg.Frame("Preset:", signal_layout_sub1), sg.Frame("Custom:",signal_layout_sub2)], 
    [sg.Button("START"), sg.Button("STOP")]
]

position_and_speed_plot_layout = [
    [sg.Text("Motor 1: "), sg.Checkbox('Position', default= False, k = '_m1positionkey_'), sg.Checkbox('Speed', default= False, k = '_m1speedkey_')],
    [sg.Text("Motor 2: "), sg.Checkbox('Position', default= False, k = '_m2positionkey_'), sg.Checkbox('Speed', default= False, k = '_m2speedkey_')],
    [sg.Canvas(k = '_PositionSpeedPlotKey_')]
]

left_layout = [
    [sg.Frame(title='Setup: ', layout= setup_layout)],
    [sg.Frame(title= 'Signal: ', layout= signal_layout_main)],
    [sg.Frame(title = 'Position and Speed Plot: ', layout = position_and_speed_plot_layout)]
]

right_layout = [
    [sg.Canvas(k = '_m1TorquePlotKey_')],
    [sg.VPush()],
    [sg.Canvas(k = '_m2TorquePlotKey_')],
]


layout = [
    [sg.Text("Hip Exo Gui Testing")], 
    [sg.Frame(title = None, layout=left_layout), sg.Frame(title= 'Torque Plots: ', layout= right_layout)],
    [sg.VPush()],
    [sg.Push(), sg.Button("Close")]
]

window = sg.Window("Hip Exo Gui", layout, size =(950,600), element_justification= 'c', finalize= True)
#Aesthetic Layout

#Window Functionality
while True:
    
    #Add Plots to Window
    tkcanvas = draw_figure(window['_m1TorquePlotKey_'].TKCanvas, fig_m1TorquePlot)
    tkcanvas2 = draw_figure(window['_m2TorquePlotKey_'].TKCanvas, fig_m2TorquePlot)
    tkcanvas3 = draw_figure(window['_PositionSpeedPlotKey_'].TKCanvas, fig_PositionandSpeedPlot)
    #Add Plots to Window

    event, values = window.read()
    
    if event =="Close" or event == sg.WIN_CLOSED:
        break
    elif event == "Connect":
        #NEED Code for translating portinfo object into port string and assigning to variable
        if ComPort == 'Something':
            comport_real = ComPort
            print("Serial Connection: Success")
        else:
            print("Serial Connection: Failure")

    elif event =="Refresh Port List":
        ports = list_ports.comports(include_links = False)
        #NEED: Code to convert portinfo obects to str of identifiers e.g. /dev/ttyUSB0
        
        length = len(ports)
        window['_dropdownkey_'].Update(value = ports)
        
        for i in range(1,length):
            print(str(ports[i]))

    elif event == 'START':
        break
    elif event == 'STOP':
        break
window.close()
#Window Functionality
