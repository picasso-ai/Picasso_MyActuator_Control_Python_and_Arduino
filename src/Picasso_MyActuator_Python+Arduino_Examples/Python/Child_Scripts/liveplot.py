import argparse
import time
import numpy as np
import matplotlib.pyplot as plt
import signal

# Parse command-line arguments
parser = argparse.ArgumentParser()
parser.add_argument('--file_name', type=str, default='data.csv', help='Name of the CSV file')
args = parser.parse_args()

# Create empty lists to store data
time_data = []
torque_data = []
measured_torque_data = []

# Create the figure and axes
fig, ax = plt.subplots()

def update_plot():
    # Read data from CSV
    data = np.genfromtxt(args.file_name, delimiter=',', skip_header=1)

    # Separate the columns
    time_data = data[:, 0]
    torque_data = data[:, 1]
    measured_torque_data = data[:, 2]

    # Update the plot
    ax.clear()
    ax.plot(time_data, torque_data, label='Variable Torque')
    ax.plot(time_data, measured_torque_data, label='Measured Torque')
    ax.set_xlabel('Time')
    ax.set_ylabel('Torque')
    ax.set_title('Variable Torque vs. Measured Torque')
    ax.legend()
    plt.pause(1.0)

# Catch the termination signal and export the final plot
def handle_termination(signum, frame):
    update_plot()
    exit(0)

signal.signal(signal.SIGINT, handle_termination)

# Update the plot every second
while True:
    update_plot()