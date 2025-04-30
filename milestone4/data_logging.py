import serial
import csv
from datetime import datetime

# Set up serial port
ser = serial.Serial('COM3', 115200)  # Replace COM3 with your port
filename = f"pid_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time(ms)", "Pitch", "PID_Output"])  # header

    print(f"Logging to {filename}... Press Ctrl+C to stop.")

    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                row = line.split(',')
                writer.writerow(row)
                print(row)
    except KeyboardInterrupt:
        print("\nLogging stopped.")
