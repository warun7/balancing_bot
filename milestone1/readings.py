import serial
import csv
import time

# Set the correct port and baud rate
SERIAL_PORT = "COM5"  # Change to your port (e.g., "/dev/ttyUSB0" for Linux)
BAUD_RATE = 115200
CSV_FILE = "mpu6050_angles_filter.csv"

# Open serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Allow time for Arduino to reset

# Open CSV file to store data
with open(CSV_FILE, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "Roll (°)", "Pitch (°)"])  # CSV header

    print("Recording data... Press Ctrl+C to stop.")

    try:
        while True:
            line = ser.readline().decode("utf-8").strip()  # Read and decode serial data
            if "Roll:" in line and "Pitch:" in line:  # Ensure valid data
                parts = line.replace("°", "").split("|")  # Remove degrees symbol
                roll = float(parts[0].split(":")[1].strip())  # Extract roll
                pitch = float(parts[1].split(":")[1].strip())  # Extract pitch
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")  # Current time
                
                writer.writerow([timestamp, roll, pitch])  # Save to CSV
                print(f"{timestamp} -> Roll: {roll:.2f}°, Pitch: {pitch:.2f}°")

    except KeyboardInterrupt:
        print("\nData recording stopped.")
        ser.close()
