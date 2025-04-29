import serial
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# Configure serial connection
PORT = 'COM5'
BAUD_RATE = 115200

def read_serial_data(duration_seconds=10):
    """
    Read data from Arduino serial port for the specified duration
    and save to CSV file.
    """
    print(f"Opening serial port {PORT}...")

    # Open serial connection
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        print("Serial port opened successfully!")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return None

    data = []

    # reset arduino buffer
    print("Waiting for Arduino to initialize...")
    ser.reset_input_buffer()

    # Read and discard until we see "Send any character"
    while True:
        line = ser.readline().decode('utf-8').strip()
        print(line)
        if "Send any character" in line:
            break

    # Send any character to start test
    ser.write(b'a')
    print("Sent start signal, test will begin shortly...")

    # Read data
    print("Reading data...")
    start_time = None

    while True:
        try:
            line = ser.readline().decode('utf-8').strip()

            if not line:
                continue

            if "Test complete" in line:
                print("Test completed successfully!")
                break

            # Parse CSV data (skip header line)
            if "Time,Method,Roll,Pitch" not in line and "," in line:
                parts = line.split(',')
                if len(parts) == 4:
                    time_ms = float(parts[0])
                    method = parts[1]
                    roll = float(parts[2])
                    pitch = float(parts[3])

                    # Record the data
                    data.append([time_ms, method, roll, pitch])

                    # Print progress every 500ms
                    if start_time is None:
                        start_time = time_ms

                    elapsed = time_ms - start_time
                    if elapsed % 500 < 10:
                        print(f"Reading data... {elapsed/1000:.1f}s elapsed")
            else:
                print(line)

        except KeyboardInterrupt:
            print("Data collection stopped by user.")
            break
        except Exception as e:
            print(f"Error reading data: {e}")

    # Close serial connection
    ser.close()
    print("Serial port closed.")

    # Save data to CSV
    if data:
        with open('step_response_data2.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time(ms)', 'Method', 'Roll', 'Pitch'])
            writer.writerows(data)
        print("Data saved to 'step_response_data.csv'")
        return data
    else:
        print("No data collected!")
        return None

def plot_step_response(data):
    """
    Plot the step response for all three methods
    """
    if not data:
        print("No data to plot!")
        return

    # Convert to numpy arrays for easier processing
    data_np = np.array(data)
    time_ms = data_np[:, 0].astype(float)
    methods = data_np[:, 1]
    roll = data_np[:, 2].astype(float)
    pitch = data_np[:, 3].astype(float)

    # Create figure
    fig = plt.figure(figsize=(12, 10))
    gs = GridSpec(2, 1, figure=fig)

    # Roll plot
    ax1 = fig.add_subplot(gs[0, 0])

    # Filter data by method and plot
    for method, color, label in [('ACCEL', 'blue', 'Accelerometer'), 
                                ('GYRO', 'red', 'Gyroscope'), 
                                ('COMP', 'green', 'Complementary Filter')]:
        mask = methods == method
        ax1.plot(time_ms[mask], roll[mask], color=color, label=label)

    ax1.set_title('Roll Angle Step Response')
    ax1.set_xlabel('Time (ms)')
    ax1.set_ylabel('Roll Angle (degrees)')
    ax1.grid(True)
    ax1.legend()

    # Pitch plot
    ax2 = fig.add_subplot(gs[1, 0])

    # Filter data by method and plot
    for method, color, label in [('ACCEL', 'blue', 'Accelerometer'), 
                                ('GYRO', 'red', 'Gyroscope'), 
                                ('COMP', 'green', 'Complementary Filter')]:
        mask = methods == method
        ax2.plot(time_ms[mask], pitch[mask], color=color, label=label)

    ax2.set_title('Pitch Angle Step Response')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('Pitch Angle (degrees)')
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.savefig('step_response_plot2.png', dpi=300)
    print("Plot saved to 'step_response_plot.png'")
    plt.show()

def main():
    print("MPU6050 Step Response Analysis")
    print("==============================")
    print("This script will collect step response data from the Arduino")
    print("and plot the results for all three methods.")

    # Read data from Arduino
    data = read_serial_data()

    # Plot the results
    if data:
        plot_step_response(data)

if __name__ == "__main__":
    main()