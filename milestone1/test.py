import pandas as pd
import matplotlib.pyplot as plt

# Load CSV data
csv_files = {
    "Accelerometer": "mpu6050_angles_accelerometer.csv",
    "Gyroscope": "mpu6050_angles_gyro.csv",
    "Complementary Filter": "mpu6050_angles_filter.csv"
}

data = {}

# Load data without timestamps
for method, file in csv_files.items():
    df = pd.read_csv(file, encoding="ISO-8859-1")  # or "latin1"
    df = df.reset_index()  # Use row index as x-axis
    data[method] = df

# Plot Roll and Pitch angles
plt.figure(figsize=(12, 6))

for method, df in data.items():
    plt.plot(df.index, df["Roll (°)"], label=f"{method} - Roll", linestyle='dashed')
    plt.plot(df.index, df["Pitch (°)"], label=f"{method} - Pitch")

plt.xlabel("Sample Number")  # Instead of timestamp
plt.ylabel("Angle (°)")
plt.title("MPU6050 Roll and Pitch Angles Over Time (Step Response)")
plt.legend()
plt.grid()
plt.show()