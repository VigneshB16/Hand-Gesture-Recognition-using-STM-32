import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from collections import deque
import csv
import time
import os
import threading

print(os.getcwd())

ser = serial.Serial('COM8', 115200, timeout=1)

# Use deque with a maximum length to keep the data size fixed
max_length = 100
accel_x = deque(maxlen=max_length)
accel_y = deque(maxlen=max_length)
accel_z = deque(maxlen=max_length)
gyro_x = deque(maxlen=max_length)
gyro_y = deque(maxlen=max_length)
gyro_z = deque(maxlen=max_length)
timestamp = deque(maxlen=max_length)

def read_serial():
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            try:
                parts = line.split(' ')
                ax, ay, az = float(parts[0]), float(parts[1]), float(parts[2])
                gx, gy, gz = float(parts[3]), float(parts[4]), float(parts[5])

                accel_x.append(ax)
                accel_y.append(ay)
                accel_z.append(az)
                gyro_x.append(gx)
                gyro_y.append(gy)
                gyro_z.append(gz)
                timestamp.append(time.time())
            except Exception as e:
                print(f"Error: {e}")

# Start the serial reading in a separate thread
serial_thread = threading.Thread(target=read_serial)
serial_thread.daemon = True
serial_thread.start()

def update_long_rod(frame):
    if accel_x and accel_y and accel_z and gyro_x and gyro_y and gyro_z:
        ax_val, ay_val, az_val = accel_x[-1], accel_y[-1], accel_z[-1]
        gx_val, gy_val, gz_val = gyro_x[-1], gyro_y[-1], gyro_z[-1]

        # Define the vertices of the long rod-shaped prism
        prism_vertices = np.array([[-0.01, -0.005, -0.5],
                            [-0.01, 0.005, -0.5],
                            [0.01, 0.005, -0.5],
                            [0.01, -0.005, -0.5],
                            [-0.01, -0.005, 0.5],
                            [-0.01, 0.005, 0.5],
                            [0.01, 0.005, 0.5],
                            [0.01, -0.005, 0.5]])
        # Define the edges of the long rod-shaped prism
        prism_edges = [[0, 1], [1, 2], [2, 3], [3, 0],
                       [4, 5], [5, 6], [6, 7], [7, 4],
                       [0, 4], [1, 5], [2, 6], [3, 7]]

        # Calculate rotation angles
        elevation = np.arctan2(ax_val, np.sqrt(ay_val**2 + az_val**2))
        azimuth = np.arctan2(ay_val, az_val)
        tilt = np.arctan2(np.sqrt(ay_val**2 + az_val**2), gx_val)

        # Apply rotation transformations
        rotation_matrix = np.array([[np.cos(azimuth), -np.sin(azimuth), 0],
                                    [np.sin(azimuth), np.cos(azimuth), 0],
                                    [0, 0, 1]])
        rotated_prism = np.dot(prism_vertices, rotation_matrix)

        rotation_matrix = np.array([[np.cos(elevation), 0, np.sin(elevation)],
                                    [0, 1, 0],
                                    [-np.sin(elevation), 0, np.cos(elevation)]])
        rotated_prism = np.dot(rotated_prism, rotation_matrix)

        rotation_matrix = np.array([[1, 0, 0],
                                    [0, np.cos(tilt), -np.sin(tilt)],
                                    [0, np.sin(tilt), np.cos(tilt)]])
        rotated_prism = np.dot(rotated_prism, rotation_matrix)

        ax.clear()
        for edge in prism_edges:
            ax.plot3D([rotated_prism[edge[0], 0], rotated_prism[edge[1], 0]],
                      [rotated_prism[edge[0], 1], rotated_prism[edge[1], 1]],
                      [rotated_prism[edge[0], 2], rotated_prism[edge[1], 2]], 'b')

def update_data(frame):
    ax1.cla()
    ax2.cla()
    ax1.plot(accel_x, label='Accel X')
    ax1.plot(accel_y, label='Accel Y')
    ax1.plot(accel_z, label='Accel Z')
    ax2.plot(gyro_x, label='Gyro X')
    ax2.plot(gyro_y, label='Gyro Y')
    ax2.plot(gyro_z, label='Gyro Z')
    ax1.legend(loc='upper right')
    ax2.legend(loc='upper right')
    ax1.set_title('Accelerometer Data')
    ax2.set_title('Gyroscope Data')

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ani_long_rod = FuncAnimation(fig, update_long_rod, interval=100)

fig2, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
ani_graph = FuncAnimation(fig2, update_data, interval=100)

plt.show()

ser.close()

csv_path = "C:/Users/Bedre'sAcer/OneDrive/Desktop/PythonSTM32Files/sensor_data.csv"
print(os.getcwd())

with open(csv_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Timestamp', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z'])
    for i in range(len(timestamp)):
        writer.writerow([timestamp[i], accel_x[i], accel_y[i], accel_z[i], gyro_x[i], gyro_y[i], gyro_z[i]])


