import matplotlib.pyplot as plt
import numpy as np
import csv

measurement_times = []
# Data for ODRV0
position_measurements_0 = []
velocity_measurements_0 = []
# Data for ODRV1
position_measurements_1 = []
velocity_measurements_1 = []
# Data for ODRV2
position_measurements_2 = []
velocity_measurements_2 = []
# Data for ODRV3
position_measurements_3 = []
velocity_measurements_3 = []

with open('/home/dvolpi/Source/DASH-Hardware-Test/DASH-Hardware-Interface/logs/position_measurement_log.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    next(lines)  # Skip header row
    for row in lines:
        measurement_times.append(float(row[0]))
        # ODRV0
        position_measurements_0.append(float(row[1]))
        velocity_measurements_0.append(float(row[2]))
        # ODRV1
        position_measurements_1.append(float(row[3]))
        velocity_measurements_1.append(float(row[4]))
        # ODRV2
        position_measurements_2.append(float(row[5]))
        velocity_measurements_2.append(float(row[6]))
        # ODRV3
        position_measurements_3.append(float(row[7]))
        velocity_measurements_3.append(float(row[8]))

initial_measurement_time = measurement_times[0]
for i in range(len(measurement_times)):
   measurement_times[i] = measurement_times[i] - initial_measurement_time

# Plot positions for all three motors
plt.figure(1, figsize=(10, 6))
plt.plot(measurement_times, position_measurements_0, color='blue', label='ODRV0', linewidth=1.5)
plt.plot(measurement_times, position_measurements_1, color='red', label='ODRV1', linewidth=1.5)
plt.plot(measurement_times, position_measurements_2, color='green', label='ODRV2', linewidth=1.5)
plt.plot(measurement_times, position_measurements_3, color='orange', label='ODRV3', linewidth=1.5)
plt.xlabel('Time (s)')
plt.ylabel('Position (rev)')
plt.title('Position Measurement Log - All Motors', fontsize=20)
plt.legend()
plt.grid(True, alpha=0.3)

# Plot velocities for all three motors
plt.figure(2, figsize=(10, 6))
plt.plot(measurement_times, velocity_measurements_0, color='blue', label='ODRV0', linewidth=1.5)
plt.plot(measurement_times, velocity_measurements_1, color='red', label='ODRV1', linewidth=1.5)
plt.plot(measurement_times, velocity_measurements_2, color='green', label='ODRV2', linewidth=1.5)
plt.plot(measurement_times, velocity_measurements_3, color='orange', label='ODRV3', linewidth=1.5)
plt.xlabel('Time (s)')
plt.ylabel('Velocity (rev/s)')
plt.title('Velocity Measurement Log - All Motors', fontsize=20)
plt.legend()
plt.grid(True, alpha=0.3)

plt.show()

    