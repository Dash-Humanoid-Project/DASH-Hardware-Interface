import csv
import numpy as np

times = []
with open('/home/dvolpi/Source/DASH-Hardware-Test/DASH-Hardware-Interface/logs/position_measurement_log.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    next(lines)  # Skip header
    for row in lines:
        times.append(float(row[0]))

# Calculate timesteps
timesteps = np.diff(times)

print(f"Total measurements: {len(times)}")
print(f"Total time span: {times[-1] - times[0]:.3f} seconds")
print(f"Average timestep: {np.mean(timesteps):.6f} seconds ({1/np.mean(timesteps):.1f} Hz)")
print(f"Median timestep: {np.median(timesteps):.6f} seconds ({1/np.median(timesteps):.1f} Hz)")
print(f"Min timestep: {np.min(timesteps):.6f} seconds ({1/np.min(timesteps):.1f} Hz)")
print(f"Max timestep: {np.max(timesteps):.6f} seconds ({1/np.max(timesteps):.1f} Hz)")
print(f"Std dev: {np.std(timesteps):.6f} seconds")
