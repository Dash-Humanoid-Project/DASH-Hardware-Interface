import csv
import numpy as np

times = []
positions = []
with open('/home/dvolpi/Source/DASH-Hardware-Test/DASH-Hardware-Interface/logs/position_measurement_log.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    next(lines)  # Skip header
    for row in lines:
        times.append(float(row[0]))
        positions.append(float(row[1]))  # ODRV0 position

# Calculate timesteps
timesteps = np.diff(times)
unique_times, counts = np.unique(times, return_counts=True)

print(f"Total measurements: {len(times)}")
print(f"Unique timestamps: {len(unique_times)}")
print(f"Max samples per timestamp: {np.max(counts)}")
print(f"Average samples per unique timestamp: {np.mean(counts):.1f}")
print(f"\nNon-zero timesteps:")
nonzero_timesteps = timesteps[timesteps > 0]
if len(nonzero_timesteps) > 0:
    print(f"  Count: {len(nonzero_timesteps)}")
    print(f"  Average: {np.mean(nonzero_timesteps):.6f} seconds ({1/np.mean(nonzero_timesteps):.1f} Hz)")
    print(f"  Median: {np.median(nonzero_timesteps):.6f} seconds ({1/np.median(nonzero_timesteps):.1f} Hz)")

print(f"\nZero timesteps: {np.sum(timesteps == 0)}")

# Check if position is actually changing
unique_positions = len(np.unique(positions))
print(f"\nUnique position values: {unique_positions}")
print(f"Position range: {np.min(positions):.6f} to {np.max(positions):.6f}")
