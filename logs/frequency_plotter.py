import matplotlib.pyplot as plt
import numpy as np
import csv

send_frequency_times = []
send_frequency_measurements = []

with open('/home/dvolpi/Source/DASH-Hardware-Interface/logs/send_frequency_log.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines:
        send_frequency_times.append(float(row[0]))
        send_frequency_measurements.append(float(row[1]))

initial_send_frequency_time = send_frequency_times[0]
for i in range(len(send_frequency_times)):
   send_frequency_times[i] = send_frequency_times[i] - initial_send_frequency_time

send_frequency_average = np.mean(send_frequency_measurements)
send_frequency_std = np.std(send_frequency_measurements)
print(f'Send Frequency Average: {send_frequency_average:.3f} Hz')
print(f'Send Frequency Standard Deviation: {send_frequency_std:.3f} Hz')

plt.figure(1)
plt.hist(send_frequency_measurements, bins=50, color='blue')
plt.xlabel('Send Frequency (Hz)')
plt.ylabel('Occurances')
plt.title('Send Frequency Log', fontsize = 20)

receive_frequency_times = []
receive_frequency_measurements = []

with open('/home/dvolpi/Source/DASH-Hardware-Interface/logs/receive_frequency_log.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines:
        receive_frequency_times.append(float(row[0]))
        receive_frequency_measurements.append(float(row[1]))

t_initial = receive_frequency_times[0]
for i in range(len(receive_frequency_times)):
   receive_frequency_times[i] = receive_frequency_times[i] - t_initial

receive_frequency_average = np.mean(receive_frequency_measurements)
receive_frequency_std = np.std(receive_frequency_measurements)
print(f'Receive Frequency Average: {receive_frequency_average:.3f} Hz')
print(f'Receive Frequency Standard Deviation: {receive_frequency_std:.3f} Hz')

plt.figure(2)
plt.hist(receive_frequency_measurements, bins=50, color='green')
plt.xlabel('Receive Frequency (Hz)')
plt.ylabel('Occurances')
plt.title('Receive Frequency Log', fontsize = 20)
plt.show()

