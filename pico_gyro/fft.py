import serial
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

PORT = 'COM14'
BAUD = 115200
SAMPLES = 512
WINDOW_SIZE = 100
SAMPLE_RATE = 8000


ser = serial.Serial(PORT, BAUD, timeout=1)
gyro_data = deque(maxlen=SAMPLES)
avg_data = deque(maxlen=WINDOW_SIZE)

plt.ion()
fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(10, 6))


while True:
    try:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            continue

        try:
            value = float(line)
        except ValueError:
            continue

        gyro_data.append(value)
        avg_data.append(value)


        ax_time.clear()
        ax_time.plot(list(gyro_data), label='Gyro Z')
        if len(avg_data) > 0:
            avg = sum(avg_data) / len(avg_data)
            ax_time.axhline(avg, color='red', linestyle='--', label=f'Moving Avg: {avg:.4f}')
        ax_time.set_title('Gyro Raw Data')
        ax_time.set_ylabel('deg/s')
        ax_time.legend()

        if len(gyro_data) == SAMPLES:
            fft_vals = np.fft.fft(np.array(gyro_data) - np.mean(gyro_data))
            fft_freq = np.fft.fftfreq(SAMPLES, d=1/SAMPLE_RATE)
            ax_freq.clear()
            ax_freq.plot(fft_freq[:SAMPLES//2], np.abs(fft_vals)[:SAMPLES//2])
            ax_freq.set_title('FFT of Gyro Data')
            ax_freq.set_xlabel('Frequency (Hz)')
            ax_freq.set_ylabel('Amplitude')

        plt.pause(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")
        break

ser.close()
