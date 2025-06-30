import serial
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

PORT = 'COM14'
BAUD = 115200
SAMPLES = 512
AVG_WINDOW = 100
RC = 0.000265

gyro_raw = deque(maxlen=SAMPLES)
gyro_filtered = deque(maxlen=SAMPLES)
gyro_avg = deque(maxlen=AVG_WINDOW)

filtered_val = 0.0
first = True

ser = serial.Serial(PORT, BAUD, timeout=1)

plt.ion()
fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(10, 6))

def apply_lowpass(new_val, dt):
    global filtered_val, first
    alpha = dt / (RC + dt)
    if first:
        filtered_val = new_val
        first = False
    else:
        filtered_val = alpha * new_val + (1 - alpha) * filtered_val
    return filtered_val

while True:
    try:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            continue

        try:
            gyro_str, dt_str = line.split(',')
            gyro_z = float(gyro_str)
            dt = float(dt_str)
        except ValueError:
            continue

        gyro_raw.append(gyro_z)
        filtered = apply_lowpass(gyro_z, dt)
        gyro_filtered.append(filtered)
        gyro_avg.append(gyro_z)

        ax_time.clear()
        ax_time.plot(gyro_raw, label='Raw')
        ax_time.plot(gyro_filtered, label='Filtered', linewidth=2)
        avg = sum(gyro_avg) / len(gyro_avg)
        ax_time.axhline(avg, color='red', linestyle='--', label=f'Avg: {avg:.4f}')
        ax_time.set_title('Gyro Data (Z-axis)')
        ax_time.legend()
        ax_time.set_ylabel('deg/s')

        if len(gyro_raw) == SAMPLES:
            sample_rate = SAMPLES / sum(list(dt for _ in gyro_raw))
            fft_vals = np.fft.fft(np.array(gyro_raw) - np.mean(gyro_raw))
            fft_freq = np.fft.fftfreq(SAMPLES, d=1/sample_rate)

            ax_freq.clear()
            ax_freq.plot(fft_freq[:SAMPLES//2], np.abs(fft_vals[:SAMPLES//2]))
            ax_freq.set_title(f'FFT (estimated fs = {sample_rate:.1f} Hz)')
            ax_freq.set_xlabel('Frequency (Hz)')
            ax_freq.set_ylabel('Amplitude')

        plt.pause(0.01)
        ser.reset_input_buffer()

    except KeyboardInterrupt:
        print("\nStopped.")
        break

ser.close()
