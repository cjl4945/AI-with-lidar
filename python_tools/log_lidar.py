import serial
import time
import csv
from pathlib import Path

def log_lidar_data(port= 'COM5', baudrate=115200, duration_sec=60):
    filename = Path(f'data/raw/lidar_data_{time.strftime("%Y-%m-%d")}.csv')
    filename.parent.mkdir(parents=True, exist_ok=True)

    with serial.Serial(port, baudrate, timeout=1) as ser, open(filename, 'w', newline='') as csvfile:
        write = csv.writer(csvfile)
        write.writerow(['timestamp', 'distance', 'signal_strength'])

        start = time.time()
        while time.time() - start < duration_sec:
            line = ser.readline().decode('utf-8').strip()
            if line:
                try:
                    distance, strength = map(float, line.split(','))
                    write.writerow([time.time(), distance, strength])
                    print(f"{distance} cm, {strength}")
                except ValueError:
                    continue
if __name__ == "__main__":
    log_lidar_data()
