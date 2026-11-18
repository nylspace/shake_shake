
import serial
import time

arduino = serial.Serial("COM4", 9600, timeout=1)

time.sleep(2)

print("Connected to Arduino!")
print("Reading MPU6050 data...")

try:
    while True:
        data = arduino.readline().decode(
            errors="ignore"
        ).strip()

        if data:
            print("Received:", data)

except KeyboardInterrupt:
    print("Stopped.")

finally:
    arduino.close()
