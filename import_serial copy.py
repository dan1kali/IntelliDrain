import serial
import threading

# Adjust serial port settings
ser1 = serial.Serial('/dev/cu.usbmodem14101', 57600)
ser2 = serial.Serial('/dev/cu.usbmodem14201', 115200)

# Function to handle reading from the first serial port (weight)
def read_serial_1():
    with open('serial_1_weight.txt', 'w') as file:
        while True:
            if ser1.in_waiting > 0:
                try:
                    data = ser1.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                        file.write(data + '\n')
                        print(f"Weight Data: {data}")
                except Exception as e:
                    print(f"Error reading from serial port 1: {e}")

# Function to handle reading from the second serial port (occlusion)
def read_serial_2():
    with open('serial_2_occlusion.txt', 'w') as file:
        while True:
            if ser2.in_waiting > 0:
                try:
                    data = ser2.readline().decode('utf-8', errors='ignore').strip()
                    if data:
                        file.write(data + '\n')
                        print(f"Occlusion Data: {data}")
                except Exception as e:
                    print(f"Error reading from serial port 2: {e}")

# Create threads for each serial read function
thread1 = threading.Thread(target=read_serial_1)
thread2 = threading.Thread(target=read_serial_2)

# Start the threads
thread1.start()
thread2.start()

# Keep the main thread alive to allow background threads to run
thread1.join()
thread2.join()
