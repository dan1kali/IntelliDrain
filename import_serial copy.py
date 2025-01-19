import serial
import threading
import time

lock = threading.Lock()

def read_serial(ser, filename):
    try:
        with open(filename, 'w') as file:  # clear file before writing
            while True:
                if ser.in_waiting > 0:
                    data = ser.readline().decode('utf-8').strip()
                    if data:  # Ensure we're writing non-empty data
                        with lock:  # Ensure that only one thread writes to the file at a time
                            file.write(data + '\n')
                        print(f"Data written to {filename}: {data}")
                    else:
                        print(f"Empty data received from {ser.portstr}")
                else:
                    print(f"No data available from {ser.portstr}")
                time.sleep(0.1)  # To prevent high CPU usage from constant polling
    except Exception as e:
        print(f"Error in reading from {ser.portstr}: {e}")


# simple version
""" def read_serial(ser, filename, lock):
    try:
        with open(filename, 'w') as file:
            while True:
                if ser.in_waiting > 0:
                    data = ser.readline().decode('utf-8').strip()
                    if data:
                        with lock:
                            file.write(data + '\n')
                        print(f"Data written: {data}")
                time.sleep(0.1)
    except Exception as e:
        print(f"Error: {e}") """


# Set up the two serial connections
try:
    ser1 = serial.Serial('/dev/cu.usbmodem14201', 115200, timeout=1)
    ser2 = serial.Serial('/dev/cu.usbmodem14101', 57600, timeout=1)
    print("Serial ports opened successfully.")
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)

# Create two threads to read from each serial port
thread1 = threading.Thread(target=read_serial, args=(ser1, 'serial_1_weight.txt'))
thread2 = threading.Thread(target=read_serial, args=(ser2, 'serial_2_occlusion.txt'))

# Start both threads
thread1.start()
thread2.start()












# Optionally, wait for threads to finish (in this case, they run forever)
try:
    thread1.join()
    thread2.join()
except KeyboardInterrupt:
    print("Program interrupted, stopping threads...")
finally:
    # Close the serial connections if needed
    ser1.close()
    ser2.close()
    print("Serial ports closed.")
