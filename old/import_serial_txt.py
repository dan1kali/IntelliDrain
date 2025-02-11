

import serial

ser = serial.Serial('/dev/cu.usbmodem14201', 115200)

# Open a file to log the data
with open('/Users/macbook/Desktop/Arduino/serial_output.txt', 'w') as file:
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            file.write(data + '\n')
            print(data)

