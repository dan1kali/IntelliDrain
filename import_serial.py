import serial

# Open the serial port that your Arduino is connected to (replace with your actual COM port)
ser = serial.Serial('/dev/cu.usbmodem14201', 115200)  # Adjust COM port accordingly

# Open a file to log the data
with open('serial_output.txt', 'w') as file:
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            file.write(data + '\n')
            print(data)  # Optional: to print to the console as well
