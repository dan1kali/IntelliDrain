import serial
import csv

ser = serial.Serial('/dev/cu.usbmodem14101', 115200)

# Open a CSV file to log the data
with open('/Users/macbook/Desktop/Arduino/serial_output.csv', 'w', newline='') as file:
    csv_writer = csv.writer(file)
    
    # Write the header for CSV (optional)
    csv_writer.writerow(['Time (s)', 'Fluid Force (mN)', 'Weight 1 (g)', 'Weight 2 (g)'])
    
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            
            # Check if the line contains only numeric data (by checking for commas and valid floats)
            try:
                values = [float(val) for val in data.split(',')]
                
                csv_writer.writerow(values)
                print(data)
            except:
                continue
