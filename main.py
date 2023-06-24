import rplidar
import time
import keyboard
import cv2
import numpy as np
import serial

lidar = rplidar.RPLidar('/dev/ttyUSB1')  # replace with your device

# Create serial object
ser = serial.Serial('/dev/ttyUSB0', 115200)  # replace with your serial port and baudrate

def display_scan(scan_data):
    # Create an empty image
    img = np.zeros((500, 500, 3), dtype=np.uint8)

    # Draw each point in the scan data
    for angle, distance in enumerate(scan_data):
        if distance > 0:  # Ignore points with zero distance
            # Convert angle and distance to image coordinates
            polar_angle = np.radians(angle)
            x = int(250 + distance * np.cos(polar_angle) / 100)
            y = int(250 + distance * np.sin(polar_angle) / 100)
            
            # Draw the point
            cv2.circle(img, (x, y), 1, (0, 255, 0), -1)

    # Show the image
    cv2.imshow('Scan', img)
    cv2.waitKey(1)

try:
    print('Starting scan')
    lidar.start_motor()
    time.sleep(1)
    lidar.start_scan()

    # create a list to hold our readings
    scan_data = [0]*360

    ser.write('motor:speed:200\n'.encode())

    while True:
        print('Collecting a scan...')
        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                scan_data[min([359, int(angle)])] = distance
            break  # We break after one full revolution to get 360 readings

        display_scan(scan_data)

        # Check keyboard input for control
        if keyboard.is_pressed('w'):  # forward
            label = 'forward'
            ser.write('motor:forward\n'.encode())  # replace with your command
        elif keyboard.is_pressed('a'):  # left
            label = 'left'
            ser.write('motor:left\n'.encode())  # replace with your command
        elif keyboard.is_pressed('d'):  # right
            label = 'right'
            ser.write('motor:right\n'.encode())  # replace with your command
        elif keyboard.is_pressed('q'):  # quit
            break
        else:
            ser.write('motor:stop\n'.encode())  # replace with your command
            continue  # Skip if no valid key is pressed

        # Save the scan_data and label
        with open('lidar_data.txt', 'a') as outfile:
            outfile.write(','.join(str(x) for x in scan_data))
            outfile.write(f',{label}\n')

        scan_data = [0]*360

    lidar.stop()
    lidar.stop_motor()

except Exception as e:
    print(f'Error: {e}')

finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    cv2.destroyAllWindows()
    ser.close()  # Close the serial connection
