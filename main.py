from rplidar import RPLidar
import keyboard
import cv2
import numpy as np
import serial
import time

lidar = RPLidar('/dev/ttyUSB1', timeout=1)
lidar.reset()

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

# Create serial object
ser = serial.Serial('/dev/ttyUSB0', 115200)  # replace with your serial port and baudrate

def display_scan(scan_data):
    # Create an empty image
    img = np.zeros((500, 500, 3), dtype=np.uint8)
    cv2.imshow('Scan', img)
    
    # Draw each point in the scan data
    for angle, distance in scan_data:
        if distance > 0:  # Ignore points with zero distance
            # Convert angle and distance to image coordinates
            distance = distance * 2
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
    time.sleep(2)

    # create a list to hold our readings
    scan_data = []

    # create a list to hold our readings
    scan_data = []

    start_collecting = False  # flag to indicate when to start collecting data

    while True:

        print('Collecting a scan...')
        for i, scan in enumerate(lidar.iter_scans(max_buf_meas=5000)):
            print(f'iter {i}')
            for (_, angle, distance) in scan:
                if angle == 0:  # When the first angle of 0 is encountered
                    start_collecting = True  # Set flag to True
                    scan_data = []  # Empty the list to remove previous readings

                if start_collecting:  # Only collect data when flag is True
                    scan_data.append((angle, distance))
                    print((angle, distance))

                if angle >= 350 and start_collecting:  # Stop collecting data after reaching 350 degrees
                    start_collecting = False
                    break

            if not start_collecting:  # Break the outer loop when data collection is complete
                break


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
        # with open('lidar_data.txt', 'a') as outfile:
        #     outfile.write(','.join(str(x) for x in scan_data))
        #     outfile.write(f',{label}\n')

        scan_data = []


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