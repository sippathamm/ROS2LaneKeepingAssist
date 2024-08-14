from datetime import datetime
import cv2
import serial
import csv

capture = cv2.VideoCapture(0)  # To use Realsense camera in RGB mode, set index to 4

capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

video_writer1 = cv2.VideoWriter('recorded_raw.mp4', cv2.VideoWriter_fourcc(*"MP4V"), 30, (512, 256))
video_writer2 = cv2.VideoWriter('recorded_timestamp.mp4', cv2.VideoWriter_fourcc(*"MP4V"), 30, (512, 256))

if not capture.isOpened():
    print('[ERROR] Could not open video device')
    exit()

serial = serial.Serial(port='/dev/tty.usbmodem2103', baudrate=9600,
                       bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                       timeout=1)  # /dev/ttyACM0

csv_file = open('data_log.csv', mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['timestamp', 'serial_data'])

process_frame = True
serial_available = True

try:
    while True:
        if serial.in_waiting > 0:
            serial_available = True

        if serial_available:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # Timestamp with milliseconds
            serial_data = serial.readline().decode('utf-8').strip()

            if process_frame:
                _, frame = capture.read()

                if not _:
                    print('[ERROR] Could not read frame')
                    break

                resized_frame = cv2.resize(frame, (512, 256))
                timestamp_frame = resized_frame.copy()

                cv2.putText(timestamp_frame, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 2,
                            cv2.LINE_AA)

                cv2.imshow('frame', timestamp_frame)

                video_writer1.write(resized_frame)
                video_writer2.write(timestamp_frame)
                csv_writer.writerow([timestamp, serial_data])

                print(f'{timestamp}: {serial_data}')

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            process_frame = not process_frame

except KeyboardInterrupt:
    print('\n[INFO] KeyboardInterrupt detected.')

finally:
    capture.release()
    video_writer1.release()
    video_writer2.release()
    serial.close()
    csv_file.close()
    cv2.destroyAllWindows()

    print('[INFO] Output saved.')
