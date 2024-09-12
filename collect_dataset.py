from datetime import datetime
import cv2
import serial
import csv

capture = cv2.VideoCapture(4)  # To use Realsense camera in RGB mode, set index to 4
target_size = (512, 256)
fps = 30

capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

video_writer_raw = cv2.VideoWriter('recorded_raw.mp4', cv2.VideoWriter_fourcc(*"MP4V"), fps, target_size)
video_writer_timestamp = cv2.VideoWriter('recorded_timestamp.mp4', cv2.VideoWriter_fourcc(*"MP4V"), fps, target_size)

if not capture.isOpened():
    print('[ERROR] Could not open video device')
    exit()

serial_in = serial.Serial(port='/dev/ttyACM0', baudrate=9600,
                          bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                          timeout=1)  # /dev/ttyACM0

csv_file = open('data_log.csv', mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['timestamp', 'serial_data'])

process_frame = True
serial_available = True

try:
    while True:
        if serial_in.in_waiting > 0:
            serial_available = True

        if serial_available:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # Timestamp with milliseconds
            serial_data = serial_in.readline().decode('ascii').strip().replace('\x00', '')

            if process_frame:
                _, frame = capture.read()

                if not _:
                    print('[ERROR] Could not read frame')
                    break

                resized_frame = cv2.resize(frame, target_size)
                timestamp_frame = resized_frame.copy()

                cv2.putText(timestamp_frame, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (255, 255, 255), 2,
                            cv2.LINE_AA)

                cv2.imshow('frame', timestamp_frame)

                video_writer_raw.write(resized_frame)
                video_writer_timestamp.write(timestamp_frame)
                csv_writer.writerow([timestamp, serial_data])

                print(f'{timestamp}: {serial_data}')

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            process_frame = not process_frame

except KeyboardInterrupt:
    print('\n[INFO] KeyboardInterrupt detected.')

finally:
    capture.release()
    video_writer_raw.release()
    video_writer_timestamp.release()
    serial_in.close()
    csv_file.close()
    cv2.destroyAllWindows()

    print('[INFO] Output saved.')
