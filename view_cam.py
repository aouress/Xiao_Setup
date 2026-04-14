import serial
import numpy as np
import cv2

ser = serial.Serial('COM14', 115200)  # change port

while True:
    # Wait for frame start
    line = ser.readline()
    if b"FRAME_START" not in line:
        print("Couldn't find frame start")
        continue

    # Read raw image
    raw = ser.read(320 * 240)

    # Convert to image
    img = np.frombuffer(raw, dtype=np.uint8)
    img = img.reshape((240, 320))

    cv2.imshow("ESP32 Cam", img)
    cv2.waitKey(1)