from datetime import time

import cv2
import serial
import numpy as np
setpoint = 160
cx = 0
cy = 0

kp = 2

data = 0
ser = 0

def setup():
    ser = serial.Serial('/dev/ttyACM0', 9600)

def pid(err):
    return kp*err

def find(path):
    cap = cv2.VideoCapture(path)
    cap.set(4, 240)
    while cap.isOpened():
        ret, frame = cap.read()
        k = cv2.waitKey(1)
        # остановка по клавишам ESC или q или Q
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY_INV)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # for contour in contours:
        #     area = cv2.contourArea(contour)
        #     if area > 800:
        #         cv2.drawContours(frame, contour, -1, (0, 255, 0), 2)
        max_area = 0
        M1
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200 and area > max_area:
                max_area = area
                M1 = contour
        M = cv2.moments(M1)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        err = setpoint - cx
        data = 3*1000 + pid(err)
        send_string = str(data)
        send_string += "\n"
        ser.write(send_string.encode('utf-8'))
        time.sleep(0.01)


        cv2.imshow('Color', frame)


    cap.release()
    cv2.destroyAllWindows()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    find(0)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
