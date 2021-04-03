import time

import cv2
import serial
import numpy as np

set_area = 20000
set_x = 160
cx = 0
cy = 0

kp = 2

data = 0
ser = 0

def pid(err):
    return kp * err


def find(path):
    ser = serial.Serial('/dev/ttyACM0', 115200)
    ser.flush()
    cap = cv2.VideoCapture(path)
    cap.set(3, 320)
    cap.set(4, 240)
    while cap.isOpened():
        ret, frame = cap.read()
        k = cv2.waitKey(1)
        # остановка по клавишам ESC или q или Q
        if k == 27 or k == ord('q') or k == ord('Q'):
            break
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_yellow = np.array([161, 155, 84])
        high_yellow = np.array([179,255,255])
        yellow_mask = cv2.inRange(hsv_image, low_yellow, high_yellow)
        contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #for contour in contours:
        #    area = cv2.contourArea(contour)
        #    if area > 200:
        #        cv2.drawContours(frame, contour, -1, (0, 255, 0), 2)
        #cv2.imshow('Color', frame)
        M1 = 0
        max_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000 and area > max_area:
                max_area = area
                M1 = contour
        M = cv2.moments(M1)
        cx = 0
        cy = 0
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        err1 = set_x - cx
        err2 = set_area - max_area
        if abs(err1) > 10:
            data = 2 * 1000 + err1 * 1+ 360
        else:
            data = 4 * 1000 + err2 * 0.001+200
        print(data)
        send_string = str(data)
        send_string += "\n"
        ser.write(send_string.encode('utf-8'))
        time.sleep(0.1)
        #cv2.imshow('Color', frame)
    cap.release()
    cv2.destroyAllWindows()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    find(0)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
