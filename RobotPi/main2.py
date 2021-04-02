from datetime import time

import cv2
import serial
import numpy as np

set_area = 400
set_x = 160
cx = 0
cy = 0

kp = 2

data = 0
ser = 0


def setup():
    ser = serial.Serial('/dev/ttyACM0', 9600)


def pid(err):
    return kp * err


def find(path):
    cap = cv2.VideoCapture(path)
    cap.set(4, 240)
    while cap.isOpened():
        ret, frame = cap.read()
        k = cv2.waitKey(1)
        # остановка по клавишам ESC или q или Q
        if k == 27 or k == ord('q') or k == ord('Q'):
            break
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_yellow = np.array([20, 110, 110])
        high_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv_image, low_yellow, high_yellow)
        contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # for contour in contours:
        #     area = cv2.contourArea(contour)
        #     if area > 800:
        #         cv2.drawContours(frame, contour, -1, (0, 255, 0), 2)
        # cv2.imshow('Color', frame)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200 and area > max_area:
                max_area = area
                M1 = contour
        M = cv2.moments(M1)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        err1 = set_x - cx
        err2 = set_area - max_area
        if abs(err1) > 10:
            data = 2 * 1000 + err1 * 40 + 200
        else:
            data = 3 * 1000 + err2 * 40
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
