import time

import cv2
import serial
import numpy as np

set_area = 4000
set_x = 100
cx = 0
cy = 0

kp = 2

data = 0
ser = 0
counter = 0

def send(ser, data):
    send_string = str(data)
    send_string += "\n"
    ser.write(send_string.encode('utf-8'))
    time.sleep(0.08)

def pid(err):
    return kp * err

def contsrt(a):
    result = int(a + 100*abs(a)/(a + 0.0001))
    if(abs(result) > 200):
        return int(200*abs(a)/(a + 0.0001))
    return result


def find(path):
    ser = serial.Serial('/dev/ttyACM1', 115200)
    ser.flush()
    cap = cv2.VideoCapture(0)
    cap.set(3, 256)
    cap.set(4, 144)
    
    while cap.isOpened():
        ret, frame = cap.read()
        
        #frame = cv2.resize(frame,(16,9))
        k = cv2.waitKey(1)
        # остановка по клавишам ESC или q или Q
        if k == 27 or k == ord('q') or k == ord('Q'):
            break
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_red = np.array([20, 100, 84])
        high_red = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv_image, low_red, high_red)
        contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        contours2 = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 250:
                #cv2.drawContours(frame, contour, -1, (0, 255, 0), 2)
                contours2.append(contour)
#         cv2.imshow('Color', frame)
        M1 = 0
        cx = 0
        cy = 0
        max_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500 and area > max_area:
                max_area = area
                M1 = contour
        
        M = cv2.moments(M1)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        err1 = set_x - cx
        err2 = set_area - max_area
        data = 1000
        if abs(err1) > 40:
            data = 2000 + contsrt(err1*0.5) + 200
        elif abs(err2) > 300:
            data = 4000 + contsrt(err2*0.001) + 200
        print(data)
        send(ser, data)
        send(ser, 1000)

    cap.release()
    cv2.destroyAllWindows()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    find(0)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/

