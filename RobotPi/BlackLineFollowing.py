import time

import cv2
import serial
import numpy as np
setpoint = 100
cx = 0
cy = 0

kp = 2

data = 0
ser = 0

def send(ser, data):
    send_string = str(data)
    send_string += "\n"
    ser.write(send_string.encode('utf-8'))
    time.sleep(0.01)

def contsrt(a):
    result = int(a + 130*abs(a)/(a + 0.0001))
    if(abs(result) > 200):
        return int(200*abs(a)/(a + 0.0001))
    return result


def pid(err):
    return kp*err
    
def get_err(cap):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY_INV)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    max_area = 0
    M1 = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500 and area > max_area:
            max_area = area
            M1 = contour
    M = cv2.moments(M1)
    cx = 0
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
    err = setpoint - cx
    return err

def find(path):
    timer = time.time()
    ser = serial.Serial('/dev/ttyACM0', 115200)
    cap = cv2.VideoCapture(path)
    cap.set(3, 256)
    cap.set(4, 144)
    while time.time() - timer < 10:
        err = get_err(cap)
        print(err)
        if abs(err) > 10:
            send(ser,2000 + 129*int(err/abs(err)) + 200)
            time.sleep(0.005)
            send(ser,1000)
        else:
            send(ser,5350)
            time.sleep(0.015)
        send(ser,1000)
        #time.sleep(0.1)
        


    cap.release()
    cv2.destroyAllWindows()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    while True:
        find(0)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
