import cv2
import numpy as np


def find(path):
    cap = cv2.VideoCapture(path)
    import time
    timer = time.time()
    while cap.isOpened():
        delta = (time.time() - timer)
        timer = time.time()
        ret, frame = cap.read()
        if delta != 0:
            frame = cv2.putText(frame, str(int(1 / delta)), (20, 50), 0, 2, (255, 0, 0), 1)
        k = cv2.waitKey(1)
        # остановка по клавишам ESC или q или Q
        if k == 27 or k == ord('q') or k == ord('Q'):
            break
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low_yellow = np.array([20, 110, 110])
        high_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv_image, low_yellow, high_yellow)
        contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 800:
                cv2.drawContours(frame, contour, -1, (0, 255, 0), 2)
        cv2.imshow('Color', frame)

    cap.release()
    cv2.destroyAllWindows()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    find('color_ball.mp4')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
