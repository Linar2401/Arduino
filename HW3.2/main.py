import cv2
import numpy as np

def find():
    cap = cv2.VideoCapture(0)
    face_cascade = cv2.CascadeClassifier('data/haarcascade_frontalface_default.xml')
    while (cap.isOpened()):
        ret, frame = cap.read()
        k = cv2.waitKey(1)
        # остановка по клавишам ESC или q или Q
        if k == 27 or k == ord('q') or k == ord('Q'):
            break
        image = frame
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        print(len(faces))
    cap.release()
    cv2.destroyAllWindows()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    find()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
