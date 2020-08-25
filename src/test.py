import cv2
import matplotlib.pyplot as plt

cap = cv2.VideoCapture(0)
ret, frame = cap.read()
if ret == False:
    print("Na")
else:
    plt.imshow(frame)

