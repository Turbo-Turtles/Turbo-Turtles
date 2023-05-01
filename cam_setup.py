import cv2
from matplotlib import pyplot as plt

cap = cv2.VideoCapture(1)

def take_photo():
    cap = cv2.VideoCapture(1)
    ret, frame = cap.read()
    cv2.imwrite('photo.jpg', frame)
    cap.release()
    
while cap.isOpened():
	ret, frame = cap.read()
	cv2.imshow('Webcam', frame)
    
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break