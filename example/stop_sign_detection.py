import cv2

# Sign Cascade Classifier xml
left_sign = cv2.CascadeClassifier('example/signs/left_sign.xml')
parking_sign = cv2.CascadeClassifier('example/signs/parking_sign.xml')
right_sign = cv2.CascadeClassifier('example/signs/right_sign.xml')
stop_sign = cv2.CascadeClassifier('example/signs/stop_sign.xml')

cap = cv2.VideoCapture(0)

i = 1

while cap.isOpened():
    _, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    left_sign_scaled = left_sign.detectMultiScale(gray, 1.3, 5)
    parking_sign_scaled = parking_sign.detectMultiScale(gray, 1.3, 5)
    right_sign_scaled = right_sign.detectMultiScale(gray, 1.3, 5)
    stop_sign_scaled = stop_sign.detectMultiScale(gray, 1.3, 5)

    # Detect the stop sign, x,y = origin points, w = width, h = height
    for (x, y, w, h) in left_sign_scaled:
        # Draw rectangle around the sign
        left_sign_rectangle = cv2.rectangle(img, (x,y),
                                            (x+w, y+h),
                                            (0, 255, 0), 3)
        # Write "... sign" on the bottom of the rectangle
        left_sign_text = cv2.putText(img=left_sign_rectangle,
                                     text="Left Sign",
                                     org=(x, y+h+30),
                                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                     fontScale=1, color=(0, 0, 255),
                                     thickness=2, lineType=cv2.LINE_4)
    
    for (x, y, w, h) in stop_sign_scaled:
        # Draw rectangle around the sign
        stop_sign_rectangle = cv2.rectangle(img, (x,y),
                                            (x+w, y+h),
                                            (0, 255, 0), 3)
        # Write "... sign" on the bottom of the rectangle
        stop_sign_text = cv2.putText(img=stop_sign_rectangle,
                                     text="Stop Sign",
                                     org=(x, y+h+30),
                                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                     fontScale=1, color=(0, 0, 255),
                                     thickness=2, lineType=cv2.LINE_4)
    
    for (x, y, w, h) in parking_sign_scaled:
        # Draw rectangle around the sign
        parking_sign_rectangle = cv2.rectangle(img, (x,y),
                                            (x+w, y+h),
                                            (0, 255, 0), 3)
        # Write "... sign" on the bottom of the rectangle
        parking_sign_text = cv2.putText(img=parking_sign_rectangle,
                                     text="Parking Sign",
                                     org=(x, y+h+30),
                                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                     fontScale=1, color=(0, 0, 255),
                                     thickness=2, lineType=cv2.LINE_4)
        
    for (x, y, w, h) in right_sign_scaled:
        # Draw rectangle around the sign
        right_sign_rectangle = cv2.rectangle(img, (x,y),
                                            (x+w, y+h),
                                            (0, 255, 0), 3)
        # Write "... sign" on the bottom of the rectangle
        right_sign_text = cv2.putText(img=right_sign_rectangle,
                                     text="Right Sign",
                                     org=(x, y+h+30),
                                     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                     fontScale=1, color=(0, 0, 255),
                                     thickness=2, lineType=cv2.LINE_4)

    cv2.imshow("img", img)
    key = cv2.waitKey(30)
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
    if key == ord('k'):
        cv2.imwrite('example/image' + str(i) + '.jpg', img)