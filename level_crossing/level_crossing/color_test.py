import cv2
import numpy as np

def video():
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        _, img = cap.read()

        # convert to hsv colorspace
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # lower bound and upper bound for Red color
        lower_bound = np.array([170, 75, 110])
        upper_bound = np.array([180, 255, 255])

        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        c1x = 1000
        c1y = 1000
        c2x = 1000
        c2y = 1000
        j = 0
        

        for i in contours:
            M = cv2.moments(i)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.drawContours(img, [i], -1, (0, 255, 0), 2)
                cv2.circle(img, (cx, cy), 2, (0, 255, 255), -1)
                cv2.putText(img, "center", (cx - 20, cy - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                if j == 0:
                    c1x = cx
                    c1y = cy
                if j == len(contours) - 1:
                    c2x = cx
                    c2y = cy
                j += 1
                

        cv2.line(img, (c1x, c1y), (c2x, c2y), (255, 127, 0), 3)
    
        
        cv2.imshow("img", img)
        key = cv2.waitKey(30)
        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break

def image():
    img = cv2.imread("level_crossing\level_crossing\level.png")

    # convert to hsv colorspace
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # lower bound and upper bound for Red color
    lower_bound = np.array([0, 40, 90])
    upper_bound = np.array([180, 255, 255])

    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    c1x = 0
    c1y = 0
    c2x = 0
    c2y = 0
    j = 0

    for i in contours:    
        M = cv2.moments(i)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.drawContours(img, [i], -1, (0, 255, 0), 2)
            cv2.circle(img, (cx, cy), 7, (255, 0, 0), -1)
            cv2.putText(img, "center", (cx - 20, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            if j == 0:
                c1x = cx
                c1y = cy
            if j == len(contours) - 1:
                c2x = cx
                c2y = cy
            j += 1

    print(c1x)
    print(c1y)
    print(c2x)
    print(c2y)

    cv2.line(img, (c1x, c1y), (c2x, c2y), (255, 127, 0), 3)

    cv2.imshow("img", img)
    key = cv2.waitKey(0)
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()


video()
# image()