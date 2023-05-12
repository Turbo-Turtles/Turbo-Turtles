import urllib.request
import cv2
import numpy as np
import os
from PIL import Image




def normalize_images():
    num_img = 0
    for i in os.listdir("C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/neg_img/"):
        try:
            print(i)
            img = cv2.imread("C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/neg_img/" + str(num_img) + ".jpg", cv2.IMREAD_GRAYSCALE)
            resized_img = cv2.resize(img, (100,100))
            #cv2.imshow("img", resized_img)
            cv2.imwrite("C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/ng/" + str(num_img) + ".jpg", resized_img)
            #cv2.waitKey(0)
            num_img += 1

        except Exception as e:
            print(str(e))

def test():
    num = 0
    for i in os.listdir("C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/negatives/"):
        im = Image.open("C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/negatives/" + str(i))
        im.save("C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/neg_img/" + str(num) + ".jpg")
        num += 1
        
    #img = cv2.imread("C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/negatives/0.ppm")
    #cv2.imshow("img", img)
    #cv2.waitKey(0)

#test()
normalize_images()
