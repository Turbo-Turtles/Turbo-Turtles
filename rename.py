import os

def rename():
    i = 19
    path = "/home/marvin/Desktop/Signs/5/"
    for filename in os.listdir(path):
        my_dest = "5_" + str(i) + ".jpg"
        my_source = path + filename
        my_des = path + my_dest
        os.rename(my_source, my_des)
        i += 1

rename()

def create_neg():
    path = "HAAR_cascade/myData/"
    file = open("neg.txt", "w")
    
    for i in range(9):    
        for img in os.listdir(path + str(i)):
            file.write("negative/" + img + "\n")