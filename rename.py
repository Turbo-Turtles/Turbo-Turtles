import os

def rename():
    num = 0
    path = "C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/positives/"
    #for j in os.listdir(path):
    for i in os.listdir(path):
        my_source = path + i
        os.rename(my_source, path + str(num) + ".jpg")
        num += 1

def create_neg():
    path = "C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/negatives/"
    file = open("neg.txt", "w")
    
    for j in os.listdir(path):
        file.write("negatives/" + str(j) + "\n")
        #for i in os.listdir(path + j):  
            
            #print("negative/" + j + "/" + i + "\n")

def create_pos():
    path = "C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/positives/"
    file = open("pos_stop.txt", "w")

    for j in range(896):
        file.write("positives/" + str(j) + ".jpg 1 264 612 476 446\n")

    

rename()
#create_neg()
#create_pos()