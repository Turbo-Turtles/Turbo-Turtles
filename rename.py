import os

def rename():
    num = 1122
    path = "C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/images/"
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
    
    file.close()

def create_pos():
    path = "C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/positives/"
    file = open("pos_tunnel.txt", "w")

    for j in range(896):
        file.write("positives/" + str(j) + ".jpg 1 264 612 476 446\n")

    
def change_pospic():
    path = "C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/positives/tunnel/"
    
    for i in os.listdir(path):
        my_source = path + i
        os.rename(my_source, path + "tunnel_" + i)
    
    os.rename("positives/tunnel/tunnel_pos.lst", "positives/tunnel/pos.lst")

def change_poslst():
    file = open("positives/tunnel/pos.lst", "r")
    f = open ("positives/tunnel/p.lst", "w")

    for i in file.readlines():
        f.write("tunnel_" + i)

    f.close()
    file.close()







def test():
    path = "C:/Users/marvi/OneDrive/Dokumente/Studium/4. Semester/Robotik/Turbo-Turtles/positives/parking/"
    for i in os.listdir(path):
        newname = i.removesuffix(".jpg")
        os.rename(path + i, path + newname)
        










#rename()
create_neg()
#create_pos()
#change_pospic()
#change_poslst()
#test()