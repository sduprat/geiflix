import cv2
import os
import time
import glob

import torchvision.transforms as transforms # transform data


transform = transforms.Compose( # composing several transforms together
    [transforms.ToTensor(), # to tensor object
     transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))]) # mean = 0.5, std = 0.5

def Get_data(vid,Display=False) :
    global cap
    Img_Path="/home/corentin/Desktop/TSFlow/ImageSet/No_Fire"
    dim=(32,32)

    File_name="Frame_0.jpg"
    os.chdir(Img_Path)

    if not vid.isOpened():
        raise IOError("Cannot open webcam")

    #only display camera feed if asked to
    if Display:
        while True:
            ret, frame = vid.read()
            #frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
            cv2.imshow('Input', frame)

            c = cv2.waitKey(1)
            if c == 27:
                break
        
    _,Frame=vid.read()
    Frame = cv2.resize(Frame, dim,interpolation=cv2.INTER_AREA)
    cv2.imwrite(File_name,Frame)
