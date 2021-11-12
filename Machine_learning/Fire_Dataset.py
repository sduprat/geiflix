import glob
import cv2
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader

class TrainSet(Dataset):

    def __init__(self,transform=True):
        self.imgs_path = "TrainSet/"
        self.transform = transform
        file_list = glob.glob(self.imgs_path + "*")#Get the path to the differtn image's "classes"
        print(file_list)
        self.data=[]
        for class_path in file_list:
            class_name = class_path.split("/")[-1]
            for img_path in glob.glob(class_path + "/*.jpg"):#Remember to adapt this to the img extension in dataset
                self.data.append([img_path, class_name])
        #print(self.data)
        self.class_map = {"Fire" : 0, "No_Fire": 1}
        self.img_dim = (32, 32)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, index):
        img_path, class_name = self.data[index]
        img = cv2.imread(img_path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) #convert to bgr to show the image later with plt
        img = cv2.resize(img, self.img_dim)
        class_id = self.class_map[class_name]
        img_tensor = torch.from_numpy(img)
        img_tensor = img_tensor.permute(2, 0, 1)
        img_tensor = img_tensor.float()
        class_id = torch.tensor([class_id])
        return img_tensor, class_id

class TestSet(Dataset):

    def __init__(self,transform=True):
        self.imgs_path = "TestSet/"
        self.transform = transform
        file_list = glob.glob(self.imgs_path + "*")#Get the path to the differtn image's "classes"
        print(file_list)
        self.data=[]
        for class_path in file_list:
            class_name = class_path.split("/")[-1]
            for img_path in glob.glob(class_path + "/*.jpg"):#Remember to adapt this to the img extension in dataset
                self.data.append([img_path, class_name])
        #print(self.data)
        self.class_map = {"Fire" : 0, "No_Fire": 1}
        self.img_dim = (32, 32)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, index):
        img_path, class_name = self.data[index]
        img = cv2.imread(img_path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) #convert to bgr to show the image later with plt
        img = cv2.resize(img, self.img_dim)
        class_id = self.class_map[class_name]
        img_tensor = torch.from_numpy(img)
        img_tensor = img_tensor.permute(2, 0, 1)
        img_tensor = img_tensor.float()
        class_id = torch.tensor([class_id])
        return img_tensor, class_id