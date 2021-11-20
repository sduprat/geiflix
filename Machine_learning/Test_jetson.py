from torch import cuda
from Fire_Dataset import TrainSet, TestSet, ImageSet
from Fire_NN import Fire_Net

import matplotlib.pyplot as plt # for plotting
import numpy as np # for transformation

import torch # PyTorch package
import torchvision # load datasets
import torchvision.transforms as transforms # transform data
import torch.nn as nn # basic building block for neural neteorks
import torch.nn.functional as F # import convolution functions like Relu
import torch.optim as optim # optimzer
import cv2

transform = transforms.Compose( # composing several transforms together
    [transforms.ToTensor(), # to tensor object
     transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))]) # mean = 0.5, std = 0.5

PATH = './cifar_net_V3.pth'
# set batch_size
batch_size = 1

# set number of workers
num_workers = 2

# load test data
testset=ImageSet(transform=transform)
testloader=torch.utils.data.DataLoader(testset, batch_size=batch_size, shuffle=True, num_workers=num_workers)

classes = ('Fire', 'No_Fire')
net=Fire_Net()
net.load_state_dict(torch.load(PATH))

#torch.save(net.state_dict(), PATH)

# get random training images with iter function
dataiter = iter(testloader)
images, labels = dataiter.next()

outputs = net(images)

_, predicted = torch.max(outputs, 1)

print(classes[predicted])