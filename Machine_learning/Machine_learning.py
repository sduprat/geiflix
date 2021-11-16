from torch import cuda
from Fire_Dataset import TrainSet, TestSet
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


PATH = './cifar_net_V2.pth'
# set batch_size
batch_size = 256

# set number of workers
num_workers = 2

# load train data
trainset=TrainSet(transform=transform)
trainloader = torch.utils.data.DataLoader(trainset, batch_size=batch_size, shuffle=True, num_workers=num_workers)
# load test data
testset=TestSet(transform=transform)
testloader=torch.utils.data.DataLoader(testset, batch_size=batch_size, shuffle=True, num_workers=num_workers)

classes = ('Fire', 'No_Fire')
net=Fire_Net()
net.load_state_dict(torch.load(PATH))

criterion = nn.CrossEntropyLoss()
optimizer = optim.SGD(net.parameters(), lr=0.001, momentum=0.9)

def imshow(img):
  ''' function to show image '''
  img = img / 2 + 0.5 # unnormalize
  gbr = img.numpy() # convert to numpy objects
  plt.imshow(np.transpose(gbr/255, (1, 2, 0)))
  plt.show()


for epoch in range(2):  # loop over the dataset multiple times

    running_loss = 0.0
    for i, data in enumerate(trainloader, 0):
        # get the inputs; data is a list of [inputs, labels]
        inputs, labels = data
        labels=labels.flatten()
        # zero the parameter gradients
        optimizer.zero_grad()

        # forward + backward + optimize
        outputs = net(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

        # print statistics
        running_loss += loss.item()
        if i % 20 == 19:    # print every 2000 mini-batches
            print('[%d, %5d] loss: %.3f' %
                  (epoch + 1, i + 1,running_loss / 2000))
            running_loss = 0.0

torch.save(net.state_dict(), PATH)

# get random training images with iter function
dataiter = iter(testloader)
images, labels = dataiter.next()


""" print('GroundTruth: ', ' '.join('%s' % classes[labels[j]] for j in range(batch_size))) """

outputs = net(images)

_, predicted = torch.max(outputs, 1)

""" print('Predicted: ', ' '.join('%s' % classes[predicted[j]]
                              for j in range(batch_size))) """

Correct_answer= 0
False_positive=0
False_negative=0

for k in range (batch_size):

  if (classes[labels[k]] == "Fire" and classes[predicted[k]] == "No_Fire"):
    False_negative += 1
  elif (classes[labels[k]] == 'No_Fire' and classes[predicted[k]] == 'Fire'):
    False_positive+=1
  else:
    Correct_answer+=1

print('Accuracy = ',(Correct_answer/batch_size)*100,'%')
print('False positives = ',(False_positive/batch_size)*100,'%')
print('False negatives = ',(False_negative/batch_size)*100,'%')


# print images
#imshow(torchvision.utils.make_grid(images))
