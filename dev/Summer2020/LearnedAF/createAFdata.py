# Last Modified: 8/3/2020 Changes made by Aaron

import torch
import torchvision
import torchvision.transforms as transforms
import matplotlib.pyplot as plt
import cv2
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np

plt.close('all')

def searchForFocus(filename, substring):
    with open(filename, 'r') as file:
        data = file.read()
        location = data.find(substring)
        croppedStr = data[location+len(substring):]
        # Split at spaces and find first number
        for word in croppedStr.split(): # Split at spaces
            # Delete any commas    
            word = word.replace(',', "")
            try:
                focusPosition = int(word)
                return focusPosition
            except ValueError:
                continue
    file.close()

def createSample(foldername, subfolderPrefix, index):
    sampleFoldername = foldername + '/' + subfolderPrefix + str(index)
    
    # Load in image as [0,1] array
    image = cv2.imread(sampleFoldername + '/before' + str(index) + '.tif', 0) * 1 / 255.0
    
    # Shift it so is from [-1,1]
    image *= 2
    image -= 1
    X = torch.from_numpy(image)
    X = X.unsqueeze(0) # Add fake first dimension to specify 1-channel

    # Get the label
    beforeFocus = searchForFocus(sampleFoldername + '/focusInfo.txt', 'before focus: ')
    afterFocus = searchForFocus(sampleFoldername + '/focusInfo.txt', 'after focus: ')
    
    y = afterFocus - beforeFocus
    
    # Save the X and y
    torch.save(X, sampleFoldername + '/X')
    torch.save(y, sampleFoldername + '/y')
    
    return X, y

foldername = '/home/aofeldman/Desktop/AFdataCollection'
subfolderPrefix = 'sample'
numSamples = 200

for sample in range(100, numSamples):
    createSample(foldername, subfolderPrefix, sample)

    

