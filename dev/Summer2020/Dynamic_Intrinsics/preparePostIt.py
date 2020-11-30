# Last Modified: 7/23/2020 Changes made by Aaron
# Aaron added for personal import setup
from setupImports2 import setup_path
setup_path()

import os

path = 'C:/Users/aofeldman/Desktop/knownZexpanded/rightBackward'
subfolderPrefixes = 'AFround'
relFocusPrefix = 'AF'
numRounds = 19

for rnd in range(numRounds+1):
    foldername = path + '/' + subfolderPrefixes + str(rnd)
    
    try:
        oldName = foldername + '/ximea' + str(1) + '.tif'
        newName = path + '/ximea' + str(rnd) + '.tif'
        os.rename(oldName, newName)
    except FileNotFoundError:
        print('Unable to find ' + oldName)
        pass
    
    try:
        oldName = foldername + '/' + relFocusPrefix + str(rnd) + '.txt'
        newName = path + '/' + relFocusPrefix + str(rnd) + '.txt'
        os.rename(oldName, newName)
    except FileNotFoundError:
        print('Unable to find ' + oldName)
        pass