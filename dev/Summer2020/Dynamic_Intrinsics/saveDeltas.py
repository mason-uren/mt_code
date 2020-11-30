import pandas as pd
import numpy as np

deltaRlist = np.load('C:/Users/aofeldman/Desktop/deltaRlist.npy')
deltaTlist = np.load('C:/Users/aofeldman/Desktop/deltaTlist.npy')
meanErrList = np.load('C:/Users/aofeldman/Desktop/meanErrList.npy')
meanRerrList = np.load('C:/Users/aofeldman/Desktop/meanRerrList.npy')
meanTerrList = np.load('C:/Users/aofeldman/Desktop/meanTerrList.npy')
avgReproj = np.load('C:/Users/aofeldman/Desktop/avgReproj.npy')

# correspondingFocus = [588, 295, 368, 441, 493, 542]
correspondingFocus = [473, 428, 526, 563, 435, 521, 572, 292, 323, 372, 606, 420]
order = np.argsort(correspondingFocus)

# Use all the points
inliers = np.array(range(12), dtype=np.int64)[order]

orderedFocus = np.array(correspondingFocus)[inliers]

medianData = np.hstack([deltaRlist[:, 0, :], 1e3 * deltaTlist[:, 0, :], np.expand_dims(meanErrList[:, 0], axis=1)])
constantFitData = np.hstack([deltaRlist[:, -2, :], 1e3 * deltaTlist[:, -2, :], np.expand_dims(meanErrList[:, -2], axis=1)])
linearFitData = np.hstack([deltaRlist[:, -1, :], 1e3 * deltaTlist[:, -1, :], np.expand_dims(meanErrList[:, -1], axis=1)])

medianData = medianData[inliers]
constantFitData = constantFitData[inliers]
linearFitData = linearFitData[inliers]

joined = [0] * (len(medianData) + len(constantFitData) + len(linearFitData))
joined[0::3] = medianData
joined[1::3] = constantFitData
joined[2::3] = linearFitData
indexNames = ['Round ' + str(i // 3 + 1) + ' (' + str(orderedFocus[i // 3]) + ')' if i % 3 == 0 else '' for i in range(len(orderedFocus) * 3)]

columnNames = ['Delta Roll [deg]', 'Delta Pitch [deg]', 'Delta Yaw [deg]', 'Delta X [mm]', 'Delta Y [mm]', 'Delta Z [mm]', 'Reproj Error']
#firstCol = pd.DataFrame(['Median', 'Fit'] * (len(joined) // 2), index=correspondingFocus, columns={'Method'})
result =  pd.DataFrame(joined, index=indexNames, columns=columnNames)
result.insert(0, 'Method', ['Median', 'Constant Center Fit', 'Linear Center Fit'] * (len(joined) // 3))

#correctOrder = ['Method', 'Delta Roll [deg]', 'Delta Pitch [deg]', 'Delta Yaw [deg]', 'Delta X [mm]', 'Delta Y [mm]', 'Delta Z [mm]', 'Reproj Error']
#result[correctOrder]


indexNames = ['Median', 'Thin Lens', 'Constant Center Fit', 'Linear Center Fit']
meansJoined = np.hstack([meanRerrList, 1e3 * meanTerrList, np.expand_dims(avgReproj[1:], axis=1)])

meansResult = pd.DataFrame(meansJoined, index=indexNames, columns=columnNames)
meansResult.insert(0, 'Method', ['Mean'] * len(meansJoined))

mergedResult = result.append(meansResult)

mergedResult.to_excel('C:/Users/aofeldman/Desktop/comparingFit.xlsx') #, columns=correctOrder)
