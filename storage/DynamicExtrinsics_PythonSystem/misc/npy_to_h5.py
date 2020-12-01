import numpy as np
import h5py

pan_tilt = []
ximea_ext = []
impx_ext = []
combined_ext = []

pan_tilt.append([[84.7678451538086,6.077699661254883]])
pan_tilt.append([[94.36139678955078,6.187049865722656]])

impx_ext.append(np.load("imperx.npy"))
impx_ext.append(np.load("imperx_one.npy"))

ximea_ext.append(np.load("ximea.npy"))
ximea_ext.append(np.load("ximea_one.npy"))

combined_ext.append(np.load("Ximea_Imperx.npy"))
combined_ext.append(np.load("Ximea_Imperx_one.npy"))

print(combined_ext[0])

print(combined_ext[1])


pan_tilt = np.asarray(pan_tilt)
impx_ext = np.asarray(impx_ext)
ximea_ext = np.asarray(ximea_ext)
combined_ext = np.asarray(combined_ext)


with h5py.File("Recomputed_Extrinsics_TV_High_Quality_Checked_by_Yang.h5",'w') as hf:
    hf.create_dataset("pan_tilt",data=pan_tilt)
    hf.create_dataset("ximea_ext",data=ximea_ext)
    hf.create_dataset("impx_ext",data=impx_ext)
    hf.create_dataset("combined_ext",data=combined_ext)