
# A tweaked version of example.py to generate ximea corners heat map (showing the # of ximea 
# corners detected), i.e., w/ the plot_corner_dist=True key to error_vs_pan_tilt_heat() function.
# by Yang Chen (ychen@hrl.com)
# Date: 2020-07-05

import numpy as np
from extrinsics import PanTiltModel, fit_dynamic_params, fit_static_params, fit_all_params
from load_dataset import load_four_pos_datasets
from measure_model_error import *

np.random.seed(1)

retrain_model = True

# These labels refer to the positions of the fiducial during the 2019-08-14
# datasets collected by scott. They are used when loading data and in the
# legends of the figures plotted at the end.
# labels = ["bottom-left"]
# labels = ["bottom-left", "bottom-right"]
#labels = ['top-left', 'top-right']
# labels = ['top-left']
labels = ["top-left", "top-right"]

# Don't use this one
# labels = ['bottom-left', 'bottom-right', 'top-left', 'top-right']

# The model loaded here is used only as an initial guess
model_path = "cad_models/2019-09-04-cad-model.json"
model = PanTiltModel.from_json(model_path)

print('Model to be evaluted: ')
print(model.params)

from error_plots import *
print('Loading dataset to be evaluated as "training" dataset:')
datasets = load_four_pos_datasets(round=1, labels=labels)
print('Dumping evaluation plots ...')
error_vs_pan_tilt_heat(model, datasets, labels, 'training',
                       err_fn=static_exploded_norms, plot_corner_dist=True)
               
print('Loading dataset to be evaluated as "validation" dataset:')
datasets = load_four_pos_datasets(round=2, labels=labels)
print('Dumping evaluation plots ...')
error_vs_pan_tilt_heat(model, datasets, labels, 'validation',
                      err_fn=static_exploded_norms, plot_corner_dist=True)



# # An online version of the calibration process would look like this:
# datasets = []
# while not converged(model):
#     datasets.append(collect_dataset())
#
#     fit_dynamic_params(model, *datasets,
#                        skip_params=skip_params,
#                        loss_fn=dynamic_exploded_loss)
#
#     fit_static_params(model, *datasets,
#                       skip_params=skip_params,
#                       loss_fn=static_exploded_loss)
