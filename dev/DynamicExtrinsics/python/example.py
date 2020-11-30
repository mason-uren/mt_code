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
labels = ['top-left', 'top-right']
# labels = ['top-left']

# Don't use this one
# labels = ['bottom-left', 'bottom-right', 'top-left', 'top-right']

# This is a list of parameters whose values are known for one reason or another.
# pan_scale and tilt_scale should be 1, and the offsets should be 0.
skip_params = ['pan_scale', 'tilt_scale', 'rz_offset', 'z_offset']
# skip_params = ['rz_offset', 'z_offset']
# skip_params = ['pan_scale', 'tilt_scale']
# skip_params = []

# The model loaded here is used only as an initial guess
model_path = "cad_models/2019-09-04-cad-model.json"
#model = PanTiltModel.from_json(model_path)

params = PanTiltModel.guess_params()
#params[10] = 0.1
#params[11] = -0.1
#params[12] = -1.2
model = PanTiltModel(*params)
print(model.params)

#########################################################
# Important stuff here: this is how you train the model #
#########################################################
# Make sure you fit the dynamic part before the static part.
if retrain_model:
    # To use different datasets, you can do something like
    # datasets = [load_dataset('path/to/dataset.h5') for path in paths]
    
    datasets = load_four_pos_datasets(round=1, labels=labels)
                              
    # This function assumes that each dataset corresponds to a single positions
    # of the fiducial.
    print('Fitting dynamic model ...')
    fit_dynamic_params(model, *datasets,
                       skip_params=skip_params,
                       loss_fn=dynamic_exploded_loss)
                
    
    # TODO: use a different dataset for the static params.
    
    # If you wanted to use a different collection of datasets for the static
    # parameters you could load them here.
    # datasets = [load_dataset('path/to/dataset.h5') for path in paths]
    print('Fitting static model ...')
    fit_static_params(model, *datasets,
                      skip_params=skip_params,
                      loss_fn=static_exploded_loss)
    
    
    print(model.params)
    
    #model.to_json(model_path)

# # Here is another way you can train the model. Fits all the params at once.
# if retrain_model:
#     datasets = load_four_pos_datasets(round=1, labels=labels)
#     print('Fitting cad-model all-params ...')
#     fit_all_params(model, *datasets,
#                       skip_params=skip_params,
#                       loss_fn=static_exploded_loss)
#     print(model.params)
#     model.to_json(model_path)

from error_plots import *
datasets = load_four_pos_datasets(round=1, labels=labels)
error_vs_pan_tilt_heat(model, datasets, labels, 'training',
                       err_fn=static_exploded_norms)
               
datasets = load_four_pos_datasets(round=2, labels=labels)
error_vs_pan_tilt_heat(model, datasets, labels, 'validation',
                      err_fn=static_exploded_norms)




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
