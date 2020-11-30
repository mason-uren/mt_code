import numpy as np
from extrinsics import PanTiltModel, PanTiltModel_PanOffset, fit_dynamic_params, fit_static_params, fit_all_params
from load_dataset import load_dataset
from measure_model_error import *
import json
import pdb
import os

# Load instrinsics from 2020-06:
# from intrinsics2020_06 import distortion, intrinsics
from intrinsics2020_07 import distortion, intrinsics

np.random.seed(0)

retrain_model = False

# These labels refer to the positions of the fiducial during the 2019-08-14
# datasets collected by scott. They are used when loading data and in the
# legends of the figures plotted at the end.
# labels = ["bottom_left", "bottom_right"]
labels = ["bottom_left", "bottom_right"]
# labels =["top_left", "top_right"]
# labels=["bottom_right"]
# labels=["bottom_left"]
labels_validation = labels

# This is a list of parameters whose values are known for one reason or another.
# pan_scale and tilt_scale should be 1, and the offsets should be 0.
skip_params = ['pan_scale', 'tilt_scale', 'rz_offset', 'z_offset']
# skip_params = ['rz_offset', 'z_offset']
# skip_params = ['pan_scale', 'tilt_scale']
# skip_params = []

# dataset_unique_label="2020_10_01_ptu1"
dataset_unique_label = "2020_11_18"

# Load the data sets for the round we have
def load_datasets(labels, round=1, date_str=dataset_unique_label):
    ''' Load the data sets according to given labels, round and the date str.
    return the loaded data in a list, and a list of string dataset filenames + options used for loading
    the second returned list can be used as comments for print-out or saved with derived models (json)
    '''
    ds_paths = [
        # 'datasets/2020_07_16_{}_r{}.h5'.format(pos, round)
        'datasets/' + date_str + '_ptu0_rand11_{}_r{}.h5'.format(pos, round)
        for pos in labels
    ]
    print(os.getcwd())
    for path in ds_paths: print(path)
    strip_max_pan = False
    strip_min_tilt = False
    min_num_chesscorners = None  # 400
    pan_shift = 0.0
    datasets = [load_dataset(path, strip_max_pan=strip_max_pan, strip_min_tilt=strip_min_tilt,
                             min_num_chesscorners=min_num_chesscorners, pan_shift=pan_shift if "left" in path else 0.0,
                             intrinsics=intrinsics, distortion=distortion)
                for path in ds_paths]
    for dataset, label in zip(datasets, labels):
        # print(dataset.keys())
        dataset['label'] = label
        # print(dataset.keys())

    config = {'dataset_paths': ds_paths}
    config['strip_max_pan'] = strip_max_pan
    config['strip_min_tilt'] = strip_min_tilt
    config['left_pan_shift'] = pan_shift
    config['min_num_chesscorners'] = min_num_chesscorners

    return datasets, config


if __name__ == '__main__':

    # Both for saving the results and for loading to evaluate, depending on retrain_model
    # model_path = "cad_models/cpp-2020-08-04-cad-model.json"
    # model_path = "cad_models/{}-cad-model.json".format(dataset_unique_label)
    model_path = "cad_models/{}_rand11_cad_model.json".format(dataset_unique_label)

    model = {}
    if retrain_model:
        params = PanTiltModel.guess_params()
        # params[10] = 0.1
        # params[11] = -0.1
        # params[12] = -1.2
        model = PanTiltModel(*params)
    else:
        model = PanTiltModel.from_json(model_path)

    print('Initial model parameters: ')
    print(model.params)

    # Make sure you fit the dynamic part before the static part.
    if retrain_model:
        # To use different datasets, you can do something like
        # datasets = [load_dataset('path/to/dataset.h5') for path in paths]

        datasets, dataset_config = load_datasets(round=1, labels=labels)
        import pdb

        # pdb.set_trace()
        # This function assumes that each dataset corresponds to a single positions
        # of the fiducial.

        if (True):  # 2-step, dynamic-then-static
            print('Fitting dynamic model ...')
            _, poses = fit_dynamic_params(model, *datasets,
                                          skip_params=skip_params,
                                          # loss_fn=dynamic_reprojection_loss
                                          loss_fn=dynamic_exploded_loss
                                          )

            # TODO: use a different dataset for the static params.

            # If you wanted to use a different collection of datasets for the static
            # parameters you could load them here.
            # datasets = [load_dataset('path/to/dataset.h5') for path in paths]

            print('Fitting static model ...')
            fit_static_params(model, *datasets,
                              skip_params=skip_params,
                              # loss_fn=static_reprojection_loss
                              loss_fn=static_exploded_loss
                              )

        else:  # 1-step fitting
            print('Fitting all-params ...')
            fit_all_params(model, *datasets,
                           skip_params=skip_params,
                           loss_fn=static_exploded_loss)

        print('Found model parameters: ')
        print(model.params)

        print('Saving model parameters to:', model_path)
        model.to_json(model_path, comments=dataset_config)

        print('Saving board pose estimates ...')
        pose_data = {'poses': poses}
        pose_data['datasets'] = dataset_config  # dataset that produced the poses
        json.dump(pose_data, open("board_poses.json", "w"), indent=4)

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

    print('Creating error plots for training data ...')
    # labels_validation = ["bottom_right"]
    datasets, _ = load_datasets(round=1, labels=labels_validation)
    error_vs_pan_tilt_heat(model, datasets, labels_validation, '../../../src/DynamicExtrinsics/data/Charts/cpp_training',
                           err_fn=static_exploded_norms,
                           plot_corner_dist=False, interactive_3d=False)

    # print('Creating error plots for validation data ...')
    # datasets, _ = load_datasets(round=2, labels=labels_validation)
    # error_vs_pan_tilt_heat(model, datasets, labels_validation, '../../../src/DynamicExtrinsics/data/Charts/cpp_validation',
    #                      err_fn=static_exploded_norms,
    #                      plot_corner_dist=False, interactive_3d=False)

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
