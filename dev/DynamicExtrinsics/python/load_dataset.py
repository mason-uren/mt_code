
'''
Functions related to loading .h5 datasets provided by scott
'''

import numpy as np
import h5py
import cv2
from intrinsics import distortion, intrinsics
from board import board
from measure_model_error import get_chessboard_corners, get_imgpoints

# TODO: dataset class with named attributes for each member data
# For convenience of acces: `dataset.pan` vs `dataset["pan"]`

# TODO: consider adding a new item to each dataset which indicates the position
# of the fiducial. This way, a single dataset can contain measurements from
# two or more positions of the fiducial.

# At the end of `load_dataset`, the object return is a dictionary whose keys
# come from among those below, and whose values are numpy arrays with the
# shapes described in the comments.

REQUIRED = {
    'impx_ext', # [B, 4, 4] approximate pose of fiducial in imperx space
    'ximea_ext', # [B, 4, 4] approximate pose of fiducial in ximea space
    'pan_tilt' # [B, 2] pan and tilt angles of the PTU during each measurement
}

OPTIONAL = {
    'combined_ext', # Not used, don't care.
    'ximea_in_imperx', # Not used, don't care.
    'UV_corners_imperx', # [B, N, 2] pixel coordinates of chessboard corners
    'UV_corners_ximea', # [B, M, 2] pixel coordinates of chessboard corners
    'imperx_chids', # Used to fill undetected corners w/ nan. Thrown away after.
    'ximea_chids', # Used to fill undetected corners w/ nan. Thrown away after.
}

# While loading data, try to undistort these particular measurements.
UNDISTORTABLE = {
    ('imperx_chids', 'UV_corners_imperx'),
    ('ximea_chids', 'UV_corners_ximea')
}

def _undistort(imgpoints, cameraMtx, distCoeffs):
    '''Undistort the pixel coords in `imgpoints` using `camera`'s distortion.
    '''
    #dst = distortion[camera]
    #mtx = intrinsics[camera]
    dst = distCoeffs
    mtx = cameraMtx

    orig_shape = imgpoints.shape
    imgpoints = imgpoints.reshape(-1, 2)

    undistorted = cv2.undistortPoints(imgpoints, mtx, dst).squeeze()

    return undistorted.reshape(orig_shape)

def _fill_missing_ids(points, chids):
    '''Fill any corner we failed to detect with np.nan
    '''
    all_padded = []
    num_valid = []
    for img_points, img_chids in zip(points, chids):
        padded = np.full((board.chessboardCorners.shape[0], 2), np.nan)
        
        valid = img_chids >= 0
        img_points = img_points[valid]
        img_chids = img_chids[valid]
        
        padded[img_chids] = img_points
        
        num_valid.append(len(img_chids))

        all_padded.append(padded)

    return np.stack(all_padded), np.array(num_valid)


def load_dataset(filepath, strip_min_tilt=True, strip_max_pan=False, min_num_chesscorners=None,
                 pan_shift=0.0, intrinsics=intrinsics, distortion=distortion):
    '''Load measurements from the .h5 file at `filepath`
       strip_min_tilt: if True discard the data collected at the min tilt angle since these
                   data are suspected to have large error due to camera motion (PTU not yet
                   settled down; applicable only to data collected in 2019)
       strip_max_pan: if True discard the data collected at the max pan angle. Can be used
                   with data collected on 2020-07-16/07-023.
       min_num_chesscorners: if not None, is the lower limit for a (pan,tilt) data point to
                   be included in the model fitting. If None, every data points are used
                   if other criteria above are met.
       pan_shift:  default 0.0; add this amount (in degrees) to all pan angles of a dataset
                   found to be useful when applied to bottom-left/top-left datasets 
                   collected in 2020-07-16 & 07-23.
       intrinsics & distortion: by default use the ones loaded from  "intrinsics.py"
                   but can be overriden to make it easy to be used in different settings.
    '''
    if pan_shift != 0.0:
        print('Loading data from "{}" with pan_shift = {}'.format(filepath, pan_shift))
    
    f = h5py.File(filepath, 'r')
    
    # Make sure there isn't anything unexpected in the file
    bad_keys = set(f.keys()) - (REQUIRED | OPTIONAL)
    assert not bad_keys, 'Unexpected keys: {}'.format(bad_keys)
    
    # Dump dataset into this dict
    dataset = {}
    
    # These keys must be present in the file, they correspond to required data
    for key in REQUIRED:
        assert key in f, 'Required key is missing: {}'.format(key)
        dataset[key] = np.array(f[key]).squeeze()
        
    # Some datasets may not contain data corresponding to these keys
    for key in OPTIONAL:
        if key in f:
            dataset[key] = np.array(f[key]).squeeze()
            
    # Convert units and orientation from PTU defaults
    dataset['pan'] = np.radians(pan_shift + dataset['pan_tilt'][:,0])
    dataset['tilt'] = np.radians(dataset['pan_tilt'][:,1])

    # Rid of the outliers observed in the data
    if strip_min_tilt:
        tilts = dataset['pan_tilt'][:,1]
        min_tilt = min(tilts)
        keepers = tilts > min_tilt + 0.05
        
        for key, arr in dataset.items():
            dataset[key] = arr[keepers]

    if strip_max_pan:
        pans = dataset['pan_tilt'][:,0]
        max_pan = max(pans)
        keepers = pans < max_pan - 0.05
        
        for key, arr in dataset.items():
            dataset[key] = arr[keepers]


    for chids_key, uv_corners_key in UNDISTORTABLE:
        # Can't do anything if there isn't any data.
        if chids_key not in dataset:
            pass
        
        uv_corners = dataset[uv_corners_key]
        
        # Figure out if the camera is imperx or ximea
        camera = 'ximea' if 'ximea' in uv_corners_key else 'imperx'
        
        # Put the undistorted image coordinates into the dataset
        undist_uv = _undistort(uv_corners, intrinsics[camera], distortion[camera])
        chids = dataset.pop(chids_key)
        dataset[uv_corners_key], num_valids = _fill_missing_ids(undist_uv, chids)

        #print(f'{uv_corners_key}:\n', num_valids)
        # data that holds for each (pan,tilt) the number of detected chessboard corners
        dataset['num_valids_'+camera] = num_valids
    
    # filter by the min number of detected chessboard corners (Ximea only) in each pan-tilt position
    if min_num_chesscorners:
        keepers = dataset['num_valids_ximea'] >= min_num_chesscorners
        for key, arr in dataset.items():
            try:
                dataset[key] = arr[keepers]
            except:
                print('Skipping data for enforcing "min_num_chesscorners" with key: {}'.format(key))
                
    #import pdb
    #pdb.set_trace()

    # Count the number of measurements only after possibly removing some.
    dataset['n_records'] = len(dataset['pan_tilt'])
    

    return dataset
    
def sample_dataset(dataset, fraction=1.0):
    """Returns a random sample from the provided dataset"""
    
    n_samples = int(dataset['n_records'] * fraction)
    sampler = np.random.choice(dataset['n_records'], n_samples, replace=False)
    sampled_dataset = {
        'n_records': n_samples
    }
    
    for key, val in dataset.items():
        try:
            sampled_dataset[key] = val[sampler]
        except TypeError:
            # Skip things like n_records which need not be sampled
            pass
    
    return sampled_dataset
    
# If you need to load from some collection of datasets, you can muck with this.
def load_four_pos_datasets(*, round=1, labels):
    '''Loads datasets from a subset of the 4 positions collected by scott.
    
    round - 1 or 2 - each position has 2 datasets referred to as round 1 or 2
    labels - a subset of ['bottom-left', 'bottom-right', 'top-left', 'top-right']
    
    Example usage:
    
        >>> datasets = data_getter(round=2, labels=['top-left', 'top-right'])
        
    will load the second set of data collected with the fiducial elevated
    on the left and right sides of the desk.
    '''
    ds_paths = [
        'datasets/2019-08-14-{}-round-{}.h5'.format(pos, round)
        for pos in labels
    ]
    for path in ds_paths: print(path)
    datasets = [load_dataset(path) for path in ds_paths]
    return datasets
    
def fake_dataset(*, model, dataset):
    '''Modifies the given dataset to be consistent with the given model.'''
    
    # imperx doesn't move, so it should have the same reading at all times.
    dataset['impx_ext'][:] = np.median(dataset['impx_ext'], axis=0)
    
    # ximea's measured extrinsics should be precisely those corresponding to the
    # imperx extrinsics passed through the model transformation.
    dataset['ximea_ext'] = model.apply(pans=dataset['pan'],
                                       tilts=dataset['tilt'],
                                       world_exts=dataset['impx_ext'])
    
    impx_corners = get_chessboard_corners(dataset['impx_ext'])
    impx_uv_corners = get_imgpoints(impx_corners)
    dataset['UV_corners_imperx'] = impx_uv_corners
    
    ximea_corners = get_chessboard_corners(dataset['ximea_ext'])
    ximea_uv_corners = get_imgpoints(ximea_corners)
    dataset['UV_corners_ximea'] = ximea_uv_corners
    
    return dataset
    

if __name__ == '__main__':
    datasets = load_four_pos_datasets(round=1, labels=['top-left', 'top-right'])
    dataset = datasets[0]
    sampled_dataset = sample_dataset(dataset, fraction=0.5)
    
    '''
    Below, perform an experiment to verify that fake data can be used to recover
    a given model precisely.
    '''
    
    from extrinsics import PanTiltModel
    model_path = "cad_models/2019-09-04-cad-model.json"
    model = PanTiltModel.from_json(model_path)
    fake_datasets = [fake_dataset(model=model, dataset=ds) for ds in datasets]
    
    orig_params = model.params.copy()
    print(orig_params)
    
    # Now mess up the parameters of the model
    fudge_factor = 0.01
    model.params += fudge_factor
    model.params.pan_scale = 1
    model.params.tilt_scale = 1

    from extrinsics import *
    from measure_model_error import *
    
    for dataset in fake_datasets:
        print(static_exploded_loss(model, dataset))
    
    skip_params = ['pan_scale', 'tilt_scale']
    
    
    fit_dynamic_params(model, *fake_datasets,
                       skip_params=skip_params,
                       loss_fn=dynamic_exploded_loss)
                
    fit_static_params(model, *fake_datasets,
                      skip_params=skip_params,
                      loss_fn=static_exploded_loss)
    
    # fit_all_params(model, *fake_datasets,
    #                   skip_params=skip_params,
    #                   loss_fn=static_exploded_loss)
                      
    print(model.params)

    # If the loss functions above go to zero, we probably succeeded.
    # Just in case, verify that the correct params were recovered:
    print(model.params - orig_params)
