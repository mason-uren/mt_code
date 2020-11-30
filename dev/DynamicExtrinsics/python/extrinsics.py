"""
This file provides a class which represents the pan-tilt camera model. The class
can be instantiated from json files, parameter arrays, or by fitting the model
to a dataset.
"""

# TODO: the model classes should not be responsible for fitting themselves
# TODO: Try tensorflow for fun
# TODO: study components of error (x,y,z)
# TODO: don't assume pan/tilt are given in radians.
# TODO: reduce code duplication (n_params, param_names, fit_..._params)

import json
import numpy as np
import pandas as pd
from tqdm import tqdm
from transforms import *
from scipy.optimize import minimize
from measure_model_error import *
from load_dataset import load_dataset


class ModelBase(object):
    # By default, models have no parameters.
    param_names = []
    n_params = 0
    
    @classmethod
    def __init_subclass__(cls, *, param_names, **kwargs):
        """Add params from the superclass to the subclass
        
        Example usage:
        
        >>> class Model1(ExtrinsicsModelBase, param_names=[1]):
        ...    pass

        >>> class Model2(Model1, param_names=[2]):
        ...    pass
            
        >>> Model1.param_names
        [1]
                
        >>> Model2.param_names
        [1, 2]
        """
        cls.param_names = cls.param_names + param_names
        cls.n_params += len(param_names)
        super().__init_subclass__(**kwargs)
    
    def __init__(self, *params):
        """Populate self.params with the values from params"""
        # Expect exactly the correct number of parameters
        assert len(params) == self.n_params
        self.params = pd.Series(index=self.param_names, data=params, dtype=float)

    def update(self, keys, values):
        self.params.update(pd.Series(values, index=keys))
    
    def apply(self, *, exts):
        """Return world poses of the extrinsics given in the dataset
        
        exts: [N, 4, 4]
        """
        
        # Base model applies no transformations, there is nothing to do.
        return exts
        
    @classmethod
    def guess_params(cls):
        return np.zeros(cls.n_params)

    @classmethod
    def from_json(cls, filepath, model_key="cad_model"):
        """Load 'cad_model' from json file into an instance of cls"""
        with open(filepath, 'r') as f:
            contents = json.load(f)
        
        model = contents[model_key]
        return cls(*map(model.get, cls.param_names))
        
    def to_json(self, filepath, comments=None):
        """Save model params into a .json file"""
        json_dict = {
            'cad_model': self.params.to_dict(),
            'Comments': comments
        }
        with open(filepath, 'w') as outfile:
            json.dump(json_dict, outfile, indent=4)


class PanTiltModel(ModelBase,
                   param_names=['x_camera',  'y_camera',  'z_camera',
                                'rx_camera', 'ry_camera', 'rz_camera',
                                'z_offset', 'rz_offset',
                                'pan_scale', 'tilt_scale',
                                'x_ptu',  'y_ptu',  'z_ptu',
                                'rx_ptu', 'ry_ptu', 'rz_ptu',]):
        
    @classmethod
    def guess_params(cls):
        x0 = super().guess_params()
        x0[cls.param_names.index('pan_scale')] = 1
        x0[cls.param_names.index('tilt_scale')] = 1
        x0[cls.param_names.index('z_offset')] = 0
        x0[cls.param_names.index('rz_offset')] = 0
        return x0
    
    def apply(self, *, pans, tilts, world_exts, pan_offset=None):
        """
        pans: [B] ndarray
        tilts: [B] ndarray
        world_exts: [B, 4, 4] ndarray
        ---
        returns: [B, 4, 4] ndarray
        ---
        Returns the extrinsics corresponding to exts after undergoing the
        transformations described by the model using the corresponding pans and
        tilts given in the other variables.
        """
        assert pans.shape == tilts.shape
        assert pans.shape[0] == world_exts.shape[0]
        
        exts = world_exts.copy()
        
        # 6dof transformation from ptu to imperx
        exts = apply_rigid(self.params['rx_ptu':'rz_ptu'],
                           self.params['x_ptu':'z_ptu'], exts)
        
        # Rotate about the Pan axis
        # Make sure to keep the negatives here.
        exts = apply_pans(self.params.pan_scale * pans, exts)
        
        # Translate and rotate between the tilt and pan axes
        exts = apply_zrigid(self.params.rz_offset, self.params.z_offset, exts)
        
        # Rotate about the Tilt axis
        # Make sure to keep the negatives here.
        exts = apply_tilts(self.params.tilt_scale * tilts, exts)
        
        # 6dof transformation from ximea to ptu
        local_exts = apply_rigid(self.params['rx_camera':'rz_camera'],
                                 self.params['x_camera':'z_camera'], exts)
        
        return local_exts
        
    def apply_inverse(self, *, pans, tilts, local_exts, pan_offset=None):
        '''TODO: docstring
        '''
        
        identities = np.repeat([np.eye(4)], len(pans), axis=0)
        rel_exts = self.apply(pans=pans, tilts=tilts, world_exts=identities)
        world_exts = np.linalg.inv(rel_exts) @ local_exts
        return world_exts
        
    # This is the function that Scott will use to apply the model.
    def get_relative_extrinsics(self, *, pan, tilt):
        '''Get the [4, 4] transformation matrix to go from Ximea to Imperx space
        '''
        # To get the matrix representing a transformation, apply the
        # transformation to an identity matrix.
        exts = np.eye(4)[np.newaxis, ...]
        pans = np.array([pan])
        tilts = np.array([tilt])
        backward = self.apply_inverse(local_exts=exts, pans=pans, tilts=tilts)
        return backward.squeeze()

class PanTiltModel_PanOffset(PanTiltModel, param_names=['pan_offset']):

    def apply(self, *, pans, tilts, world_exts, pan_offset=None):
        """
        pans: [B] ndarray
        tilts: [B] ndarray
        world_exts: [B, 4, 4] ndarray
        ---
        returns: [B, 4, 4] ndarray
        ---
        Returns the extrinsics corresponding to exts after undergoing the
        transformations described by the model using the corresponding pans and
        tilts given in the other variables.
        """
        assert pans.shape == tilts.shape
        assert pans.shape[0] == world_exts.shape[0]
        
        exts = world_exts.copy()
        
        # 6dof transformation from ptu to imperx
        exts = apply_rigid(self.params['rx_ptu':'rz_ptu'],
                           self.params['x_ptu':'z_ptu'], exts)
        
        # Rotate about the Pan axis
        # if pan_offset is not None, assume it's a float
        offset = self.params['pan_offset'] if pan_offset else 0.0
        #print(f'Applying pan_offset: {offset}')
        exts = apply_pans(self.params.pan_scale * pans + offset, exts)
        
        # Translate and rotate between the tilt and pan axes
        exts = apply_zrigid(self.params.rz_offset, self.params.z_offset, exts)
        
        # Rotate about the Tilt axis
        # Make sure to keep the negatives here.
        exts = apply_tilts(self.params.tilt_scale * tilts, exts)
        
        # 6dof transformation from ximea to ptu
        local_exts = apply_rigid(self.params['rx_camera':'rz_camera'],
                                 self.params['x_camera':'z_camera'], exts)
        
        return local_exts

        
    def apply_inverse(self, *, pans, tilts, local_exts, pan_offset=None):
        '''TODO: docstring
        '''
        
        identities = np.repeat([np.eye(4)], len(pans), axis=0)
        rel_exts = self.apply(pans=pans, tilts=tilts, world_exts=identities, pan_offset=pan_offset)
        world_exts = np.linalg.inv(rel_exts) @ local_exts
        return world_exts


def fit_dynamic_params(model, *datasets, skip_params=None, loss_fn):
    """Fit a model to the given datasets
       Return the model with parameters updated to the optimized, and
       a second value being a list of tuples of the poses of the board
       as (rvec,tvec).
    """
    skip_params = skip_params or {}
    
    params_to_fit = [pn for pn in model.param_names
                     if ('ptu' not in pn) and (pn not in skip_params)]
    
    print('Fitting dynamic params: ', params_to_fit)
    n_params = len(params_to_fit)
    
    #breakpoint()
    with tqdm() as pbar:
        def objective(x):
            '''
            Takes the parameters of interest concatenated with the 6dof of
            the fiducial for each dataset, outputs some error metric to minimize
            '''
            new_param_vals = x[:n_params]
            
            # Replace the old parameters with the new
            model.update(params_to_fit, new_param_vals)
            
            losses = []
            for i, dataset in enumerate(datasets):
                rvec = x[n_params+6*i: n_params+6*i+3]
                tvec = x[n_params+6*i+3: n_params+6*i+6]
                losses.append(loss_fn(model, rvec, tvec, dataset))
            
            # Average the loss across the datasets
            return np.mean(losses)
            
        def update_pbar(x):
            pbar.update()
            pbar.set_description("Loss: {:.20f}".format(objective(x)))
        
        more_params = []
        print('\nNumber of datasets (fiducial poses) loaded:', len(datasets))
        for i, dataset in enumerate(datasets):
            world_exts = model.apply_inverse(pans=dataset['pan'],
                                             tilts=dataset['tilt'],
                                             local_exts=dataset['ximea_ext'])
            rvecs = []
            tvecs = []
            print(f'Size of world_exts for dataset {i}: {world_exts.shape}')
            for ext in world_exts:
                rvecs.append(np.array(rvec_from_ext_mat(ext)))
                tvecs.append(tvec_from_ext_mat(ext))
            
            rvec = sum(rvecs) / len(rvecs)
            tvec = sum(tvecs) / len(tvecs)
            
            more_params.extend([rvec, tvec])
        
        # Tack on an initial guess for rvec, tvec
        x0 = np.concatenate([model.params.loc[params_to_fit], *more_params])
        print(f'Number of model params (n_params): {n_params}')
        print(f'Size of final parameter list (x0): {len(x0)}')
        
        # Magic black box minimization algorithm courtesy of scipy.
        res = minimize(objective, x0, callback=update_pbar)

    poses=[]
    for i, dataset in enumerate(datasets):
        rvec = res.x[n_params+6*i: n_params+6*i+3]
        tvec = res.x[n_params+6*i+3: n_params+6*i+6]
        print("Final values (rvec, tvec): ", rvec, tvec)
        poses.append((rvec.tolist(),tvec.tolist()))

    model.update(params_to_fit, res.x[:n_params])

    return model, poses


def fit_static_params(model, *datasets, skip_params=None, loss_fn):
    """Fit a model to the given datasets"""
    
    skip_params = skip_params or {}
    
    params_to_fit = [pn for pn in model.param_names
                     if ('ptu' in pn) and (pn not in skip_params)]
    
    print('Fitting static params: ', params_to_fit)

    with tqdm() as pbar:
        
        def objective(x):
            # Replace the old parameters with the new
            model.update(params_to_fit, x)
            
            losses = []
            for i, dataset in enumerate(datasets):
                losses.append(loss_fn(model, dataset))
            
            # Average the loss across the datasets
            return np.mean(losses)
            
        def update_pbar(x):
            pbar.update()
            pbar.set_description("Loss: {:.20f}".format(objective(x)))
        
        # Magic black box minimization algorithm courtesy of scipy.
        res = minimize(objective, model.params.loc[params_to_fit],
                       callback=update_pbar)

    model.update(params_to_fit, res.x)
    return model
    
def fit_all_params(model, *datasets, skip_params=None, loss_fn):
    """Fit a model to the given datasets"""
    
    skip_params = skip_params or {}
    
    params_to_fit = [pn for pn in model.param_names
                     if pn not in skip_params]
    
    with tqdm() as pbar:
        
        def objective(x):
            # Replace the old parameters with the new
            model.update(params_to_fit, x)
            
            losses = []
            for i, dataset in enumerate(datasets):
                losses.append(loss_fn(model, dataset))
            
            # Average the loss across the datasets
            return np.mean(losses)
            
        def update_pbar(x):
            pbar.update()
            pbar.set_description("Loss: {:.20f}".format(objective(x)))
        
        # Magic black box minimization algorithm courtesy of scipy.
        res = minimize(objective, model.params.loc[params_to_fit],
                       callback=update_pbar)

    model.update(params_to_fit, res.x)
    return model
    
    

# TODO: write a function for fitting all of the parameters at once.
