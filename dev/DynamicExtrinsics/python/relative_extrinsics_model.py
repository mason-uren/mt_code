"""
This file provides a class which represents the pan-tilt camera model. The class
can be instantiated from json files, parameter arrays, or by fitting the model
to a dataset.
"""

import numpy as np
import torch
import pandas as pd
from matrices import rotation, translation, apply_pans, apply_tilts
from scipy.optimize import minimize
from measure_model_error import *
from tqdm import tqdm, trange
from torch_rotations import euler2mat

# TODO: Switch main code to use pytorch
# TODO: Try tensorflow for fun
# TODO: Switch to minimizing reprojection error rather than 6dof error
# TODO: initial guesses
# TODO: don't reapply the static part of the model?
# TODO: closure recomputes the gradient unnecessarily, thus slow.
# TODO: don't assume pan/tilt are given in radians.
# TODO: study correlations between pan/tilt and components of error.

class RelativeExtrinsicsModel(object):
    param_names = np.array([
        'yaw_ximea_tilt',   'pitch_ximea_tilt',     'roll_ximea_tilt',
        'x_ximea_tilt',     'y_ximea_tilt',         'z_ximea_tilt',
        'z_tilt_pan',
        'yaw_base_imperx',  'pitch_base_imperx',    'roll_base_imperx',
        'x_base_imperx',    'y_base_imperx',        'z_base_imperx'
    ])

    n_params = len(param_names)
    
    def __init__(self, **params):
        """
        Populate self.params with the values from **params.
        """
        # We should never receive keyword arguments with names other than those
        # expected as the names of model parameters.
        missing_names = set(self.param_names) - set(params.keys())
        if missing_names:
            raise TypeError("Missing keyword args: {}".format(missing_names))
        
        # Create a pandas series with one element for each parameter. The
        # default values for the parameters will be NaN unless they are passed
        # in as a keyword argument to **params.
        import pandas as pd
        self.params = pd.Series(index=self.param_names, dtype=float)
        for param_name, param_val in params.items():
            if param_name not in self.param_names: continue
            self.params[param_name] = param_val
        
        # TODO: is the optimization step going to be okay if we pass in a
        # series as the argument to be optimized? We'll see.
            
        self.r_ximea_tilt = rotation(roll=self.params.roll_ximea_tilt,
                                pitch=self.params.pitch_ximea_tilt,
                                yaw=self.params.yaw_ximea_tilt)
                                
        self.t_ximea_tilt = translation(x=self.params.x_ximea_tilt,
                                   y=self.params.y_ximea_tilt,
                                   z=self.params.z_ximea_tilt)
                                   
        self.t_tilt_pan = translation(x=0, y=0, z=self.params.z_tilt_pan)
                                                           
        self.t_base_imperx = translation(x=self.params.x_base_imperx,
                                    y=self.params.y_base_imperx,
                                    z=self.params.z_base_imperx)
                                   
        self.r_base_imperx = rotation(roll=self.params.roll_base_imperx,
                                 pitch=self.params.pitch_base_imperx,
                                 yaw=self.params.yaw_base_imperx)
        
    def show(self):
        angle_params = self.param_names[
            np.char.startswith(self.param_names, 'yaw_') |
            np.char.startswith(self.param_names, 'pitch_') |
            np.char.startswith(self.param_names, 'roll_')]
        dist_params = self.param_names[
            np.char.startswith(self.param_names, 'x_') |
            np.char.startswith(self.param_names, 'y_') |
            np.char.startswith(self.param_names, 'z_')]
        
        params = self.params.copy()
        params[angle_params] *= 180 / np.pi
        # params[dist_params] *= 1000
        
        print(params)
    
    def apply(self, *, pans, tilts, exts):
        """
        Returns the extrinsics corresponding to exts after undergoing the
        transformations described by the model using the corresponding pans and
        tilts given in the other variables.
        """
        # Transform Ximea -> Tilt
        exts = self.t_ximea_tilt @ self.r_ximea_tilt @ exts
        
        # Rotate about the Tilt axis
        exts = apply_tilts(tilts, exts)
        
        # Translate Tilt -> Pan
        exts = self.t_tilt_pan @ exts
        
        # Rotate about the Pan axis
        exts = apply_pans(pans, exts)
        
        # Transform Base -> Imperx
        return self.r_base_imperx @ self.t_base_imperx @ exts
        
    def get_relative_extrinsics(self, *, pan, tilt):
        """
        Given pan and tilt angles, returns the corresponding transformation to
        take poses from Ximea coordinates to Imperx coordinates.
        """
        
        r_tilt_pan = rotation(pitch=tilt)
        r_pan_base = rotation(yaw=pan)
        
        return self.r_base_imperx @ self.t_base_imperx @ r_pan_base @ self.t_tilt_pan @ r_tilt_pan @ self.t_ximea_tilt @ self.r_ximea_tilt
        
    @classmethod
    def from_array(cls, param_vals, **params):
        """
        TODO: docstring
        We'll need something like this to do the model fitting step
        """
        assert len(param_vals) + len(params) == cls.n_params, \
            "There should be {} parameters".format(cls.n_params)
        
        i = 0
        for key in cls.param_names:
            if key in params:
                continue
            
            params[key] = param_vals[i]
            i += 1
        
        return cls(**params)
    
    def to_array(self):
        """
        TODO: docstring
        """
        return np.array(self.params)

    @classmethod
    def from_json(cls, filepath):
        """
        Load 'cad_model' from json file, return an instance of
        RelativeExtrinsicsModel using the loaded parameters.
        """
        import json
        with open(filepath, 'r') as f:
            contents = json.load(f)
        
        # The model parameters are expected to be in an object with the name
        # of "cad_model"
        assert "cad_model" in contents, "No model found."
        
        # The model parameters should be in a dictionary with one entry per
        # parameter. The names must correspond exactly to param_names above,
        # while the values should be floating point numbers.
        return cls(**contents["cad_model"])
        
    def to_json(self, filepath, comments=None):
        json_dict = {
            'cad_model': {key: self.params[key] for key in self.param_names},
            'Comments': comments
        }
        import json
        with open(filepath, 'w') as outfile:
            json.dump(json_dict, outfile)
                
    @classmethod
    def from_dataset(cls, dataset, *,
                     x0=None,
                     **params):
        """
        TODO: docstring
        """
        def objective(x):
            model = cls.from_array(x, **params)
            est_exts = model.apply(pans=dataset['pan'],
                                   tilts=dataset['tilt'],
                                   exts=dataset['ximea_ext'])
            errs = est_exts - dataset['impx_ext']
            rot_err = np.linalg.norm(errs[:,0:3,0:3], axis=(1,2))
            tln_err = np.linalg.norm(errs[:,0:3,3], axis=1)
            return np.mean(tln_err) + 0.1 * np.mean(rot_err)
            
        # Initial guess for minimization routine.
        if x0 is None:
            # model = cls.from_json('cad_models/cad_model.json')
            x0 = np.zeros(cls.n_params)
        
        # Magic black box minimization algorithm courtesy of scipy.
        res = minimize(objective, x0)
        
        return cls.from_array(res.x, **params)
            
    @classmethod
    def from_dataset_pytorch(cls, dataset, *,
                             x0=None,
                             **params):
        # These parameters are used to configure the torch data structures
        tkwargs = {
            "dtype": torch.double,
            "device": torch.device("cpu")
        }
        
        # Top 3 rows of each 4x4 extrinsics matrix from Ximea perspective
        ximea_ext = dataset["ximea_ext"][:,0:3,:]
        ximea_poses = torch.tensor(ximea_ext, **tkwargs) # [B, 3, 4]
        
        # Top 3 rows of each 4x4 extrinsics matrix from Imperx perspective
        imperx_ext = dataset["impx_ext"][:,0:3,:]
        imperx_poses = torch.tensor(imperx_ext, **tkwargs) # [B, 3, 4]
        
        # Pan angles turned into 3x3 rotation matrices
        pans = torch.tensor(dataset["pan"], **tkwargs).reshape(-1,1)
        zeros = torch.zeros_like(pans, **tkwargs)
        pan_mats = euler2mat(torch.cat([zeros, pans, zeros], dim=1)) # [B, 3, 3]
        
        # Tilt angles turned into 3x3 rotation matrices
        tilts = torch.tensor(dataset["tilt"], **tkwargs).reshape(-1,1)
        zeros = torch.zeros_like(tilts, **tkwargs)
        tilt_mats = euler2mat(torch.cat([tilts, zeros, zeros], dim=1)) # [B, 3, 3]
        
        # These are the model parameters we will optimize over
        tln_ximea_tilt_vec = torch.zeros(3, **tkwargs, requires_grad=True)
        rot_ximea_tilt_vec = torch.zeros(1,3, **tkwargs, requires_grad=True)
        tln_tilt_pan = torch.zeros(1, **tkwargs, requires_grad=True)
        # rot_tilt_pan = torch.zeros(1, **tkwargs, requires_grad=True)
        tln_pan_imperx_vec = torch.zeros(3, **tkwargs, requires_grad=True)
        rot_pan_imperx_vec = torch.zeros(1,3, **tkwargs, requires_grad=True)
        
        # These parameters can be calibrated using only the PTU mounted camera
        dynamic_params = [
            tln_ximea_tilt_vec,
            rot_ximea_tilt_vec,
            tln_tilt_pan
        ]
        
        # These parameters require measurements taken from both cameras to calibrate
        static_params = [
            tln_pan_imperx_vec,
            rot_pan_imperx_vec
        ]
        
        def ximea_to_imperx(poses):
            """Change coordinates of the input poses from ximea to imperx perspective.
            
            poses: [B, 3, 4]
            """
        
            # Rotation from ximea to tilt
            rot_ximea_tilt_mat = torch.squeeze(euler2mat(rot_ximea_tilt_vec))
            poses = torch.matmul(rot_ximea_tilt_mat, poses)
            
            # Translation from ximea to tilt
            poses[:,:,3] += tln_ximea_tilt_vec
            
            # Rotation for each value of tilt
            poses = torch.bmm(tilt_mats, poses)
            
            # Translation offset between rotational axes of the PTU
            poses[:,2,3] += tln_tilt_pan
            
            # Rotation for each value of pan
            poses = torch.bmm(pan_mats, poses)
            
            # Translation from Base to Imperx
            poses[:,:,3] += tln_pan_imperx_vec
            
            # Rotation from Base to Imperx
            rot_pan_imperx_mat = torch.squeeze(euler2mat(rot_pan_imperx_vec))
            poses = torch.matmul(rot_pan_imperx_mat, poses)
            
            return poses
            
        def combined_err(pose_err, rot_scale=0.1):
            rot_err = torch.norm(pose_err[:,0:3,0:3], dim=(1,2))
            tln_err = torch.norm(pose_err[:,0:3,3], dim=1)
            return torch.mean(tln_err) + rot_scale * torch.mean(rot_err)
        
        def mad_closure():
            poses = ximea_to_imperx(ximea_poses)
            return combined_err(poses - torch.median(poses, dim=0)[0])
            
        def mae_closure():
            poses = ximea_to_imperx(ximea_poses)
            return combined_err(poses - imperx_poses)
            
        def grad_norm(params):
            norm = torch.tensor(0, **tkwargs)
            for param in params:
                norm += param.grad.abs().sum()
            return norm
                  
        def run_optimizer(params, loss_fn, lr=1e-1, gtol=1e-4, max_iter=500):
            def loss_closure():
                optimizer.zero_grad()
                loss = loss_fn()
                loss.backward()
                return loss
                
            optimizer = torch.optim.LBFGS(params, lr=lr)
            pbar = tqdm()
            for i in range(max_iter):
                optimizer.step(loss_closure)
                pbar.update()
                desc = "Loss: {:.10f}".format(loss_closure())
                desc += ", Grad Norm: {:.2e}".format(grad_norm(params))
                pbar.set_description(desc)
                
                if grad_norm(params) < gtol:
                    break
            pbar.close()
                
        run_optimizer(dynamic_params, mad_closure, gtol=5e-4)
        run_optimizer(static_params, mae_closure)
        
        def descramble_rot(rot):
            """translate from (rx, ry, rz) to (yaw, pitch, roll)"""
            return rot[[1, 0, 2]]
        
        all_params = np.concatenate([
            descramble_rot(rot_ximea_tilt_vec.detach().numpy().flat),
            tln_ximea_tilt_vec.detach().numpy().flat,
            tln_tilt_pan.detach().numpy().flat,
            descramble_rot(rot_pan_imperx_vec.detach().numpy().flat),
            tln_pan_imperx_vec.detach().numpy().flat
        ])
            
        return cls.from_array(all_params)

if __name__ == '__main__':
    model = RelativeExtrinsicsModel.from_json('cad_models/cad_model.json')
    print(model.get_relative_extrinsics(pan=0, tilt=0))
    
    rand_params = np.random.rand(RelativeExtrinsicsModel.n_params)
    model = RelativeExtrinsicsModel.from_array(rand_params)
    print(model.get_relative_extrinsics(pan=0, tilt=0))

    from load_dataset import load_dataset
    dataset_filepath = 'datasets/2019-07-11-pnp-measurements-x256.h5'
    dataset = load_dataset(dataset_filepath)
    
    error_func = lambda model, ds=dataset: pose_fast_mae(model, ds)
    model = RelativeExtrinsicsModel.from_dataset(dataset)
    model.show()
    model = RelativeExtrinsicsModel.from_dataset_pytorch(dataset)
    model.show()

    dataset_filepath = 'datasets/2019-07-15-pnp-measurements-x256.h5'
    dataset = load_dataset(dataset_filepath)
    from measure_model_error import litmus_test, pose_std, pose_rmse
    print("Standard Deviation of Predicted Pose")
    print(litmus_test(model, dataset, error_func=pose_std))
    print("MAD of Predicted Pose")
    print(litmus_test(model, dataset, error_func=pose_mad))
    
    model.to_json('cad_models/cad_model_mae.json')
    
