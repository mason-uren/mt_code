# Getting started
0. Install [miniconda](https://docs.conda.io/en/latest/miniconda.html)
    * During installation it will ask you where you want to install it. Tell it to install to somewhere on your shared drive if you don't want to have to reinstall on every new computer.
1. To create the necessary environment, move to the `python` directory and run `conda env create -f environment.yml`
    * By default this will create an environment called "relexts".
    * Enter the environment by running `conda activate relexts`
    * You can verify that you are in the correct environment by running `which python` and checking that the python installation is in your miniconda envs directory.
2. Try running `python example.py` to verify that your environment is correct.
3. The output of this script is 6 images. 3 representing training performance, 3 for validation performance. Their names are `...-cdf.pdf`, `...-dist.pdf`, and `...-heat.pdf`. These are cumulative and regular histograms of the error metric, and a scatter plot vs pan/tilt.

# Descriptions of files and directories

The files you should care about are:

* `environment.yml`
    * Lists the python package requirements to run all the code
* `example.py`
    * Example of loading data, fitting model, and plotting error.
    * The output of this script is 6 images. 3 representing training performance, 3 for validation performance. Their names are `...-cdf.pdf`, `...-dist.pdf`, and `...-heat.pdf`. These are cumulative and regular histograms of the error metric, and a scatter plot vs pan/tilt.
* `extrinsics.py`
    * Class representing the model, and functions for fitting the model `fit_dynamic_params` and `fit_static_params`.
    * This file replaces `relative_extrinsics_model.py` which is what Scott's code is currently using. We'll need to switch his code at some point.
* `load_dataset.py`
    * Loads data from .h5 files created by scott
    * If there were any one place where a mistake could cause the pixel based approach to perform poorly, it would be in `_undistort`
* You can probably safely ignore the files below here.
* `measure_model_error.py`
    * various loss functions to try minimizing for calibration.
* `error_plots.py`
    * Function for plotting error at each pan/tilt.
    * Add more as needed.
* `board.py`
    * Creates an object to represent the TV ChArUco board
* `intrinsics.py`
    * Loads the intrinsics and distortion coefficients for ximea and imperx
* `transforms.py`
    * Helper functions for geometric transformations
    * A lot of these might be unused.
    * TODO: stop doing everything manually and find a library that implements these properly for you.

Files that you don't need to worry about:

* `relative_extrinsics_model.py`
    * contains an old implementation of the model, and a pytorch implementation from when I thought that would be a good idea.
* `matrices.py`
    * Is superceded by `transforms.py` but this is the version that `relative_extrinsics_model.py` expects.

The folders you should care about are:

* `cad_model` where .json files describing models are saved
* `datasets` where the .h5 files from scott live

# Miscellaneous

* Some of the datasets whose names start with `2019-08-14` are not actually from that date. Not sure which. Check Scott's sent mail. Basically, the "bottom" and "top" datasets were collected on different days. Thus, when used to calibrate the same model the model ends up bad.
* If you start with a very bad initial guess model, the calibration process may not converge to a "good" calibration.
