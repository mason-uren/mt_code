'''
Plot validation error vs number of samples used for training to determine
how many samples are really needed.
'''

import seaborn as sns; sns.set()
import matplotlib.pyplot as plt
import numpy as np
np.random.seed(0)

from extrinsics import PanTiltModel, fit_dynamic_params, fit_static_params
from load_dataset import load_four_pos_datasets, sample_dataset
from measure_model_error import *

labels = ['top-left', 'top-right']
training_datasets = load_four_pos_datasets(round=1, labels=labels)
validation_datasets = load_four_pos_datasets(round=2, labels=labels)

# Initial guess model
model_path = "cad_models/2019-09-04-cad-model.json"
model = PanTiltModel.from_json(model_path)

# Parameters to ignore (use ground truth for these).
skip_params = ['pan_scale', 'tilt_scale', 'rz_offset', 'z_offset']

n_trials = 10
fractions = [0.02, 0.05, 0.075, 0.1, 0.15, 0.2]
data = []
sample_counts = []

for fraction in fractions:
    for trial in range(n_trials):
        sampled_training_datasets = [sample_dataset(ds, fraction=fraction)
                                     for ds in training_datasets]
        sample_count = sum(ds['n_records'] for ds in sampled_training_datasets)

        fit_dynamic_params(model, *sampled_training_datasets,
                           skip_params=skip_params,
                           loss_fn=dynamic_exploded_loss)

        fit_static_params(model, *sampled_training_datasets,
                          skip_params=skip_params,
                          loss_fn=static_exploded_loss)

        losses = []
        for dataset in validation_datasets:
            losses.append(static_exploded_loss(model, dataset))

        data.append((fraction, sample_count, np.mean(losses)))

df = pd.DataFrame(data, columns=['fraction', 'n_samples','error'])
df.to_csv('data-vs-samples.csv')
sns.lineplot(x="n_samples", y="error", data=df)
plt.xlabel('Samples')
plt.ylabel('Validation Error')
plt.tight_layout()
plt.savefig('samples-exper.pdf')
