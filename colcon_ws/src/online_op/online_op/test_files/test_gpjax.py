import os, sys
home_path_op = '/home/colcon_ws/src/online_op/online_op/'
#sys.path.append(current_dir + '/GPJax/')
sys.path.append(home_path_op+'GPJax')

import gpjax as gpx
import jax.numpy as jnp

xtrain = jnp.linspace(0, 1).reshape(-1, 1)
ytrain = jnp.sin(xtrain)
D = gpx.Dataset(X=xtrain, y=ytrain)
xtest = jnp.linspace(0, 1).reshape(-1, 1)

prior = gpx.gps.Prior(mean_function = gpx.mean_functions.Zero(), kernel = gpx.kernels.RBF())
posterior = prior * gpx.likelihoods.Gaussian(num_datapoints = D.n)
predictive_dist = posterior(xtest, D)

