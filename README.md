# MPC-CBF
We propose a control framework which unifies the model predictive control and control barrier functions, where terminal cost function serves as control Lyapunov functions for stability. This is the reference implementation of our paper:
### Safety-Critical Model Predictive Control with Discrete-Time Control Barrier Function
[PDF](https://arxiv.org/abs/2007.11718) | [Code: Double Integratror](double-integrator-2D) | [Code: Car Racing](https://github.com/HybridRobotics/Car-Racing)

*Jun Zeng, Bike Zhang and Koushil Sreenath*

#### Citing
If you find this code useful in your work, please consider citing:
```shell
@article{zeng2020mpc-cbf,
  title={Safety-critical model predictive control with discrete-time control barrier function},
  author={Zeng, Jun and Zhang, Bike and Sreenath, Koushil},
  journal={arXiv preprint arXiv:2007.11718},
  year={2020}
}
```

#### Instructions
The 2D double integrator is assigned to reach the target position at origin while avoiding obstacles. We have three classes for different controllers: `DCLF_DCBF.m` (DCLF-DCBF), `MPC_CBF.m` (MPC-CBF) and `MPC_DC` (MPC-DC), respectively.

Moreover, to illustrate the performance among them, we have:
* `main.m`: Run DCLF-DCBF/MPC-CBF/MPC-DC respectively.
* `analysis_gamma.m`: Run analysis for different hyperparameter $\gamma$.
* `analysis_horizon.m`: Run analysis for different horizon.

We illustrate the performance between DCLF-DCBF/MPC-DC/MPC-CBF
| DCLF-DCBF  | MPC-DC (N=8) |
| --- | --- |
| <img src="double-integrator-2D/figures/dclf-dcbf-avoidance.png" width="200" height="200"> | <img src="double-integrator-2D/figures/mpc-dc-avoidance.png" width="200" height="200"> |

| MPC-CBF (N=1) | MPC-CBF (N=8) |
| --- | --- |
| <img src="double-integrator-2D/figures/mpc-cbf-avoidance-one-step.png" width="200" height="200"> | <img src="double-integrator-2D/figures/mpc-cbf-avoidance-several-steps.png" width="200" height="200"> |

and also the safety performance for different numbers of horizon and hyperparameters
| Different hyperparameter | Different horizon |
| --- | --- |
| <img src="double-integrator-2D/figures/benchmark-gamma.png" width="200" height="200"> | <img src="double-integrator-2D/figures/benchmark-horizon.png" width="200" height="200">

#### Dependencies
The packages needed for running the code are [Yalmip](https://yalmip.github.io/) and [IPOPT](https://projects.coin-or.org/Ipopt/wiki/MatlabInterface).

We also provide the zipped version of precompiled .mex files for IPOPT in the folder `packages` in case you don't have it. Unzip that file and add those .mex files into your MATLAB path.
