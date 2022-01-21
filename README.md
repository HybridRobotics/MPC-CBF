**Status:** This repository is archived. For latest work about discrete-time CBF, please refer to the [collection repository](https://github.com/HybridRobotics/NMPC-DCLF-DCBF).

# MPC-CBF
We propose a control framework which unifies the model predictive control and control barrier functions, where terminal cost function serves as control Lyapunov functions for stability. This is the reference implementation of our paper:
### Safety-Critical Model Predictive Control with Discrete-Time Control Barrier Function
[PDF](https://arxiv.org/abs/2007.11718) | [Code: Double Integratror](examples) | [Code: Car Racing](https://github.com/HybridRobotics/Car-Racing)

*Jun Zeng, Bike Zhang and Koushil Sreenath*

#### Citing
If you find this project useful in your work, please consider citing following work:
```
@inproceedings{zeng2021mpccbf,
  title={Safety-critical model predictive control with discrete-time control barrier function},
  author={Zeng, Jun and Zhang, Bike and Sreenath, Koushil},
  booktitle={2021 American Control Conference (ACC)},
  year={2021},
  volume={},
  number={},
  pages={3882-3889}
}
```

For analysis of feasibility, safety and computational complexity, please check out the following paper:
```
@inproceedings{zeng2021mpccbf-feasibility,
  title={Enhancing feasibility and safety of nonlinear model predictive control with discrete-time control barrier functions},
  author={Zeng, Jun and Li, Zhongyu and Sreenath, Koushil},
  booktitle={2021 Conference on Decision and Control (CDC)},
  year={2021},
  volume={},
  number={},
  pages={6137-6144}
}
``` 

#### Instructions
The 2D double integrator is assigned to reach the target position at origin while avoiding obstacles. We have three classes for different controllers: `DCLFDCBF.m` (DCLF-DCBF), `MPCCBF.m` (MPC-CBF) and `MPCDC` (MPC-DC), respectively.

Moreover, to illustrate the performance among them, we have:
* `test.m`: Run DCLF-DCBF/MPC-CBF/MPC-DC respectively.
* `testGamma.m`: Run analysis for different hyperparameter $\gamma$.
* `testHorizon.m`: Run analysis for different horizon.
* `testBenchmark.m`: Run analysis for some benchmark.

We illustrate the performance between DCLF-DCBF/MPC-DC/MPC-CBF
| DCLF-DCBF  | MPC-DC (N=8) |
| --- | --- |
| <img src="examples/figures/dclf-dcbf-avoidance.png" width="200" height="200"> | <img src="examples/figures/mpc-dc-avoidance.png" width="200" height="200"> |

| MPC-CBF (N=1) | MPC-CBF (N=8) |
| --- | --- |
| <img src="examples/figures/mpc-cbf-avoidance-one-step.png" width="200" height="200"> | <img src="examples/figures/mpc-cbf-avoidance-several-steps.png" width="200" height="200"> |

and also the safety performance for different numbers of horizon and hyperparameters
| Different hyperparameter | Different horizon |
| --- | --- |
| <img src="examples/figures/benchmark-gamma.png" width="200" height="200"> | <img src="examples/figures/benchmark-horizon.png" width="200" height="200">

#### Dependencies
The packages needed for running the code are [Yalmip](https://yalmip.github.io/) and [IPOPT](https://projects.coin-or.org/Ipopt/wiki/MatlabInterface).

We also provide the zipped version of precompiled .mex files for IPOPT in the folder `packages` in case you don't have it. Unzip that file and add those .mex files into your MATLAB path.
