### 2D double integrator
The 2D double integrator is assigned to reach the target position at origin while avoiding obstacles. We have three classes for different controllers: `DCLF_DCBF.m` (DCLF-DCBF), `MPC_CBF.m` (MPC-CBF) and `MPC_DC` (MPC-DC), respectively.

Moreover, to illustrate the performance among them, we have:
* `main.m`: Run DCLF-DCBF/MPC-CBF/MPC-DC respectively.
* `analysis_gamma.m`: Run analysis for different hyperparameter $\gamma$.
* `analysis_horizon.m`: Run analysis for different horizon.

We illustrate the performance between DCLF-DCBF/MPC-DC/MPC-CBF
| DCLF-DCBF  | MPC-DC (N=8) |
| --- | --- |
| <img src="figures/dclf-dcbf-avoidance.png" width="200" height="200"> | <img src="figures/mpc-dc-avoidance.png" width="200" height="200"> |

| MPC-CBF (N=1) | MPC-CBF (N=8) |
| --- | --- |
| <img src="figures/mpc-cbf-avoidance-one-step.png" width="200" height="200"> | <img src="figures/mpc-cbf-avoidance-several-steps.png" width="200" height="200"> |

and also the safety performance for different numbers of horizon and hyperparameters
| Different hyperparameter | Different horizon |
| --- | --- |
| <img src="figures/benchmark-gamma.png" width="200" height="200"> | <img src="figures/benchmark-horizon.png" width="200" height="200">