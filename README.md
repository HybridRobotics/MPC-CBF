# MPC-CBF
We propose a control framework which unifies the model predictive control and control barrier functions. This is the reference implementation of our paper:
### Safety-Critical Model Predictive Control with Discrete-Time Control Barrier Function
[PDF](https://arxiv.org/abs/2007.11718) | [Code: Double Integratror](double-integrator-2D) | [Code: Car Racing](car-racing)

*Jun Zeng, Bike Zhang and Koushil Sreenath*

<img src="car-racing/demo.gif" height="200"/>

#### Citing
If you find this code useful in your work, please consider citing:
```shell
@article{zeng2020mpccbf,
  title={Safety-critical model predictive control with discrete-time control barrier function},
  author={Zeng, Jun and Zhang, Bike and Sreenath, Koushil},
  journal={arXiv preprint arXiv:2007.11718},
  year={2020}
}
```

### Instructions
Two independent markdown files are written to illustrate numerical examples of [double integrator](double-integrator-2D/README.md) and [racing car](car-racing/README.md).

### Dependencies
#### Matlab
The packages needed for running the code are [Yalmip](https://yalmip.github.io/) and [IPOPT](https://projects.coin-or.org/Ipopt/wiki/MatlabInterface).

We also provide the zipped version of precompiled .mex files for IPOPT in the folder `packages` in case you don't have it. Unzip that file and add those .mex files into your MATLAB path.