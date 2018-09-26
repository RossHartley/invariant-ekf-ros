# inekf-ros
This repository contains a ROS wrapper for the C++ inekf library located at [https://github.com/RossHartley/invariant-ekf](https://github.com/RossHartley/invariant-ekf)

This filter can be used to estimate a robot's 3D pose and velocity using an IMU motion model for propagation. The following measurements are currently supported:
* Prior landmark position measurements (localization)
* Estaimted landmark position measurements (SLAM)
* Kinematic and contact measurements


## Installation
TODO

## Parameters
TODO

## Examples
TODO

## Citations
The contact-aided invariant extended Kalman filter is described in: 
* R. Hartley, M. G. Jadidi, J. Grizzle, and R. M. Eustice, “Contact-aided invariant extended kalman filtering for legged robot state estimation,” in Proceedings of Robotics: Science and Systems, Pittsburgh, Pennsylvania, June 2018.
```
@INPROCEEDINGS{Hartley-RSS-18, 
    AUTHOR    = {Ross Hartley AND Maani Ghaffari Jadidi AND Jessy Grizzle AND Ryan M Eustice}, 
    TITLE     = {Contact-Aided Invariant Extended Kalman Filtering for Legged Robot State Estimation}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2018}, 
    ADDRESS   = {Pittsburgh, Pennsylvania}, 
    MONTH     = {June}, 
    DOI       = {10.15607/RSS.2018.XIV.050} 
} 
```
The core theory of invariant extended Kalman filtering is presented in:
* Barrau, Axel, and Silvère Bonnabel. "The invariant extended Kalman filter as a stable observer." IEEE Transactions on Automatic Control 62.4 (2017): 1797-1812.
```
@article{barrau2017invariant,
  title={The invariant extended Kalman filter as a stable observer},
  author={Barrau, Axel and Bonnabel, Silv{\`e}re},
  journal={IEEE Transactions on Automatic Control},
  volume={62},
  number={4},
  pages={1797--1812},
  year={2017},
  publisher={IEEE}
}
```
