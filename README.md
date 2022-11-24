# On the Initialization Problem for Timed-Elastic Bands
The code and simulation for the IFAC WC2023 submission "On the Initialization Problem for Timed-Elastic Bands"

## Authours
Niklas Persson, Martin C. Ekström, Mikael Ekström, Alessandro V. Papadopoulos

## Setup 

The code is written in MATLAB 2022b and the simulation is created in Simulink 2022b

### Requried toolboxes

- [Control System Toolbox](https://se.mathworks.com/products/control.html)
- [Model Predictive Control Toolbox](https://se.mathworks.com/products/model-predictive-control.html)
- [Parallel Computing Toolbox](https://se.mathworks.com/products/parallel-computing.html)
- [Image Processing Toolbox](https://se.mathworks.com/products/image.html)
- [Navigation Toolbox](https://se.mathworks.com/products/navigation.html)
- [Signal Processing Toolbox](https://se.mathworks.com/products/signal.html)
- [Simscape Multibody](https://se.mathworks.com/products/simscape-multibody.html)
- [Simscape](https://se.mathworks.com/products/simscape.html)

## Usage
- Run ´comparePP.m´ to create 300 random mazes and plan a path on each maze using A*, Theta*, Hybrid A*, and RRT*. Each planned path is also optimised using Time Elastic Bands. 
	*Important Note: This file takes a long time to execute, about 24hours when running with 8 cores at 2.3MHz. 
	*The file ´PPallRes.mat´ in the ´data´ folder includes the data which was used to create the results in the paper. 
- Run ´postProcess.m´ to compute the path length, the curvature and the IAT for all paths on all mazes from the ´PPallRes.mat´ file.
- Run ´simBicycle.m´ to start the simulation of the autonomous bicycle tracking a path planned by Optimised Theta*. 
 



