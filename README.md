# Robotics2Project

This is the project for the course of Robotics II (A.Y. 2019/2020). 

The project consists in the implementation and comparison of two different **nonlinear state observers** whose state estimate is exploited to control a **robot manipulator with flexible joints** with two different controllers.

## Project

### Motivation
The problem of estimating the full state of an elastic joint robot is a longstanding problem in the literature, since the control of this kind of robots through static state feedback requires the knowledge of the four state variables for each joint, usually joints and motors positions and their time derivatives.

### Outline
In this project we implemented the following nonlinear state observers:
- **Tomei observer** that requires the measurement of both the links' position and velocity (see [An observer for 
exible joint robots](https://ieeexplore.ieee.org/abstract/document/53558) for details)
- **Ciccarella - Germani observer** that in principle requires just the measurement of the links position (see [A luenberger-like observer for nonlinear systems](https://www.tandfonline.com/doi/abs/10.1080/00207179308934406) for details)

These observers were tested in a simulated environment on the models of two simple manipulators with flexible joints that we implemented, following the dynamic modelling outlined in [Modeling and Control of Elastic Joint Robots](https://asmedigitalcollection.asme.org/dynamicsystems/article-abstract/109/4/310/399330/Modeling-and-Control-of-Elastic-Joint-Robots). In fact, the reconstructed state has been used to implement the following controllers, presented by Spong himself in the previous paper:
- **Feedback linearizing controller** that allows to linearize the system dynamics and to obtain a controllable, LTI and
decoupled system (in which every I/O channel appears as a chain of four integrators)
- **Integral manifold controller** that is obtained starting from a reformulation of the dynamic equations of the robot model as a singularly perturbed system

### Project structure

The project consists of MATLAB scripts and a Simulink scheme, with the former performing
the computations required to simulate the latter. In particular, the project is structured as follows:
- `simulation.m`. The root script of the project, and the only one that must be run, since it
  internally invokes all the others and then opens the Simulink scheme.
  The only change required is to specify the desired case (2R or 3R robot) by setting the variable n.
- `simulink_model.slx`. The Simulink scheme of the project.
- `init.m`. Set the paths to Matlab.
- `config_{2R,3R}.m`. Stores the parameters' configuration for simulating the project for the case of a {2R planar, 3R spatial} robot.
  Modifying these parameters is allowed, but beware that by doing so the MATLAB functions will need to be computed again, 
  with the new set of parameters, and this might be a long process, especially for the 3R case.
- `model_{2R, 3R}.m`. Performs the symbolic derivation of the model for the  {2R planar, 3R spatial} robot.
- `computegains.m`. Computes the gains for the observers and controllers, given the parameters specified in the configuration files.
- `exportfuns.m`. Exports the results of the previous symbolic derivations to MATLAB functions.
- `funs{2R, 3R}/`. Folder that collects the exported MATLAB functions mentioned above.
- `private/`. Folder that collects our user-defined utility functions.
- `slprj/`. Folder created internally by Simulink.

### Simulink scheme
The Simulink scheme consists in six subsystems:
1. Robot dynamics.
2. Observer 1 (Tomei).
3. Observer 2 (Ciccarella-Germani) : observer with only links' positions.
4. Observer 3 (Ciccarella-Germani) : observer with also links' velocities as additional output.
5. Observer 4 (Ciccarella-Germani) : observer with motors' positions.
6. Observer 5 (Ciccarella-Germani) : observer with motors' positions and elastic torque.
7. Manifold Controller.
8. FL (Feedback Linearizing) Controller.
    
### How to start a simulation

To summarize, to perform a simulation one has to:
1. Open `simulation.m` in MATLAB and correctly set the variable `n` (to either 2 or 3). In addition, you can change the others settings at the beginning of `simulation.m` (not recommended).
2. Open the appropriate config file (either `config_2R.m` or `config_3R.m`) in MATLAB and set the desired configuration for the script
3. Run `simulation.m`
4. Once the script is completed, it will open the Simulink scheme in a new window.
5. Run the configured simulation then open the Simulink scheme and run the configured simulation. The default solver is `ode15s`, but some simulations may perform better with `ode23s`.

## Documentation

You can find the full report of our work on the project [here](report.pdf).

## Resources used

- [MATLAB](https://www.mathworks.com/products/matlab.html)
- [Simulink](https://www.mathworks.com/products/simulink.html)

## Authors

- [Andrea Caciolai](https://github.com/caciolai)
- Pietro Pustina
