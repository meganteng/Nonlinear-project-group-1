# UC Berkeley EE222/ME237 Nonlinear Systems Ball and Beam Project

Group 1: Megan Teng, Boyuan Liang, Sveinung Myhre

Spring 2025

## Project Overview

This project involves designing and testing nonlinear controllers for a ball and beam system. The objective is to develop controllers that stabilize the ball at a desired position on the beam. You will first implement your controllers in MATLAB simulations and later test them on physical hardware.

## Code Instructions

### Prerequisites

Install MATLAB and Simulink using the Berkeley academic license.

### Getting Started

Clone or fork this repository.

Run `setup.m` or manually add the repository and its subfolders to the MATLAB path.

To try the controllers:

Run `run_matlab_ball_and_beam.m` for a MATLAB-based simulation (You can choose which control method to run).

Run `run_simulink_ball_and_beam.m` for a Simulink-based simulation (It only runs method LQR+FL+Luenberger).


-----------------------------------------------------------------------------
## Controller versions
Please modify the variable `controller_name` in script `run_matlab_ball_and_beam.m` if you want to try different control methods. The default controller is LQR with feedback linearization and Luenberger observer, since it has the best performance.
