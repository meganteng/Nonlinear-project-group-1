# Nonlinear Control for a Ball and Beam System

**UC Berkeley EE222/ME237 - Spring 2025**

*Group 1: Megan Teng, Boyuan Liang, Sveinung Myhre*

## Project Summary

This project focused on designing, simulating, and testing controllers for a nonlinear physical ball and beam system. The primary objective was to stabilize the ball's position along the beam, tracking various reference trajectories.

We implemented and compared two main control strategies:
1.  A standard **PID Controller**.
2.  A **Feedback Linearization (FL) controller combined with LQR**.

Both controllers were tested in simulation and on hardware, tracking sine wave, square wave, and customized reference trajectories.

While the PID controller demonstrated effective and stable performance in both simulation and hardware experiments, the FL+LQR controller encountered unexpected difficulties. Despite attempts at implementing a Luenberger observer to estimate the system states as well as other approaches, the resulting control input (`u`) exhibited significant noise, hindering its performance considerably.

See [Setup Instructions](docs/SETUP.md) for details on running the code.

---

## Results

Below are links to detailed plots and videos from our simulations and hardware experiments.

- [PID Controller Plots & Simulation Video](docs/PID-controller-plots.md)  
- [Feedback Linearization + LQR Controller Plots & Simulation Video](docs/FL-LQR-controller-plots.md)  
- Experimental Hardware Video Clips are located in the `docs/Videos-(hardware)/` directory.

### PID Controller Hardware vs Simulation

Here's a comparison showing the PID controller running on the hardware versus in simulation:

**Hardware Experiment:**  
<a href="https://raw.githubusercontent.com/meganteng/Nonlinear-project-group-1/master/docs/Videos-(hardware)/PXL_20250430_201119682.mp4">
  <img src="docs/thumbnail-video-experiment-hardware-pid-controller.png" alt="PID Hardware Experiment" width="400"/>
</a>

**Simulation:**  
<a href="https://raw.githubusercontent.com/meganteng/Nonlinear-project-group-1/master/docs/PID-Controller-Plots/PID-simulator-tracking.mp4">
  <img src="docs/thumbnail-video-simulation-pid-controller.png" alt="PID Simulator Tracking" width="400"/>
</a>
