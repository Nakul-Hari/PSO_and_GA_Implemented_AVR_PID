# PSO_and_GA_Implemented_AVR_PID

This repository contains the MATLAB implementation of a project comparing the characteristics of PSO-PID controllers and GA-PID controllers. The project explores the use of Particle Swarm Optimization (PSO) and Genetic Algorithm (GA) for tuning Proportional-Integral-Derivative (PID) controllers, which are widely used in industrial control systems.

## Abstract

Proportional-Integral-Derivative (PID) control is a common algorithm in the industry used to improve dynamic response and reduce steady-state error. Traditional tuning methods like the Ziegler-Nichols method are often inadequate for finding near-optimal parameters. This project uses Particle Swarm Optimization (PSO) and Genetic Algorithm (GA) to find optimal PID parameters and compares their performance using MATLAB and the Control Systems Toolbox.

## Objectives

The main objectives of this project are:
- To explore the tuning of PID controllers using PSO and GA.
- To compare the performance and computational efficiency of PSO-PID and GA-PID controllers.
- To assess the advantages and disadvantages of both methods in real-world control systems.

## Methodology

### PID Controller

The PID controller aims to enhance the dynamic response and minimize steady-state errors of a system. It consists of three terms:
- Proportional (P)
- Integral (I)
- Derivative (D)

### Modeling a Linearized AVR System

An Automatic Voltage Regulator (AVR) system is used as the test case. The AVR system is linearized and consists of the following components:
- Amplifier
- Exciter
- Generator
- Sensor

### Performance Estimation

A new performance criterion is used to evaluate the PID controller, considering factors like maximum overshoot, rise time, settling time, and steady-state error.

### Genetic Algorithm (GA)

GA is an optimization technique inspired by natural selection. It involves selection, crossover, and mutation to evolve a population of candidate solutions towards an optimal solution.

### Particle Swarm Optimization (PSO)

PSO is inspired by the social behavior of birds and fish. It involves a population of particles that adjust their positions based on personal and global best positions to find an optimal solution.

## Results

Simulation results show that PSO-PID controllers demonstrate faster convergence and stability compared to GA-PID controllers. The performance criterion of the PSO-PID controller is significantly lower than that of the GA-PID controller.

## Conclusion

PSO and GA are effective methods for tuning PID controllers. PSO is particularly noted for its fast convergence and computational efficiency, making it suitable for real-time applications. GA, while robust, can be computationally intensive and less consistent in results.

## Future Perspective

Future work could explore hybrid algorithms combining PSO and GA, investigate different variants of these algorithms, and integrate machine learning approaches like reinforcement learning and neural networks for adaptive and self-tuning PID controllers.

## References

1. Zwe-Lee Gaing, "A particle swarm optimization approach for optimum design of PID controller in AVR system," IEEE Transactions on Energy Conversion, vol. 19, no. 2, pp. 384-391, June 2004. doi: 10.1109/TEC.2003.821821.
2. Solihin, Mahmud & Tack, Lee & Moey, Lip Kean. (2011). Tuning of PID Controller Using Particle Swarm Optimization (PSO). Proceeding of the International Conference on Advanced Science, Engineering and Information Technology. 1. 10.18517/ijaseit.1.4.93.
3. J. Kennedy and R. Eberhart, "Particle swarm optimization," Proceedings of ICNN'95 - International Conference on Neural Networks, Perth, WA, Australia, 1995, pp. 1942-1948 vol.4. doi: 10.1109/ICNN.1995.488968.

## How to Run

1. Clone this repository.
2. Open MATLAB and navigate to the project directory.
3. Run the MATLAB script to execute the simulations and view the results.

## Acknowledgements
This work is part of the course project for the course EE3060: Control Systems.
