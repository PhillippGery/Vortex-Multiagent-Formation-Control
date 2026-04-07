# Vortex Multiagent Navigation and Formation Control

## Project Title
Vortex Multiagent Navigation and Formation Control

## Description
This project implements a distributed multi-agent formation control system for nonholonomic unicycle robots using a Vortex Artificial Potential Field (APF) combined with Feedback Linearization control techniques. The system enables autonomous robots to navigate in formation while avoiding obstacles and maintaining connectivity constraints through Laplacian-based barriers.

**Key Features:**
- Vortex Artificial Potential Field for agent attraction and repulsion
- Feedback Linearization with Look-Ahead point control strategy
- Connectivity-preserving constraints using Laplacian barriers
- Multi-agent formation scenarios (V-formation, convoy, etc.)
- Obstacle avoidance with circular obstacle modeling

## Installation

### Prerequisites
- MATLAB R2022b or later
- MATLAB Toolboxes:
  - Control System Toolbox
  - Robotics System Toolbox (optional, for visualization enhancements)

### Setup
1. Clone or download the repository:
   ```bash
   git clone <repository_url>
   cd Vortex-Multiagent-Formation-Control
   ```

2. Add the project directories to MATLAB path:
   ```matlab
   addpath(genpath(pwd))
   ```

3. Verify installation by running a test scenario:
   ```matlab
   cd 30_Simulation_Scenarios
   main_milestone_1
   ```

## Usage

### Running Simulations
Navigate to the `30_Simulation_Scenarios` folder and run the desired milestone:

- **Milestone 1**: Single agent obstacle avoidance
  ```matlab
  main_milestone_1
  ```

- **Milestone 2**: Two-agent connectivity verification
  ```matlab
  main_milestone_2
  ```

- **Milestone 3**: Five-agent V-formation control
  ```matlab
  main_milestone_3
  ```

### Project Structure
```
10_Documentation/          % Project documentation and mathematical framework
20_Core_Math/             % Core mathematical functions
30_Simulation_Scenarios/  % Milestone scripts and simulation experiments
40_Utilities/             % Visualization and helper functions
```

### Key Functions

- **compute_total_apf.m**: Calculates attractive, repulsive, vortex, and connectivity forces
- **compute_laplacian.m**: Computes Laplacian matrices and algebraic connectivity
- **unicycle_dynamics.m**: Implements ODE45-compatible unicycle robot dynamics
- **plot_environment.m**: Renders obstacles and workspace bounds
- **animate_robots.m**: Visualizes robot trajectories and formations in real-time

## Mathematical Framework
For detailed mathematical derivations of the Look-Ahead kinematics, Vortex APF formulation, and Laplacian connectivity barriers, see `math_framework.md`.

## License
[Add your license information here]

## Contributors
[Add contributor information]

## Contact
For questions or issues, please create an issue in the repository.
