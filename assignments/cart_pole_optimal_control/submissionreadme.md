# Submission README: Cart-Pole Optimal Control Assignment

## Overview
This submission contains all the required deliverables for the Cart-Pole Optimal Control assignment. The focus is on analyzing and tuning an LQR controller for stabilizing the cart-pole system under earthquake disturbances. Additionally, the submission includes performance analysis, supporting visualizations, and video demonstrations.

## Submission Contents
1. **Technical Report**
   - Document detailing:
     - LQR tuning approach
     - Analysis of controller parameters (Q and R matrices)
     - Performance metrics and observations
     - Trade-offs and findings
   - File: `technical_report.pdf`

2. **Simulation Videos**
   - Demonstrations of system performance under different LQR configurations:
     - Configuration 1: Higher instability observed
       - **Q Matrix**: `diag([1.5, 2.5, 25.0, 10.5])`
       - **R Value**: `0.10`
     - Configuration 2: Improved stability with better parameter tuning
       - **Q Matrix**: `diag([1.2, 2.2, 22.0, 8.0])`
       - **R Value**: `0.08`
   - File Names:
     - `configuration1_performance.mp4`
     - `configuration2_performance.mp4`

3. **Code Files**
   - ROS2 package with LQR controller implementation and earthquake disturbance generator:
     - Tuned Q and R matrices
     - Configurable parameters
   - Directory: `cart_pole_optimal_control`

4. **Performance Plots and Visualizations**
   - Data visualizations showing:
     - Cart position and velocity
     - Pole angle deviation
     - Control effort distribution
     - System recovery from disturbances
   - Directory: `visualizations/`
     - Files: `cart_position_plot.png`, `pole_angle_plot.png`, `control_effort_plot.png`

5. **Extra Credit (Optional)**
   - Implementation of a DQN controller:
     - Training progress visualizations
     - Comparison with LQR performance
   - Directory: `extra_credit/`
     - Files: `dqn_training_plot.png`, `dqn_vs_lqr_comparison.pdf`

## How to Access the Submission
### ROS2 Simulation
1. **Setup Instructions**:
   - Prerequisites: ROS2, Gazebo, Python dependencies
   - Build the package:
     ```bash
     cd ~/ros2_ws
     colcon build --packages-select cart_pole_optimal_control --symlink-install
     ```
   - Source the workspace:
     ```bash
     source install/setup.bash
     ```
2. **Run the Simulation**:
   ```bash
   ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py
   ```
3. **Visualize in RViz**:
   - Observe cart-pole dynamics, control forces, and disturbances.

### Videos
- Open `configuration1_performance.mp4` and `configuration2_performance.mp4` in any video player to review system behavior.

### Reports and Visualizations
- All reports and plots can be viewed in their respective directories.

## Contact Information
For any queries or clarifications regarding this submission, please contact:
- **Name**: [Your Name]
- **Email**: [Your Email]
- **GitHub**: [Your GitHub Profile]

## License
This submission is licensed under a [Creative Commons Attribution 4.0 International License](https://creativecommons.org/licenses/by/4.0/).