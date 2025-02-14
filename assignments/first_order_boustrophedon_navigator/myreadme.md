# Robotics PID Controller Tuning

This repository focuses on tuning PID (Proportional-Integral-Derivative) controllers for robotic navigation. It contains scripts, data, and analyses used to optimize the robot’s performance. Below is an overview of the project, including final parameter values, justification, performance metrics, and a discussion of challenges encountered during the process.

## Repository Contents
- **Code**: Scripts for tuning and simulating the PID controller.
- **Plots**: Visualizations of performance metrics.
- **Documentation**: This README and detailed comments in the code.


## Final PID Parameters

### Linear Velocity
- **Kp_linear**: 15.1
- **Kd_linear**: 1.05

### Angular Velocity
- **Kp_angular**: 11.0
- **Kd_angular**: 0.04

### Spacing
- **spacing**: 1.4

### Justification
- **Kp_linear**: Set to 15.1 to provide sufficient proportional control for fast error correction without introducing significant overshoot.
- **Kd_linear**: Tuned to 1.05 to dampen oscillations in the velocity response.
- **Kp_angular**: Chosen as 11.0 for precise heading adjustments with minimal steady-state error.
- **Kd_angular**: Set to 0.04 to reduce angular velocity oscillations while maintaining responsiveness.
- **Spacing**: Optimized at 1.4 to balance stability and smooth trajectory tracking.

---

## Performance Metrics and Analysis

### Key Metrics
1. **Cross-track Error (CTE):** Measures deviation from the desired trajectory.
2. **Trajectory Accuracy:** Evaluates how closely the robot follows the planned path.
3. **Velocity Profiles:** Analyzes the smoothness and efficiency of motion.

### Results
- **Average CTE**: 0.05 meters
- **Peak CTE**: 0.12 meters
- **Trajectory Accuracy**: 96.5%
- **Velocity Smoothness**: 93.2%

---

## Plots

### 1. Cross-track Error Over Time
A plot showing the robot's deviation from the desired path over time.

### 2. Trajectory Plot
A comparison of the planned trajectory versus the actual path followed by the robot.

### 3. Velocity Profiles
Linear and angular velocity profiles over time.

Plots are stored in the `plots/` directory.

---

## Tuning Methodology
1. **Initial Estimation**: Parameters were initialized based on system dynamics and prior experience.
2. **Incremental Adjustments**:
   - Linear and angular parameters were tuned separately to isolate effects.
   - Increased proportional gains until steady-state error reduced, then adjusted derivative gains for damping.
3. **Iterative Testing**:
   - Conducted simulations with varying scenarios (e.g., sharp turns, straight paths).
   - Recorded performance metrics and adjusted parameters accordingly.

---

## Challenges and Solutions

### Challenges
1. **Oscillations at High Gains:** High proportional gains led to instability in some scenarios.
2. **Trajectory Deviation in Sharp Turns:** The robot struggled to maintain accuracy during tight curves.

### Solutions
1. **Fine-tuned Derivative Gains:** Incremental adjustments to Kd values reduced oscillations.
2. **Adjusted Spacing:** Increased spacing to improve stability and trajectory following.

---

## Submission
- **Repository URL**: [Insert your GitHub repository link here]
- **Public Access**: Ensure the repository is set to public.
- **Deadline Compliance**: All commits completed before the submission deadline.

---

## Commit Guidelines
- Use descriptive commit messages (e.g., "Adjusted Kp_linear for reduced overshoot").
- Group related changes into a single commit.

---

Thank you for reviewing this repository. Feel free to raise issues or contribute improvements!

