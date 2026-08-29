Self-Balancing Two-Wheeled Robot

A controls engineering project covering mathematical modeling, state-space analysis, PID versus LQR controller design in MATLAB and Simulink, and experimental Arduino and MPU6050 hardware validation.


Executive Summary

This repository documents the modeling, control-system design, simulation, and hardware implementation of an inverted pendulum system mounted on a two-wheeled mobile platform.

The primary objective is to maintain an upright position at zero degrees while stabilizing spatial position under non-zero initial conditions, sensor noise, motor saturation, and torque disturbances.


Controller Performance & Video Demonstrations

No Controller (Unstable System):

The uncontrolled inverted pendulum system cannot maintain upright equilibrium and diverges immediately.


https://github.com/user-attachments/assets/b8ae47fd-0084-4922-9757-daacec539905





PID Control (Classical Feedback):

Classical PID control stabilizes the pendulum angle and recovers equilibrium. 
When subjected to external impulse pushes, the controller restores balance, 
though with minor overshoot and oscillation as it compensates for position drift.




https://github.com/user-attachments/assets/ec427553-9775-4963-9a01-e5ffcb30c9e1








LQR Control (Optimal State-Space Stabilization):

Full-state feedback LQR control optimizes the trade-off between state deviation and control effort. When subjected to successive external impulse pushes, the system exhibits rapid disturbance rejection, zero steady-state position drift, and minimal settling time without actuator saturation.



https://github.com/user-attachments/assets/95d6b5c3-9936-4dc5-a3d5-7796f7bb91ad






Mathematical Model and State-Space Analysis

The system state vector is defined by four variables:
1. Cart position (x)
2. Cart linear velocity (x_dot)
3. Pitch angle from vertical equilibrium (theta)
4. Angular velocity (theta_dot)

Linearized State-Space Representation:
dx/dt = A*x + B*u
y = C*x + D*u

System Parameters:
- Cart mass (M) = 0.09 kg
- Pendulum body mass (m) = 0.34 kg
- Distance to center of mass (l) = 0.031 m
- Pendulum moment of inertia (I) = 0.000953 kg-m^2
- Gravity (g) = 9.81 m/s^2

A matrix:
  0    1.0000         0         0
  0    0        -2.3415         0
  0    0              0    1.0000
  0    0        31.2910         0

B matrix:
  0
  0.3120
  0
 -3.8150

System Properties Analysis:
- Controllability: rank(Co) = 4. The system is fully controllable.
- Observability: rank(Ob) = 4. The system states are fully observable.


Optimal LQR Design

State and input weighting matrices selected for controller synthesis:
- Q = diag([1, 1, 500, 10])
- R = 1

Full-State Feedback Gain Vector derived via Continuous Algebraic Riccati Equation:
K = [-1.0000, -2.5300, -30.7086, -3.8517]

Control Law:
u = -K * x


Simulink Model Progression

The simulation was built incrementally across six model iterations to capture real-world non-idealities:

1. Ideal Linearized State-Space Plant
2. Full-State LQR Gain Integration
3. DC Motor Back-EMF and Inductance Dynamics
4. PWM Actuator Voltage Saturation (+/- 12V limits)
5. External Impulse and Torque Disturbance Injection
6. Gaussian Sensor Noise and Complementary Filtering for MPU6050


Hardware Implementation and Debugging

- Microcontroller: Arduino Nano running a fixed 100 Hz control loop.
- Sensor Fusion: MPU6050 accelerometer and gyroscope fused using a Complementary Filter to remove high-frequency vibration noise and low-frequency gyro drift.
- Power Architecture: Resolved early motor power starvation by isolating logic and motor power rails with a shared single-point common ground. Full details are available in Hardware/components.md.


Repository Directory Structure

Self-Balancing-Robot-Control/
│
├── MATLAB/            Scripts for physical parameters, state matrices, and LQR gains
├── Simulink/          Progressive simulation models (Ideal through Non-ideal)
├── Arduino/           Real-time C++ microcontroller implementation code
├── Hardware/          Wiring layout, components specs, and power debugging notes
└── Results/           Simulation MP4 renders and performance comparisons
