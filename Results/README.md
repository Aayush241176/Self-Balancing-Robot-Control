Simulation Response Analysis

<img width="1119" height="857" alt="PID vs LQR Comparison Graphs" src="https://github.com/user-attachments/assets/721255a2-b74e-4ca8-bca1-8af23e0be9f0" />


Quantitative Comparison Analysis

1. Faster Angular Recovery: LQR restores the robot from an initial 10 degree tilt back to vertical faster than PID, exhibiting minimal undershoot (-1 degree) before smoothly settling to 0 degrees.

2. Minimal Position Drift: Because LQR considers all system states simultaneously via full-state feedback, total spatial displacement is capped at under 0.19 meters. Classical PID control allows maximum spatial drift up to 0.30 meters before correcting trajectory.

3. Coupled State Optimization: LQR penalizes position error alongside tilt angle using weight matrix Q = diag([1 1 500 10]), resulting in faster overall system settling without excessive actuator effort.
