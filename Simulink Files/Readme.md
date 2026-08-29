### Required Simulink Toolboxes & Blocks

To execute the multi-physics and dynamic simulations, the following toolboxes are required:

* **Simulink:** Core platform for block diagram modeling, continuous-time state-space simulation, and signal routing.
* **Control System Toolbox:** Required to run the internal State-Space blocks and continuous PID controller blocks.
* **Simscape / Simscape Driveline:** *(If modeling DC motor dynamics & physical mechanical linkages)* Required for physical domain network modeling.
* **DSP System Toolbox:** *(If using digital filtering/sensor noise modeling)* Required for MPU6050 signal processing and noise generation blocks.
 
 **Note:** 
 
 **All MATLAB Files attached in this repo needed to be run first before running Simulink Model.**
