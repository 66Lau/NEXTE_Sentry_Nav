# velocity_smoother_ema ROS package
This repository is an implementation of the Exponential Moving Average (EMA) formula for velocity smoothing.

## Velocity Smoother for Mobile Robots in ROS

This GitHub repository provides a velocity smoother implementation for mobile robots using the Robot Operating System (ROS). The velocity smoother utilizes the Exponential Moving Average (EMA) algorithm to achieve a smoothed velocity output.

The EMA algorithm blends the current velocity measurement with the previous smoothed velocity using a weight factor, resulting in a more stable and less noisy velocity signal. By applying this velocity smoother, robot control can be improved, leading to smoother and more precise motion.

## Key Features:

  - Implements the Exponential Moving Average (EMA) algorithm for velocity smoothing.
  - Subscribes to the robot's velocity topic and publishes the smoothed velocity output.
  - Allows customization of the weight factor (alpha) for fine-tuning the smoothing effect.
  - Compatible with ROS framework, facilitating integration with existing robotic systems.
  - Provides a launch file and example configuration for easy deployment.
  - This repository aims to enhance the control performance of mobile robots by reducing velocity fluctuations and improving overall motion quality.   
  - It serves as a valuable resource for ROS developers and roboticists seeking to implement velocity smoothing capabilities in their mobile robot projects.

Feel free to clone, contribute, and adapt the code to suit your specific robotic applications. Join the community and help advance the field of mobile robot control with a smoother and more efficient motion experience.

Let me know if there's anything else I can assist you with (seiftinik@gmail.com)!

## Usage:
Adapt the parameters to much your needs, then launch the following:

```console
you@you:~$ roslaunch velocity_smoother_ema velocity_smoother_ema.launch
```
or you can use rosrun

```console
you@you:~$ rosrun velocity_smoother_ema velocity_smoother_ema_node
```

<p align="center">
  <!-- ![velocity_smoother_ema](https://github.com/seifEddy/velocity_smoother_ema/blob/master/velocity_smoother_ema.png | width=100) -->
  <img src="https://github.com/seifEddy/velocity_smoother_ema/blob/master/velocity_smoother_ema.png" width="400" height="400">
</p>
