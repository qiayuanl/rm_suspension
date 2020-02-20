# rm_suspension
Rigid body dynamics simulation for suspension design of RoboMaster robots

Simulation type:
- Drop from 1.2 meter height
- Run down the stairs
- Fly the ramp
- Brake in maximum speed;
- Diagonally across the ramp

![sim.png](https://i.loli.net/2020/02/21/QdIfsjzWmKxVB2Z.png)

## Dependencies:
- ROS (for visualization and params loading) - http://wiki.ros.org/ROS 
- Eigen - http://eigen.tuxfamily.org

to install Eigen on ubuntu 16.04/18.04, try:
- `sudo apt-get install gfortran liblapack-dev`

## Credits
The floating base dynamics and collision detect algorithms is base on @mit-biomimetics 's Cheetah software.

-https://github.com/mit-biomimetics/Cheetah-Software