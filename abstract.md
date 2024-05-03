# Abstract

Autonomous navigation is necessary for a robotic system to interact with its
surroundings in a real world environment, and it is necessary to realize
technologies such as fully autonomous unmanned aerial vehicles (UAVs) and land
vehicles. Reinforcement Learning (RL) has proven to be a novel and effective
method for autonomous navigation and control, as it is capable of optimizing a
method of converting its instantaneous state to an action at a point in time.
Here we use a Deep Deterministic Policy Gradient (DDPG) RL algorithm to train
the COEX Clover quadcopter system to perform autonomous navigation. With the
advent of solid state lasers, miniaturized optical ranging systems have become
ubiquitous for aerial robotics because of their low power and accuracy. By
equipping the Clover with ten Time of Flight (ToF) ranging sensors, we supply
continuous spatial data in combination with inertial data to determine the
quadcopter's state, which is then mapped to its control output. Our results
suggest that, while the DDPG algorithm is capable of training a quadcopter
system for autonomous navigation, its computation-heavy nature leads to delayed
convergence, and relying on discretized algorithms may permit more rapid
convergence across episodes.
