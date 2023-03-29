# Ballbot
This repo contains the dynamics, simulation, and control code for a planar ballbot system. A ballbot is a robot that balances atop a basketball and rolls the basketball underneath it in order to move about the world. Though similar to a segway, the ballbot is a particularly challenging system to control because its center of mass is significantly above the ball. This makes the dynamics more similar to an inverted pendulum on a cart. 

This code was developed for a class project in Applied Optimal Control taught by Dr. Christian Hubicki in Fall 2023. The course was offered in person to Florida A&M / Florida State Univerisity students and remotely to University of Michigan students. 

The dynamics and modeling theory for this project was inspired by the Bachelor's Thesis by Fankhauser and Gwerder [1]. 

## Dynamics and Simulation
This repo contains the code to derive the equations of motion of the system using Lagrangian Mechanics. To update the dynamics and get started with a test simulation, use the following steps:
1) Update the parameters in [`Ballbot.defineParams`](+Ballbot/defineParams.m)
2) Run [`Ballbot.genFunctions`](+Ballbot/genFunctions.m)
3) Run [`test_BallbotSimulation`](Tests/test_BallbotSimulation.m). In this script, you can modify the simulation parameters, such as the initial and final conditionds, constraints, and duration. 

Here's an example of the multiple-shooting optimal controller moving the ballbot between two positions. 


https://user-images.githubusercontent.com/70407790/227629255-93a0f35a-fe6e-471c-8e22-80b6b0ee4f2e.mp4

## Control
Coming soon!

## References
[1] P. Fankhauser and C. Gwerder, “Modeling and control of a ballbot,” Bachelor’s Thesis, ETH Zurich, 2010, online: https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154271/eth-7943-01.pdf.
