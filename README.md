<h1 align="center">ROLL</h1>
<h2 align="center">Real-time Optimization of Link Length for highly efficient quadruped robots</h2>

## ✨Summary
This repository about real-time optimization of input link length for 4bar linkage based quadruped robots.
<br>
We suggest new mechanism that adjust input link length using sub actuator and optimal length control method.
<br>
<br>
[Youtube Video1](https://youtu.be/Fd4iINc_CYo?si=tXBfKRfczNwu97U1) / [Youtube Video2](https://youtu.be/Z9Yeyp4EUFA?si=rExXM8d1uZO81aSZ)

<br>

<img src="https://github.com/cobang0111/ROLL/assets/97373900/55d20f19-4c78-4da8-8d52-a951c38639bd" width="640">


<br>

## ✨Modeling
3D CAD Modeling
<br>

<img src="https://github.com/cobang0111/ROLL/blob/main/image/3dcad_modeling.png" width="320">



Metal Modeling (Stainless steel + Aluminium)
<br>

<img src="https://github.com/cobang0111/ROLL/blob/main/image/metal_modeling.png" width="320">

<br>

## ✨Mechanism

Hip (AK80-9 Motor) 
<br>

<img src="https://github.com/cobang0111/ROLL/blob/main/image/hip.gif" width="320">

Knee (AK80-9 Motor) 
<br>

<img src="https://github.com/cobang0111/ROLL/blob/main/image/knee.gif" width="320">

Input Link Length (BL2446S Motor)
<br>

<img src="https://github.com/cobang0111/ROLL/blob/main/image/clutch.gif" width="320">


- Motor control on MATLAB Simulink R2024a
- TI launchpad f28379d Motor Driver


<br>

## ✨Overview
coming soon
<br>

- Our Method
  We optimized the input link length under the constraint of motor. 
  <br>
  
  <img src="https://github.com/cobang0111/ROLL/blob/main/image/link_length_adjustment.png" width="640">

  First, we focused on a sit and stand task using position control. <br>
  We determined the trajectory of end effector to carry out sit and stand task. <br>
  Then, we obtained the angle trajectory of 2 main motor(hip and knee) by solving inverse kinematics of legs. <br>  

  <br>

  To optimize the input link length, we set the constrained optimization problem as like below. <br>
  All constraint from the specification of hardware. <br>
  Our main optimization purpose is reducing energy consumtion.
  <br>  

  <img src="https://github.com/cobang0111/ROLL/blob/main/image/optimization_problem.png" width="640">

  <br>
  We know about the torque term from model dynamics. <br>
  All the things are same as other quadruped robot leg except the knee torque term. 
  <br>
  
  <img src="https://github.com/cobang0111/ROLL/blob/main/image/model_dynamics.png" width="640">

  Non-liner function f determined on the convex plane.
  We know the initial point on the plane.
  And we know the next $$\theta_2$$
  <img src="https://github.com/cobang0111/ROLL/blob/main/image/new_term_graph.png" width="640">



## ✨Execution
coming soon
<br>


## ✨Result

<img src="https://github.com/cobang0111/ROLL/blob/main/image/result_graph.jpg" width="640">

<br>



