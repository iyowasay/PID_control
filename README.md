# PID control project
Self-Driving Car Engineer Nanodegree Program

---

## Goal

The objective of this project is to build a PID controller in C++, so that the car in the Udacity simulator can smoothly complete a full lap without driving off the road. The simulator communicates with the application via uWebSockets. The cross track error and the throttle are calculated and act as the inputs of the PID controller. 

Cross track error(CTE): the distance between the center of the lane and the actual position of the vehicle.

## Concept

PID controller is a common controller using a feedback loop. There are three main components:

* P(propotional): Correspond to Kp. This term is propotional to the CTE. In other words, the system will react and steer harder when the error is large. However this is definitely not enough for autonomous control since the mass and inertia of the vehicle are not considered, which leads to overshooting. Besides, it only takes account of the current(present) state, meaning that it doesn't consider the previous or future state of the system. 

* I(integral): Correspond to Ki. This term accumulates the previous errors and hence is able to make a correction of the unwanted trend after a duration.

* D(differential): Correspond to Kd. This term predicts the variation of error and adjust the input accordingly. 


## Parameters tuning 

The most important part is to finely tune these hyperparameters in order to acquire the desired control response. Here two PID controllers are implemented to adjust the steering angle and the speed, respectively. 
There are different methods for parameters tuning, such as twiddling or stochastic gradient descent(SGD). In this project, these hyperparameters are chosen by manually tuning. In addition, the twiddle method(coordinate ascent algorithm) is also implemented in `PID.cpp`.  

Steering controller: 
- Kp = 0.2
- Ki = 0.00007
- Kd = 5

Speed controller:
- Kp = 0.8
- Ki = 0.00007
- Kd = 2








