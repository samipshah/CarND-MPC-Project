# MODEL

Following equations describes the model used in this project for car.
```
   x     = x0 + v0 * cos(psi0) * dt;
   y     = y0 + v0 * sin(psi0) * dt;
   psi   = psi0 + (v0/Lf)*delta0*dt;
   v     = v0 + a0*dt;
   cte   = f(x0) - y0 + v0 * sin(epsi0) * dt;
   epsi  = psi0 - psides0 + (v0/Lf)*delta0*dt;

   f(x)  = c0 + c1*x + c2*x^2 + c3*x^3;
   f'(x) = c1 + 2*c2*x + 3*c3*x^2;
   psides(x) = tan-1(f'(x)); 
```
State Variables
* Location, Orientation and Speed of car
    * x   (x-axis), 
    * y   (y-axis), 
    * psi (orientation with respect to x-axis), 
    * v   (velocity), 

* Error with respect to expected location, orientation
    * cte  (cross track error)
    * epsi (orientation error)

Actuators of Car
* delta (Steering angle)
* a     (Acceleration)

Car Charecteristic
* Lf (Distance beetween Center of Gravity and front bumper)

Path
* f (polynomial function describing path)


# Timestep Length and Elapsed Duration
Timestamp selected is 0.05 and number of steps chosen is 15 so looking forward till 0.75 seconds. So at maximum speed that results in about 20 meters distance. If number of steps is increased it slows down the process of calculating optimum control signals. If step size is increased from .05s to more results in less smooth calculation of path which may result in abrupt control outputs and it affects ride.

# Polynomial Fitting and MPC Preprocessing
This covers general process that results in unity inputs to control outputs

## Polynomial Fitting

First received unity inputs is converted to vehicle coordinate system using affine transformation. I have done it in two steps, first step is origin of coordinate system is moved to vehicle CoG. And in second step coordinate system is rotated by orientation of the car such that x axis aligns in the car direction. 

Once this coordinate transformation is done for car as well as way points, polyfit is run on waypoints to find appropriate polynomial f(x) of order 3 to approximate the path of the car.

## MPC Preprocessing

As part of mpc preprocessing, a large vector is created using CppAD and vector has every state and for all 15 steps. So in our case it comes out to be 15*6 = 90 elements. And it also contains actuators for 14 steps one less as the last one is not counted. In our case that is 14*2 = 28 elements. All of the values upper bound and lower bound values are set. In our case state variables can take any values so set to max and min of double while actuators have physical limitation that steering wheel should not move beyond 25 degrees on each side and acceleration must stay within [-1, 1]. Once variables and upper lower bounds are set, for state variables we set constraints upper and lower bound which basically is set to 0 which suggests we are trying to achieve error of 0.

# Model Predictive Control with Latency

At the heart of MPC solving algorithm it has FG_eval. CppAD is really helpful here because underneath it has automatic differentiation support. Our model is defined in operator() function of FG_eval. Operator stores model values in fg vector. MPC Solve tries to make fg 0. Since constraints are set to 0, by manipulating variables , making sure variables stay within their upper/lower bounds. Coefficient is passed to FG_Eval so path is given to this function and path is fixed. Since all computation is done using ADVectors , iterative changes and computations are recorded MPC Solve takes advantage of it in deciding which direction to change variables so as to minimize steps while minimizing this non linear problem.

```
min f(x) 
where   gl < g(x) < gh 
        xl < x  < x
```

Latency between control output and its effect adds a challenge to the problem. Currently the way I have solved it is predict 15 steps and since the latency is 100 milliseconds which is equal to 2 steps , I take the 3rd control output set as output of the algorithm since that is the time it will actually be effective. Moreover first 2 control variables are also set to previously applied control values so that MPC does not make changes to those values while finding a global minimum.

