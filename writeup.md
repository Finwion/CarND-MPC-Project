# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## The Model

The model was based on the MPC quiz, with the modification of extending the equations to work with a 3rd order polynomial.
The state has the following:
* x position
* y position
* psi heading
* velocity 
* cte - Cross-track error 
* epsi - psi error

The actuators used are:
* steering angle
* acceleration 

The kinematic equations used in `MPC.cpp` were:
```C++
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * pow(x0, 2));
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```

## Timestep Length and Elapsed Duration (N & dt)

N = 25 and dt = 0.05 was chosen, which results in a 1.25second evaluation period. This is the same as the lecture & quiz values. A larger dt like 0.5 resulted in prediction that caused the car to drive off the side of the road.

## Polynomial Fitting and MPC Preprocessing
The waypoints are converted from the map coordinate reference to the vehicle coordinate reference. Then they are fitted to a 3rd order polynomial. 

## Model Predictive Control with Latency

The model compensates for the simulator latency of 100ms by adjusting the initial state vector for the x and psi values. The x given to the MPC is actually the velocity * latency value. This is the position that we project the car to be at in 100ms. A similar calculation is done using the psi term in the initial state vector. It is calculated as -velocity * steer_value / Lf * latency. This seem to smoothen the response of the car when handling curves.
