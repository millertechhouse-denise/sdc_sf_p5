# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

Project By Denise Miller

## The Model
The system uses a Kinetic Model to define the state and actuation of the car.  

The following parameters are used:

State:
-The x position of the car (x)
-The y position of the carn(y)
-The orientation angle of the car (psi)
-The velocity of the car (v)

Control Inputs:
-The steering angle of the car (delta)
-The throttle of the car(a)

The state calculations were done as follows:
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

The weights were set as follows:

const double cte_weight = 200.;
const double epsi_weight = 200.;
const double v_weight = 0.5; 
const double delta_weight = 1.; 
const double a_weight = 1.; 
const double delta_delta_weight = 600.; 
const double a_delta_weight = 1.; 


The intent of this was to provide a higher weight to the cross track error and heading error, while reducing the impact of the velocity error.  In addition, higher weight was added to the change in steering angle to prevent large changes in steering angles.

## Timestep Length and Elapsed Duration (N & dt)
N and dt are used to determine the prediction horizon.  The variables were set as follows:

size_t N = 10;
double dt = 0.05;

In this model, the prediction horizon is 0.5 seconds.  This is a balance of having enough to generate a path that matches the road, but not more than you can reliably predict (without more information of the road).  As a reference speed of 40mph, 0.5 seconds in sufficient.



## Polynomial Fitting and MPC Preprocessing
The waypoints to the path are preprocessed by converting them from global coordinates to the car coordinates as follows:

auto way_points = Eigen::MatrixXd(2, ptsy.size());
for (auto i=0; i < ptsy.size() ; ++i){
		way_points(0,i) =   cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
		way_points(1,i) =  -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);  
} 
					
Eigen::VectorXd ptsx_tx = way_points.row(0);
Eigen::VectorXd ptsy_tx = way_points.row(1);

A 3rd degree polynomial was used to fit the path as follows:

auto coeffs = polyfit(ptsx_tx, ptsy_tx, 3);

## Model Predictive Control with Latency
The latency used is this project is 100ms, so the dt was set to half of that: 50ms. To account for the latency, I predicted the updated state as follows:


double dt = 0.1;  //100ms latency
x = v * dt;
psi = -v * delta * dt / Lf;
v = v + acceleration * dt;
cte = cte + v * sin(epsi) * dt;
epsi = epsi + v * -delta / Lf * dt;

This was used as the input into the solver.