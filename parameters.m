clear all;
clc;

import lwr;
format long;

kuka = lwr("inertia.txt","masses.txt").robot;
controller = lwr("inertia_modified.txt","masses_modified.txt").robot;

%Global parameters for the framework
global Xtrain;
global Ytrain;
global Xtemp;
global Ytemp;
global DeltaT;
global acceleration;
global p1;
global alpha;
global gamma;
global last_ref;
global acc;
global gramian_error;
global torque_measurements;
global torque_measurements_filtered;
global real_torque;
global Ytrain_not_filtered;
global Ytrain_correct;

t0=0.0;
tf=7.0;

DeltaT = 0.001;
Ts = DeltaT;

%Friction parameters
friction_magnitude = 0.1;
A =0.5;
B = 0.5;
v_str = 0.01;

%Array initialization
joints = zeros(0,0);
time = zeros(1);
diff_plot = zeros(0,0);
predict_plot = zeros(0,0);
disturbance_plot = zeros(0,0);
accelerations = zeros(0,0);
variance = zeros(0,0);
last_ref = zeros(1,7);
variance = zeros(0,0);
disturbance_plot = zeros(0,0);
reference_mpc = zeros(0,0);
Xtrain = zeros(0,0);
Ytrain = zeros(0,0);
Costs = zeros(0,0);
friction_plot = zeros(0,0);

tic_toc_1 = zeros(0,0);
tic_toc_2 = zeros(0,0);
tic_toc_3 = zeros(0,0);
bounds = zeros(0,0);
torque_fl = zeros(0,0);
gramian_error = zeros(0,0);
torque_measurements = zeros(0,0);
torque_measurements_filtered = zeros(0,0);
real_torque = zeros(0,0);
Ytrain_not_filtered = zeros(0,0);
Ytrain_correct = zeros(0,0);

indeces = [];


%Logical variable 
new_point = false(1);
start = false(1);

%Sampling steps and free acquisition period
GP_sampling = 100000;
MPC_sampling = 1;
acquisition_period = 10*Ts;

%MPC cost function parameters
alpha = 1.0;
gamma = 0.01;

%MPCs parameters

nx = 14;
ny = 14;
nu = 7;
nlobj_train = nlmpc(nx,ny,nu);
u0 = zeros(1,7);
umax = 50.0;
vmax = 20;
theta_max = pi;
nlobj_train.Ts = Ts;
prediction_horizon = 3;
control_horizon = 3;
nlobj_test.Ts = Ts;
prediction_horizon_test = 3;
control_horizon_test = 3;

nlobj_train.PredictionHorizon = prediction_horizon;
nlobj_train.ControlHorizon = control_horizon;

nlobj_train.Model.StateFcn = @(q,u) double_integrator_mpc_prediction(q,u);
nlobj_train.Model.IsContinuousTime = true;

nlobj_train.Model.OutputFcn = @(q,u) q;


nlobj_train.Optimization.CustomCostFcn = @(X,U,e,nlobj_train) cost_training(X,U,e,nlobj_train);
nlobj_train.Optimization.ReplaceStandardCost = true;


%Constraints on states and actuations
%Acceleration constraints for the MPC
nlobj_train.ManipulatedVariables(1).Min = -umax;
nlobj_train.ManipulatedVariables(1).Max = +umax;
nlobj_train.ManipulatedVariables(2).Min = -umax;
nlobj_train.ManipulatedVariables(2).Max = +umax;
nlobj_train.ManipulatedVariables(3).Min = -umax;
nlobj_train.ManipulatedVariables(3).Max = +umax;
nlobj_train.ManipulatedVariables(4).Min = -umax;
nlobj_train.ManipulatedVariables(4).Max = +umax;
nlobj_train.ManipulatedVariables(5).Min = -umax;
nlobj_train.ManipulatedVariables(5).Max = +umax;
nlobj_train.ManipulatedVariables(6).Min = -umax;
nlobj_train.ManipulatedVariables(6).Max = +umax;
nlobj_train.ManipulatedVariables(7).Min = -umax;
nlobj_train.ManipulatedVariables(7).Max = +umax;


nlobj_train.States(1).Min = -theta_max;
nlobj_train.States(1).Max = +theta_max;
nlobj_train.States(2).Min = -theta_max;
nlobj_train.States(2).Max = +theta_max;
nlobj_train.States(3).Min = -theta_max;
nlobj_train.States(3).Max = +theta_max;
nlobj_train.States(4).Min = -theta_max;
nlobj_train.States(4).Max = +theta_max;
nlobj_train.States(5).Min = -theta_max;
nlobj_train.States(5).Max = +theta_max;
nlobj_train.States(6).Min = -theta_max;
nlobj_train.States(6).Max = +theta_max;
nlobj_train.States(7).Min = -theta_max;
nlobj_train.States(7).Max = +theta_max;

nlobj_train.States(8).Min = -vmax;
nlobj_train.States(8).Max = +vmax;
nlobj_train.States(9).Min = -vmax;
nlobj_train.States(9).Max = +vmax;
nlobj_train.States(10).Min = -vmax;
nlobj_train.States(10).Max = +vmax;
nlobj_train.States(11).Min = -vmax;
nlobj_train.States(11).Max = +vmax;
nlobj_train.States(12).Min = -vmax;
nlobj_train.States(12).Max = +vmax;
nlobj_train.States(13).Min = -vmax;
nlobj_train.States(13).Max = +vmax;
nlobj_train.States(14).Min = -vmax;
nlobj_train.States(14).Max = +vmax;



%Joint trajectory
Tsample = linspace(0,tf + 50*Ts,(tf + 50*Ts - t0)/Ts)';
Xd = 0.6*[repmat(cos(Tsample),[1,6]),zeros(length(Tsample),1),repmat(-sin(Tsample),[1,6]),zeros(length(Tsample),1)];

%Initialization of initial state and array of joints positions-velocities

q0 = Xd(1,:);
joints = q0;
index = 1;
options_ode = odeset('RelTol',1e-10,'AbsTol',1e-12);