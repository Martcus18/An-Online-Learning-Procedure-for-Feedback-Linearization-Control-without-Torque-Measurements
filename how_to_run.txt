Prerequisites for running the code (all Matlab packages):

1- Matlab license
2- Model Predictive Control ToolboxTM 
3- Statistics and Machine Learning ToolboxTM
4- Signal Processing ToolboxTM
5- Robotics System ToolboxTM

How to run:

1- Execute the main script LWR_sim.m 
2- After the end of the script, it will be written as .txt file with different quantities

3- In order to execute simulation with no GP correction, modify GP_sampling and acquisition_period variables,
imposing very large values in the parameters.m file (100000 is ok).
4- In order to execute simulation with GP correction, modify the GP_sampling step as needed (that values says for how many steps the GP is updated)
and acquisition_period variable (corresponding to how many steps the GP stays silent and just update the dataset). Good values for both of them are after
5 for GP_sampling and 10 for acquisition_period.

5- For modifying the MPC frequency, change the variable MPC_sampling in the file parameters.m .
This value represents for how many steps the MPC performs the optimization, best value for performance is 1, so in each step, but it slows down the evolution.
A good trade off is imposing an MPC_sampling = 5.

6- In order to use GP correction with the torque measurements, comment "dataset_generation(q0,qold,TauFL,Uref,prediction,kuka,controller)" (line 12)
and uncomment "dataset_generation_torque(q0,qold,TauFL,Uref,prediction,kuka,controller)" (line 16);


The variables in the system correspond to:

joints(i,1:7) = q vector at the step i.
joints(i,8:14) = dq vector at step i.
Xd(i,1:7) = q reference vector at step i.
Xd(i,8:14) = dq reference vector at step i.
reference_mpc(i,:) = d2q reference vector provided by the MPC at step i.
accelerations(i,:) = accelerations vector at step i.
predict_plot(i,:) = prediction provided by the GPs at step i. In case of mute GP corresponds to a vector of zeros.
disturbance_plot(i,:) = disturbance acting on the system at step i.
variance(i,:) = variance associated with the GPs prediction at step i. In case of mute prediction is just empty vector.
torque_fl(i,:) = torque imposed on the motor, corresponding to the nominal torque of feedback linearization given a certain
acceleration reference (provided by the MPC) plus the prediction of the GPs (in case of mute gp prediction is zeros(1,7)).
Costs = cumulative cost function of the MPC from actual state to actual_state+prediction_horizon state.