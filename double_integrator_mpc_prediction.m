function dqdt = double_integrator_mpc_prediction(q,tau)
d2q = tau;
dq = q(8:14);
dqdt = [dq;d2q];
end