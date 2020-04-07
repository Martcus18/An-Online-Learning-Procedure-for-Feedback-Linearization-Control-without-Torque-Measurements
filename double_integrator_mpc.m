function dqdt = double_integrator_mpc(t,q,tau)
d2q = tau;
dqdt = [q(8:14);d2q];
end