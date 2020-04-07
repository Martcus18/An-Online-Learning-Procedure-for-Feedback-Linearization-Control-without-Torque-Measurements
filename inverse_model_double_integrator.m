function u = inverse_model_double_integrator(q,qold,DeltaT)
    %Unwrapping state from SO(2) to Euclidean, so not considering
    %periodicity
    q(1) = unwrap_angle(qold(1),q(1));    
    %ZOH discretization of chain of integrators
    A = [1,DeltaT;0,1];
    B = [(DeltaT^2) / 2; DeltaT];    
    %Controllability gramian calculation W(1)
    W = B * B' +  A* B * B' * A';
    %Input reconstruction
    u = -(B' + (A*B)') * pinv(W) * (qold -q);
end
