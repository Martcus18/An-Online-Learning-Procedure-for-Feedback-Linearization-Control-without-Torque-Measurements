function dataset_generation(q,qold,tau,ref,pred,kuka,controller)
    
    global Xtrain;
    global Ytrain;
    global DeltaT;
    global acc;
    global gramian_error;
    
    qold = reshape(qold,[1,14]);
    
    %Using the gramian reconstruction for each separate joint
    u_nominal_1 = inverse_model_double_integrator([q(1);q(8)], [qold(1);qold(8)],DeltaT);
    u_nominal_2 = inverse_model_double_integrator([q(2);q(9)], [qold(2);qold(9)],DeltaT);
    u_nominal_3 = inverse_model_double_integrator([q(3);q(10)], [qold(3);qold(10)],DeltaT);
    u_nominal_4 = inverse_model_double_integrator([q(4);q(11)], [qold(4);qold(11)],DeltaT);
    u_nominal_5 = inverse_model_double_integrator([q(5);q(12)], [qold(5);qold(12)],DeltaT);
    u_nominal_6 = inverse_model_double_integrator([q(6);q(13)], [qold(6);qold(13)],DeltaT);
    u_nominal_7 = inverse_model_double_integrator([q(7);q(14)], [qold(7);qold(14)],DeltaT);
    
    %Complete input reconstructed
    u_nominal = [u_nominal_1,u_nominal_2,u_nominal_3,u_nominal_4,u_nominal_5,u_nominal_6,u_nominal_7];
    
    %For checking difference between acceleration of the system and the one
    %reconstructed through the gramian
    temp = acc - u_nominal;

    gramian_error = vertcat(gramian_error,temp);
    
    %Input training calculation
    DeltaU = (massMatrix(controller,qold(1:7)) * (ref - u_nominal)')';
    
    Yk = (DeltaU + pred);
    
    Xk = [qold,u_nominal];
    
    %Dataset Updating
    Xtrain = vertcat(Xtrain,Xk);
    
    Ytrain = vertcat(Ytrain,Yk); 
end