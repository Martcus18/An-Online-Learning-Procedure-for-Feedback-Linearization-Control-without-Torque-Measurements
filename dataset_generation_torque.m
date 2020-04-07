function dataset_generation(q,qold,tau,ref,pred,kuka,controller)
    
    global Xtrain;
    global Ytrain;
    global DeltaT;
    global acc;
    global gramian_error;
    global torque_measurements;
    global torque_measurements_filtered;
    global real_torque;
    global Ytrain_not_filtered;
    global Ytrain_correct;
    
    qold = reshape(qold,[1,14]);
    
    %Gramian for calculating the torque measuerements correct
    u_nominal_1 = inverse_model_double_integrator([q(1);q(8)], [qold(1);qold(8)],DeltaT);
    u_nominal_2 = inverse_model_double_integrator([q(2);q(9)], [qold(2);qold(9)],DeltaT);
    u_nominal_3 = inverse_model_double_integrator([q(3);q(10)], [qold(3);qold(10)],DeltaT);
    u_nominal_4 = inverse_model_double_integrator([q(4);q(11)], [qold(4);qold(11)],DeltaT);
    u_nominal_5 = inverse_model_double_integrator([q(5);q(12)], [qold(5);qold(12)],DeltaT);
    u_nominal_6 = inverse_model_double_integrator([q(6);q(13)], [qold(6);qold(13)],DeltaT);
    u_nominal_7 = inverse_model_double_integrator([q(7);q(14)], [qold(7);qold(14)],DeltaT);
    
    %%Complete input reconstructed
    u_nominal = [u_nominal_1,u_nominal_2,u_nominal_3,u_nominal_4,u_nominal_5,u_nominal_6,u_nominal_7];
    
    temp = acc - u_nominal;

    gramian_error = vertcat(gramian_error,temp);
    
    %Real torque associated with the system evolution
    
    tau2 = gravityTorque(controller,qold(1:7)) + velocityProduct(controller,qold(1:7),qold(8:14)) + (massMatrix(controller,qold(1:7)) * u_nominal')' + pred;
    
    real_torque = vertcat(real_torque,tau2);
    
    %Noise of torque sensor, gaussian distribution of mean 1 and sigma
    %0.005
    
    noise = random('norm',1,0.005,size(tau2(1:7)));
    
    %Introducing noise on the torque measuerements
    tau2 = tau2 .* noise;
    
    torque_measurements = vertcat(torque_measurements,tau2);
    
    
    %Calculating Delta Torque
    DeltaU = tau - tau2;
    
    Yk_correct = tau - real_torque(end,:) + pred;

    Yk = (DeltaU + pred);
    
    Xk = [qold,u_nominal];
    
    %Update of the dataset
    
    Xtrain = vertcat(Xtrain,Xk);
    
    Ytrain = vertcat(Ytrain,Yk);
    
    Ytrain_not_filtered = vertcat(Ytrain_not_filtered,Yk);
    
    Ytrain_correct = vertcat(Ytrain_correct,Yk_correct);
    
    %If the dataset incorporated is larger of 30 samples, starting to
    %filter all the dataset cutting high frequency components
    if(size(Ytrain,1) > 30)
        Ytrain = lowpass(Ytrain,0.3);
    end
    
end
