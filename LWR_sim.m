%Simulation of the LWR kuka robot
parameters;
global acc_est;
global vel_est;
vel_est = zeros(0,0);
acc_est = zeros(0,0);

while(t0 < tf)    
    disp(t0);
    if(t0 >= DeltaT)
        if(mod(index,GP_sampling) == 0)
%         if(mod(index,10000) == 0)
            %For using Gramian reconstruction uncomment dataset_generation
            %and comment dataset_generation_torque
            
            dataset_generation(q0,qold,TauFL,Uref,prediction,kuka,controller);
            
            %For using torque measurements uncomment
            %dataset_generation_torque and comment dataset_generation
%             dataset_generation_torque(q0,qold,TauFL,Uref,prediction,kuka,controller);
            new_point = true(1);
            start = true(1);
            indeces = vertcat(indeces,index-1);
        end
        
         %Dataset normalization
         Xtemp = dataset_normalization(Xtrain);
         Ytemp = dataset_normalization(Ytrain); 
         
           if(new_point)
                     start = true([1,1]);
                     T1 = tic;
                     %GP fitting
                     gp1 = fitrgp(Xtemp,Ytemp(:,1),'BasisFunction','None');
                     gp2 = fitrgp(Xtemp,Ytemp(:,2),'BasisFunction','None');
                     gp3 = fitrgp(Xtemp,Ytemp(:,3),'BasisFunction','None');
                     gp4 = fitrgp(Xtemp,Ytemp(:,4),'BasisFunction','None');
                     gp5 = fitrgp(Xtemp,Ytemp(:,5),'BasisFunction','None');
                     gp6 = fitrgp(Xtemp,Ytemp(:,6),'BasisFunction','None');
                     gp7 = fitrgp(Xtemp,Ytemp(:,7),'BasisFunction','None');

                     telapsed = toc(T1);
                     tic_toc_1 = vertcat(tic_toc_1,telapsed);
         end
         %Reference trajectory for the MPC  
         p1 = Xd(index:index+prediction_horizon,:);
         
        if(mod(index,MPC_sampling) == 0)
            %MPC optimization
            T3 = tic;
            [mv,options,info] = nlmpcmove(nlobj_train,q0,u0);
            telapsed3 = toc(T3);
            tic_toc_3 = vertcat(tic_toc_3,telapsed3);
            %Check if the MPC finds a feasible solution
             if(info.ExitFlag == -2)
                 disp('------------');
                 disp('MPC crashed');
                 return;
             end
             %Updates of the cumulative costs
                 Costs = vertcat(Costs,info.Cost);
                 Uref = mv';
         else
             Uref = last_ref;
        end
         
         %Normalization of the query point
         if(min(Xtrain) ~= max(Xtrain))
            q0_predict = ([q0,Uref] - min(Xtrain)) ./ (max(Xtrain) - min(Xtrain));
         else
            q0_predict = zeros(1,21);
         end
         
         if(start)             
             T2 = tic;
             %Prediction of normalized disturbance
             [pred1,var1] = predict(gp1,q0_predict);
             [pred2,var2] = predict(gp2,q0_predict);
             [pred3,var3] = predict(gp3,q0_predict);
             [pred4,var4] = predict(gp4,q0_predict);
             [pred5,var5] = predict(gp5,q0_predict);
             [pred6,var6] = predict(gp6,q0_predict);
             [pred7,var7] = predict(gp7,q0_predict);        
             telapsed2 = toc(T2);
             tic_toc_2 = vertcat(tic_toc_2,telapsed2);

             prediction = [pred1,pred2,pred3,pred4,pred5,pred6,pred7];
             %De normalization of the prediction
             prediction = prediction .* (max(Ytrain) - min(Ytrain)) + min(Ytrain);
             prediction_variance = [var1,var2,var3,var4,var5,var6,var7] .* (max(Ytrain) - min(Ytrain));                
         end
             
         last_ref = Uref;
         new_point = false(1);
         
    else 
         prediction = zeros(1,7);
         prediction_variance = zeros(1,7);
         Uref = zeros(1,7);
    end
    
    %Keep prediction to zero for the first $acquisition_period
    if(t0 < acquisition_period)
%     if(t0 < 1000000*Ts)
        prediction = zeros(1,7);
    end
    
    qold = q0;
    
    tspan =[t0,t0+DeltaT];
    
    %Friction calculation
    
%     disturbance =  0.01*(1.0 ./ (1.0 + exp(-100*q0(8:14))) + A .* q0(8:14) + B * exp(-(q0(8:14) / v_str).^2) .* (q0(8:14)/v_str));
    disturbance =  0.0*(1.0 ./ (1.0 + exp(-100*q0(8:14))) + A .* q0(8:14) + B * exp(-(q0(8:14) / v_str).^2) .* (q0(8:14)/v_str));
    
    friction_plot = vertcat(friction_plot,disturbance);
    
    %Nominal torque of feedback linearization + GP predictions
    TauFL = gravityTorque(controller,q0(1:7)) + velocityProduct(controller,q0(1:7),q0(8:14)) + (massMatrix(controller,q0(1:7)) * Uref')' + prediction;
    
    torque_fl = vertcat(torque_fl,TauFL);
    
    reference_mpc = vertcat(reference_mpc,Uref);
    
    predict_plot = vertcat(predict_plot,prediction);
    
    %Calculation of the DeltaM between real model and nominal one
    
    DeltaM = - massMatrix(controller,q0(1:7)) + massMatrix(kuka,q0(1:7));
    
    %Calculation of the DeltaNi between real model and nominal one
    
    DeltaNi =  velocityProduct(kuka,q0(1:7),q0(8:14)) + gravityTorque(kuka,q0(1:7)) - velocityProduct(controller,q0(1:7),q0(8:14)) - gravityTorque(controller,q0(1:7));
    
    %Calculation of the acceleration of the real system
%     acc = (inv(massMatrix(kuka,q0(1:7))) * (TauFL - disturbance - velocityProduct(kuka,q0(1:7),q0(8:14)) - gravityTorque(kuka,q0(1:7)))')';
    acc = (inv(massMatrix(controller,q0(1:7))) * (TauFL - disturbance - velocityProduct(controller,q0(1:7),q0(8:14)) - gravityTorque(controller,q0(1:7)))')';
    
    torque_real = TauFL - disturbance;
    
    %Real disturbance acting on the system
    temp = (DeltaM * acc')' + DeltaNi + disturbance;
    
    disturbance_plot = vertcat(disturbance_plot,temp);
    
    diff_plot = vertcat(diff_plot,prediction - temp);
    
    accelerations = vertcat(accelerations,acc);
    
    variance = vertcat(variance,prediction_variance);
    
    %ODE45 integration of the system from t0 to t0+Ts
    [t,q] = ode45(@(t,q) kuka_lwr(t,q,acc),tspan,q0);
    
    t0 = t0+DeltaT;
    vel_k = (q(end,1:7) - q0(1:7)) ./ Ts;
    vel_est = vertcat(vel_est,vel_k);
    if(size(vel_est,1) > 1)
        acc_est = vertcat(acc_est,(vel_est(end,:) - vel_est(end-1,:))./ Ts);
    end
    q0 = q(end,:);
    
    %Wrapping the state between -pi to +pi
    q0 = wrapping_states(q0);
    
    joints = vertcat(joints,q0);
    
    time = vertcat(time,t0);
    %Updating the step
    index = index + 1;
end

dlmwrite("joints.txt",joints,"\t");
dlmwrite("reference_mpc.txt",reference_mpc,"\t");
dlmwrite("accelerations.txt",acceleration, "\t");
dlmwrite("prediction_plot.txt",predict_plot,"\t");
dlmwrite("disturbance_plot.txt",disturbance_plot,"\t");
dlmwrite("diff_plot.txt",diff_plot,"\t");
dlmwrite("variance.txt",variance,"\t");
dlmwrite("torque_FL.txt",torque_fl,"\t");

function dqdt = kuka_lwr(t,q,acc)
d2q = acc;
dq = q(8:14);
dqdt = [dq;d2q'];
end
