function J = cost_training(X,U,e,data)
    global p1;
    global Xtrain;
    global alpha;
    global beta;
    global gamma;
    global last_ref;
    
    p = data.PredictionHorizon;
    
    U1 = U(1:p,:);
    
    X = X(2:p+1,:);
    
    reference = p1(2:end,:);
    
    J = 0.0;    
    
    %Cost function for tracking the reference trajectory
    for j=1:14
        J = J + alpha * (sum((X(:,j)-reference(:,j)).^2));
    end

    %Cost function for smoothing input of MPC
    J = J + gamma * sum((U1(1,:) - last_ref).^2);  
    
    for i=1:p-1
        J = J + gamma * sum((U1(i+1,:) - U1(i,:)).^2);
    end
    
end