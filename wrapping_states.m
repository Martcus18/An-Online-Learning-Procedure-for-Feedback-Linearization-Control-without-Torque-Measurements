function q = wrapping_states(q)
    
    alpha_temp = mod(q(1),2*pi);

    if(abs(alpha_temp) >= pi)
        q(1) = -(2*pi - alpha_temp);
    else
        q(1) = alpha_temp;
    end

    alpha_temp = mod(q(2),2*pi);

    if(abs(alpha_temp) >= pi)
         q(2) = -(2*pi - alpha_temp);
    else
         q(2) = alpha_temp;
    end
    
    alpha_temp = mod(q(3),2*pi);

    if(abs(alpha_temp) >= pi)
         q(3) = -(2*pi - alpha_temp);
    else
         q(3) = alpha_temp;
    end
    
    alpha_temp = mod(q(4),2*pi);

    if(abs(alpha_temp) >= pi)
         q(4) = -(2*pi - alpha_temp);
    else
         q(4) = alpha_temp;
    end
    
    alpha_temp = mod(q(5),2*pi);

    if(abs(alpha_temp) >= pi)
         q(5) = -(2*pi - alpha_temp);
    else
         q(5) = alpha_temp;
    end
    
    alpha_temp = mod(q(6),2*pi);

    if(abs(alpha_temp) >= pi)
         q(6) = -(2*pi - alpha_temp);
    else
         q(6) = alpha_temp;
    end
    
    alpha_temp = mod(q(7),2*pi);

    if(abs(alpha_temp) >= pi)
         q(7) = -(2*pi - alpha_temp);
    else
         q(7) = alpha_temp;
    end
    return;
end