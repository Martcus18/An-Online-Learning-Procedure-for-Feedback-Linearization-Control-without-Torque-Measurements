function angle_new = unwrap_angle(angle_old,angle_new)
        threshold = 0.1;
        if(((abs(abs(angle_old) - pi)) < threshold) && (sign(angle_old * angle_new) == -1))
        if(angle_old < 0)
            angle_new = -2*pi + abs(angle_new); 
        else
            angle_new =  2*pi - abs(angle_new); 
        end
     end

end