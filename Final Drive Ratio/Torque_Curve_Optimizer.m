function [C] = Torque_Curve_Optimizer(engine_acc_vel,engine_gear_ratio,final_drive_ratio,distance)

    engine_acc_vel = Gear_Acc_Vel_Curves(engine_acc_vel, engine_gear_ratio, final_drive_ratio);

    % lienarize and combine domains for every gear ratio
    combined_engine_acc_vel = [];

    for i = 1:length(engine_gear_ratio)
        % Re-fits acc/vel data for gear i to the closest integer domain with a constant step size
        x = engine_acc_vel(:, 2*i-1); y = engine_acc_vel(:,2*i); 
        xq = ceil(x(1)):1:floor(x(end)); % New Domain with step 1
        p = pchip(x, y, xq); % New Data set interpolated over new domain
        
        % transposes two data sets 
        xq = xq.'; p = p.';
        
        % fist iteration check
        if i == 1
            combined_engine_acc_vel = [xq, p];
        else
    
            x = combined_engine_acc_vel(:,1); y = combined_engine_acc_vel(:,2); % placeholder variables
            int = intersect(x,xq.'); % find intersection of domains
            
            xqI = [find(xq==int(1)),find(xq==int(end))]; % index range of intersection in xq
            xI = [find(x==int(1)), find(x==int(end))]; % index range of intersection in x
            
            diff = y(xI(1):xI(end) - p(xqI(1):xqI(end))); % difference between both curves (old - new)
    
            [min_val, min_I] = min(abs(diff));  % finds the index and value of minimum difference
    
            if min_val < 0.1 % intersection occurs 
                
                xI = xI + min_I; % finds index of intersection in x
                xqI = xqI + min_I; % finds index of intersection in xqI
    
                combined_engine_acc_vel = [combined_engine_acc_vel(1:xI), [xq(xqI:end), p(xqI:end)]];
    
            else % intersection does not occur
    
                combined_engine_acc_vel = [combined_engine_acc_vel; [xq(xqI(2):end), p(xqI(2):end)]];
            end
        end
    end 
    
    plot(combined_engine_acc_vel(:,1), combined_engine_acc_vel(:,2))

    
    
    C = combined_engine_acc_vel;
end 