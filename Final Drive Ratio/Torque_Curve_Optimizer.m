function [C] = Torque_Curve_Optimizer(engine_acc_vel,engine_gear_ratio,final_drive_ratio,distance)

    % combined_engine_acc_vel = Torque_Curve_Optimizer(engine_acc_vel,engine_gear_ratio,final_drive_ratio,75);
    engine_acc_vel = Gear_Acc_Vel_Curves(engine_acc_vel, engine_gear_ratio, final_drive_ratio);
    
    % lienarize and combine domains for every gear ratio
    combined_engine_acc_vel = [];
    
    for i = 1:length(engine_gear_ratio)
    
        % Re-fits acc/vel data for gear i to the closest integer domain with a constant step size
        x = engine_acc_vel(:, 2*i-1); y = engine_acc_vel(:,2*i); % pulls data for gear i
        xq = ceil(x(1)):1:floor(x(end)); % new domain with step 1
        p = pchip(x, y, xq); % interpolates data for gear i over new domain
        
        % transposes two data sets 
        xq = xq.'; p = p.';
        
        % fist iteration check
        if i == 1
            combined_engine_acc_vel = [xq, p];
        else
            
            % Checks for intersection between acc/vel curves for gears i, i-1
            x0 = combined_engine_acc_vel(:,1); y0 = combined_engine_acc_vel(:,2); % gear i-1 data
            int = intersect(x0,xq); % find intersection of gear domains (velocity both gears can use)
            
            xqI = [find(xq==int(1)),find(xq==int(end))]; % index range of intersection in gear i domain
            x0I = [find(x0==int(1)), find(x0==int(end))]; % index range of intersection in gear i-1 domain
            
            diff = y0(x0I(1):x0I(end) - p(xqI(1):xqI(end))); % difference between both curves (old - new)
    
            [min_val, min_I] = min(abs(diff));  % finds the index and value of minimum difference
    
            if min_val == 0 % intersection criteria
                disp("INTERSECTION")
                x0Int = x0I(1) + min_I; % finds index of intersection in gear i-1 domain
                xqInt = xqI(1) + min_I; % finds index of intersection in gear i domain
                
                % Combines gear i and i-1 data at intersection. Trims right
                % side of gear i and left side of gear i-1
                combined_engine_acc_vel = [combined_engine_acc_vel(1:x0Int,:); [xq(xqInt:end), p(xqInt:end)]];
    
            else % intersection does not occur
                
                disp("NO INTERSECTION")
                
                % Combines gear i and i-1 data at redline. Trims left side of
                % gear i-1
                combined_engine_acc_vel = [combined_engine_acc_vel; [xq(xqI(2):end), p(xqI(2):end)]];
            end
        end
    end 
    
    figure();
        plot(combined_engine_acc_vel(:,1), combined_engine_acc_vel(:,2))
end 