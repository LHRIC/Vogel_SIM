function [t] = Torque_Curve_Optimizer(engine_acc_vel,engine_gear_ratio,final_drive_ratio,distance, plot_data)

    % combined_engine_acc_vel = Torque_Curve_Optimizer(engine_acc_vel,engine_gear_ratio,final_drive_ratio,75);
    engine_acc_vel = Gear_Acc_Vel_Curves(engine_acc_vel, engine_gear_ratio, final_drive_ratio, plot_data);
    
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
%                 disp("INTERSECTION")
                x0Int = x0I(1) + min_I; % finds index of intersection in gear i-1 domain
                xqInt = xqI(1) + min_I; % finds index of intersection in gear i domain
                
                % Combines gear i and i-1 data at intersection. Trims right
                % side of gear i and left side of gear i-1
                combined_engine_acc_vel = [combined_engine_acc_vel(1:x0Int,:); [xq(xqInt:end), p(xqInt:end)]];
    
            else % intersection does not occur
                
%                 disp("NO INTERSECTION")
                
                % Combines gear i and i-1 data at redline. Trims left side of
                % gear i-1
                combined_engine_acc_vel = [combined_engine_acc_vel; [xq(xqI(2)+1:end), p(xqI(2)+1:end)]];
            end
        end
    end 
    
    if plot_data
        figure();
            plot(combined_engine_acc_vel(:,1), combined_engine_acc_vel(:,2))
            title("Ideal Acceleration Curve ")
            xlabel("Velocity (m/s)")
            ylabel("Acceleration (m/s^2)")
    end 
    
    % Distance/Velocity calculation
    distance_velocity_curve = cumtrapz(combined_engine_acc_vel(:,1),combined_engine_acc_vel(:,1)./combined_engine_acc_vel(:,2));

    % Re-fits dis/vel data for to the closest integer domain with a constant step size
    x = combined_engine_acc_vel(:,1); y = distance_velocity_curve; % pulls data for curve
    xq = ceil(x(1)):0.1:floor(x(end)); % new domain with step 0.1
    distance_velocity_curve = pchip(x, y, xq); % interpolates data over new domain
    
    % Time/Velocity calculation
    time_velocity_curve = cumtrapz(combined_engine_acc_vel(:,1), 1./combined_engine_acc_vel(:,2));
    
    % Re-fits dis/vel data for to the closest integer domain with a constant step size
    x = combined_engine_acc_vel(:,1); y = time_velocity_curve; % pulls data for curve
    xq = ceil(x(1)):0.1:floor(x(end)); % new domain with step 0.1
    time_velocity_curve = pchip(x, y, xq); % interpolates data over new domain

    % Compiles time distance data 
    time_distance_curve = [distance_velocity_curve.', time_velocity_curve.'];

    % Curve plotting
    if plot_data
        figure();
            plot(xq, distance_velocity_curve)
            title("Distance/Speed Curve")
            xlabel("Velocity (m/s)")
            ylabel("Distance (m)")
    
        figure();
            plot(xq, time_velocity_curve)
            title("Time/Speed Curve")
            xlabel("Velocity (m/s)")
            ylabel("Time (s)")
    
         figure();
            plot(time_distance_curve(:,1), time_distance_curve(:,2))
            title("Time/Distance Curve")
            xlabel("Distance (m)")
            ylabel("Time (s)")
    end 
    % Distance time search 
    [~, minI] = min(abs(time_distance_curve(:,1)-distance));
    t = time_distance_curve(minI,2);
    
end 