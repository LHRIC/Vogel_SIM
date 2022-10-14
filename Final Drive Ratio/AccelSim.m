function [t] = AccelSim(engine_data, f_max, R_fun, effective_mass, engine_gear_ratio, final_drive_ratio, distance, plot_data)

    % combined_engine_acc_vel = Torque_Curve_Optimizer(engine_acc_vel,engine_gear_ratio,final_drive_ratio,75);
    engine_data = Gear_Curves(engine_data, engine_gear_ratio, final_drive_ratio, plot_data);
    
    % lienarize and combine domains for every gear ratio
    combined_engine_data = [];
    
    % Combines engine curves for each individual gear at motive force
    % crossover point
    for i = 1:length(engine_gear_ratio)

        % fist iteration check
        if i == 1
            % Re-fits acc/vel data for gear i to the closest integer domain with a constant step size
            x = engine_data(:, 2*i-1); y = engine_data(:,2*i); % pulls data for gear i
            xq = ceil(x(1)):0.1:floor(x(end)); % new domain with step 1
            p = pchip(x, y, xq); % interpolates data for gear i over new domain
            
            % transposes two data sets 
            xq = xq.'; p = p.';

            combined_engine_data = [xq, p];
        else
            
            gear_i_data = engine_data(:,2*i-1:2*i);
            combined_engine_data = CurveIntersect(combined_engine_data, gear_i_data);
        end
    end 
    
    % subtracts resistive forces
    for i = 1:length(combined_engine_data(:,2))
        motive_force = combined_engine_data(i,2) - R_fun(combined_engine_data(i,1));
        if motive_force < 0
            motive_force = 0;
        end
        combined_engine_data(i,2) = motive_force;
    end 

    %

    % flattens curve to take into account tractive limits 
    for i = 1:length(combined_engine_data(:,2))
        if combined_engine_data(i,2) > f_max
            combined_engine_data(i,2) = f_max;
        end 
    end 
    
    % Plots cumulative motive force
    if plot_data
        figure();
            plot(combined_engine_data(:,1), combined_engine_data(:,2))
            title("Ideal Motive Force Curve ")
            xlabel("Velocity (m/s)")
            ylabel("Motive Force (N)")
    end 
    
    % Converts motive force to acceleration
    combined_engine_data(:,2) = combined_engine_data(:,2)./effective_mass;

    % Distance/Velocity calculation
    distance_velocity_curve = cumtrapz(combined_engine_data(:,1),combined_engine_data(:,1)./combined_engine_data(:,2));

    % Re-fits dis/vel data for to the closest integer domain with a constant step size
    x = combined_engine_data(:,1); y = distance_velocity_curve; % pulls data for curve
    xq = ceil(x(1)):0.1:floor(x(end)); % new domain with step 0.1
    distance_velocity_curve = pchip(x, y, xq); % interpolates data over new domain
    
    % Time/Velocity calculation
    time_velocity_curve = cumtrapz(combined_engine_data(:,1), 1./combined_engine_data(:,2));
    
    % Re-fits dis/vel data for to the closest integer domain with a constant step size
    x = combined_engine_data(:,1); y = time_velocity_curve; % pulls data for curve
    xq = ceil(x(1)):0.1:floor(x(end)); % new domain with step 0.1
    time_velocity_curve = pchip(x, y, xq); % interpolates data over new domain

    % Compiles time distance data 
    time_distance_curve = [distance_velocity_curve.', time_velocity_curve.'];

    % Curve plotting
    if plot_data
        figure();
            plot(xq, distance_velocity_curve)
            % title("Distance/Speed Curve")
            xlabel("Velocity (m/s)")
            ylabel("Distance (m)")
    
        figure();
            plot(xq, time_velocity_curve)
            % title("Time/Speed Curve")
            xlabel("Velocity (m/s)")
            ylabel("Time (s)")
    
         figure();
            plot(time_distance_curve(:,1), time_distance_curve(:,2))
            % title("Time/Distance Curve")
            xlabel("Distance (m)")
            ylabel("Time (s)")
    end 

    % Drag time calculation 
    [min_val, minI] = min(abs(time_distance_curve(:,1)-distance));
    if min_val > 1
        t = time_distance_curve(minI,2) + (distance - min_val)/xq(minI);
    else 
        t = time_distance_curve(minI,2);
    end 
    
end 