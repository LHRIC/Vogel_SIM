function [engine_gear_curve] = Gear_Curves(engine_data, gear_ratio, final_drive, plot_data)

    engine_gear_curve = [];

    if plot_data
        figure(); hold on;
            title("Engine Force Curve ")
            xlabel("Velocity (m/s)")
            ylabel("Motive Force (N)")
    end 

    for i = 1:length(gear_ratio)
        R = gear_ratio(i)*final_drive;
        engine_gear_curve = [engine_gear_curve, [engine_data(:,1)./R, engine_data(:,2).*R]];
        if plot_data
            plot(engine_data(:,1)./R, engine_data(:,2).*R)
        end 
    end 

    if plot_data
        ldg = legend(string(1:length(gear_ratio)));
        title(ldg,"Gear")
    end 
end 