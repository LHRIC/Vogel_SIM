function [C] = Gear_Acc_Vel_Curves(acc_vel_data,gear_ratio,final_drive, plot_data)

    C = [];

    if plot_data
        figure(); hold on;
            title("Acceleration Curve ")
            xlabel("Velocity (m/s)")
            ylabel("Acceleration (m/s^2)")
    end 

    for i = 1:length(gear_ratio)
        R = gear_ratio(i)*final_drive;
        C = [C, [acc_vel_data(:,1).*R, acc_vel_data(:,2)./R]];
        if plot_data
            plot(acc_vel_data(:,1).*R, acc_vel_data(:,2)./R)
        end 
    end 
        
    if plot_data
        ldg = legend(string(1:length(gear_ratio)));
        title(ldg,"Gear")
    end 
end 