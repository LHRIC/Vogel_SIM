function [C] = Gear_Acc_Vel_Curves(acc_vel_data,gear_ratio,final_drve)

    C = [];

    figure(); hold on;
        title("Acceleration Curve ")
        xlabel("Velocity (m/s)")
        ylabel("Acceleration (m/s^2)")
        
    for i = 1:length(gear_ratio)
        R = gear_ratio(i)*final_drve;
        C = [C, [acc_vel_data(:,1).*R, acc_vel_data(:,2)./R]];
        plot(acc_vel_data(:,1).*R, acc_vel_data(:,2)./R)
    end 
        

        ldg = legend(string(1:length(gear_ratio)));
        title(ldg,"Gear")
end 