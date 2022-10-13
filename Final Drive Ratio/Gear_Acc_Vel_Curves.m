function [C] = Gear_Acc_Vel_Curves(acc_vel_data,gear_ratio,final_drve)

    C = [];

    for i = 1:length(gear_ratio)
        R = gear_ratio(i)*final_drve;
        C = [C, [acc_vel_data(:,1).*R, acc_vel_data(:,2)./R]];
    end 

    % xq = min(C(:,1)):max(C(:,length(C(1,:))-1));
end 