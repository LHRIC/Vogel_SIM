function [Resistance] = ResistiveForce(velocity, vehicle_parameters, coeff_inputs)

%    Cr0 = 0.0125;
%    Crp = 2.5*(10.^-4);
%    C_drag = 1.1
%    C_down = 2.9
    
    % Input Unpacking
    mass = vehicle_parameters(1); 
    C_down = coeff_inputs(1);
    C_drag = coeff_inputs(2);
    Cr0 = coeff_inputs(3);
    Crp = coeff_inputs(4);
    
    % Coeff calculations
    Cr = Cr0*(1+Crp*(velocity.^2));
    
    % Resistance calculations 
    down_force = C_down * velocity.^2 * 0.5 * 1.225; 
    drag_force = C_drag * velocity.^2 * 0.5 * 1.225;
    F_rolling_resistance = Cr*(down_force + mass * 9.81);
    
    Resistance = drag_force + F_rolling_resistance;
    % Resistance = 0;
end 