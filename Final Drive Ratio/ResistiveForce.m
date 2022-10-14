function [Resistance] = ResistiveForce(velocity, coeff_drag)

    drag_force = coeff_drag * velocity.^2;
   
    % Resistance = drag_force;
    Resistance = 0;
end 