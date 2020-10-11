function defThrustConstraint  = defThrustConFcn(states, controls)
% Defender 3DoF Thrust Constraint

defThrustConstraint = controls(1)^2 + controls(2)^2 + controls(3)^2;

end