function ThrustConstraint  = defThrustConFcn(outputs, states, Controls)
% Defender 3DoF Thrust Constraint

ThrustConstraint = Controls(1)^2 + Controls(2)^2 + Controls(3)^2;

end