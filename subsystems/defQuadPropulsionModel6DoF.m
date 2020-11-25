function [FDPD,MDPD,Thrust] = defQuadPropulsionModel6DoF(...
    w1, w2, w3, w4,...
    T2W_max, RPM_max, cM, m, g,...
    r1, r2, r3, r4)
    %
    % Inputs:
    %   - Motor commands:
    %      - w1 [rpm]:      rotational speed motor 1
    %      - w2 [rpm]:      rotational speed motor 1
    %      - w3 [rpm]:      rotational speed motor 1
    %      - w4 [rpm]:      rotational speed motor 1
    %   - Propulsion:
    %     - cf [N/rpm^2]:   motor thrust coefficient
    %     - cm [Nm/rpm^2]:	motor torque coefficient
    %     - lever [m]:      motor lever length    
    %      - r1 [m]:        position motor 1
    %      - r2 [m]:        position motor 2
    %      - r3 [m]:        position motor 3
    %      - r4 [m]:        position motor 4
    % Outputs:
    %   - FBPB [Nm] (3, 1): propulsion force  vector in body-fixed frame
    %   - MBPB [Nm] (3, 1): propulsion moment vector in body-fixed frame

    
    % Propulsion force factor
    cf = m * g * T2W_max / 4 / RPM_max^2;                                     % 10000 ... RPM_max, 4.. Nr. of rotors
    
    % Propulsion forces
    f1 = [0; 0; - w1^2 * cf];
    f2 = [0; 0; - w2^2 * cf];
    f3 = [0; 0; - w3^2 * cf];
    f4 = [0; 0; - w4^2 * cf];
    FDPD = f1 + f2 + f3 + f4;
    Thrust = - FDPD(3);

    % Propulsion moments
    motor_torques = [0; 0; (-w1^2 + w2^2 -w3^2 + w4^2)*cM];
    MDPD =  cross(r1,f1) + cross(r2,f2) + cross(r3,f3) + cross(r4,f4) + motor_torques;

end