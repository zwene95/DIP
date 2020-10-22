function [H_linPseudo] = measLinPseudo(z_true)
%measLinPseudo Pseudo linearized measurement function 
%   Detailed explanation goes here
%   z_true = [azimuth elevation]; true measurement vecot
    
    az = z_true(1);
    el = z_true(2);    

    H_linPseudo = [
        1 0 sin(el) * cot(az) 0 0 0
        0 1 cos(el) * cot(az) 0 0 0
    ];

end

