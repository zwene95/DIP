function [setup_out] = checkModelOptions(setup)
%CHECKOPTIONS Checks model contradictions

    setup_out = setup;

    
    if ~setup.modelOptions.defender.SixDoF
        setup_out.modelOptions.defender.MotorLag = 0;
    end    
    
    if ~isequal(setup_out,setup)
        warning('Model Options were modified due to contradiction');
    end
    

end

