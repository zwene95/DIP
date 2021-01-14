function [Setup_out] = checkModelOptions(Setup)
%CHECKOPTIONS Checks model contradictions

    Setup_out = Setup;

    
    if ~Setup.ModelOptions.Defender.SixDoF
        Setup_out.ModelOptions.Defender.MotorLag = 0;
    end    
    
    if ~isequal(Setup_out,Setup)
        warning('Model Options were modified due to contradiction');
    end
    

end

