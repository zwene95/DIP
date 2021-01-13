function [Results] = logResults(Setup, Problem)
% Write output results file
% [results] = logResults(setup, problem)

    Results = struct();
    
    Results.Time = Problem.RealTime;
    
    %% Defender 
    
    % Translational States
    
    % Position
    Results.Defender.States.Pos = [ 
        Problem.StateValues(find(ismember(Problem.StateNames,'x'),1),:)
        Problem.StateValues(find(ismember(Problem.StateNames,'y'),1),:)
        Problem.StateValues(find(ismember(Problem.StateNames,'z'),1),:)
    ];
    % Velocity
    Results.defender.states.vel = [ 
        Problem.StateValues(find(ismember(Problem.StateNames,'u'),1),:)
        Problem.StateValues(find(ismember(Problem.StateNames,'v'),1),:)
        Problem.StateValues(find(ismember(Problem.StateNames,'w'),1),:)
    ];
    % Accleration
    Results.Defender.States.Acc = [
        Problem.StateDotValues(find(ismember(Problem.StateDotNames,'u_dot'),1),:)
        Problem.StateDotValues(find(ismember(Problem.StateDotNames,'v_dot'),1),:)
        Problem.StateDotValues(find(ismember(Problem.StateDotNames,'w_dot'),1),:)
    ];

    % Rotational States    
    if Setup.ModelOptions.Defender.SixDoF
        
        % Attitude
        Results.Defender.States.Att = [ 
            Problem.StateValues(find(ismember(Problem.StateNames,'phi'),1),:)   
            Problem.StateValues(find(ismember(Problem.StateNames,'theta'),1),:) 
            Problem.StateValues(find(ismember(Problem.StateNames,'psi'),1),:)   
        ];
        % Rotation
        Results.Defender.States.Rot = [ 
            Problem.StateValues(find(ismember(Problem.StateNames,'p'),1),:) 
            Problem.StateValues(find(ismember(Problem.StateNames,'q'),1),:) 
            Problem.StateValues(find(ismember(Problem.StateNames,'r'),1),:) 
        ];
        % Motor States
        if Setup.modelOptions.defender.MotorLag
            Results.Defender.States.Motor.W1 = ...
                Problem.StateValues(find(ismember(Problem.StateNames,'w1'),1),:);
            Results.Defender.States.Motor.W2 = ...
                Problem.StateValues(find(ismember(Problem.StateNames,'w2'),1),:);
            Results.Defender.States.Motor.W3 = ...
                Problem.StateValues(find(ismember(Problem.StateNames,'w3'),1),:);
            Results.Defender.States.Motor.W4 = ...
                Problem.StateValues(find(ismember(Problem.StateNames,'w4'),1),:);
            
        end
        
    end
    
    % Controls    
    if Setup.ModelOptions.Defender.SixDoF
        
        if Setup.ModelOptions.Defender.MotorLag
            
            Results.Defender.Controls.W1 = ...
                Problem.ControlValues(find(ismember(Problem.ControlNames,'w1_cmd'),1),:);
            Results.Defender.Controls.W2 = ...
                Problem.ControlValues(find(ismember(Problem.ControlNames,'w2_cmd'),1),:);
            Results.Defender.Controls.W3 = ...
                Problem.ControlValues(find(ismember(Problem.ControlNames,'w3_cmd'),1),:);
            Results.Defender.Controls.W4 = ...
                Problem.ControlValues(find(ismember(Problem.ControlNames,'w4_cmd'),1),:);
        else
            Results.Defender.Controls.W1 = ...
                Problem.ControlValues(find(ismember(Problem.ControlNames,'w1'),1),:);
            Results.Defender.Controls.W2 = ...
                Problem.ControlValues(find(ismember(Problem.ControlNames,'w2'),1),:);
            Results.Defender.Controls.W3 = ...
                Problem.ControlValues(find(ismember(Problem.ControlNames,'w3'),1),:);
            Results.Defender.Controls.W4 = ...
                Problem.ControlValues(find(ismember(Problem.ControlNames,'w4'),1),:);
            
        end
        
    else
        
        Results.Defender.Controls.Tx = ...
            Problem.ControlValues(find(ismember(Problem.ControlNames,'T_x'),1),:);
        Results.Defender.Controls.Ty = ...
            Problem.ControlValues(find(ismember(Problem.ControlNames,'T_y'),1),:);
        Results.Defender.Controls.Tz = ...
            Problem.ControlValues(find(ismember(Problem.ControlNames,'T_z'),1),:);       
        
    end
    
    % Aero
    if Setup.ModelOptions.Defender.Aero
        Results.Defender.Aero = [   
            Problem.OutputValues(find(ismember(Problem.OutputNames,'FDAD_x'),1),:)   
            Problem.OutputValues(find(ismember(Problem.OutputNames,'FDAD_y'),1),:)
            Problem.OutputValues(find(ismember(Problem.OutputNames,'FDAD_z'),1),:)
        ];
    end
    
    %% Invader
    
    % States
    
    if strcmp(Setup.ModelOptions.Invader.Type, 'Quad1')
        
        % Position
        Results.Invader.States.Pos = [  
            Problem.StateValues(find(ismember(Problem.StateNames,'x_inv'),1),:)
            Problem.StateValues(find(ismember(Problem.StateNames,'y_inv'),1),:)
            Problem.StateValues(find(ismember(Problem.StateNames,'z_inv'),1),:)
        ];  
        % Velocity
        Results.Invader.States.Vel = [  
            Problem.StateDotValues(find(ismember(Problem.StateDotNames,'x_inv_dot'),1),:)
            Problem.StateDotValues(find(ismember(Problem.StateDotNames,'y_inv_dot'),1),:)
            Problem.StateDotValues(find(ismember(Problem.StateDotNames,'z_inv_dot'),1),:)
        ];  
        
    elseif strcmp(Setup.ModelOptions.Invader.Type, 'Quad2')
        
        % Position
        Results.invader.states.pos = [  
            Problem.StateValues(find(ismember(Problem.StateNames,'x_inv'),1),:)
            Problem.StateValues(find(ismember(Problem.StateNames,'y_inv'),1),:)
            Problem.StateValues(find(ismember(Problem.StateNames,'z_inv'),1),:)
        ];
        % Velocity
        Results.Invader.States.Vel = [  
            Problem.StateValues(find(ismember(Problem.StateNames,'u_inv'),1),:)
            Problem.StateValues(find(ismember(Problem.StateNames,'v_inv'),1),:)
            Problem.StateValues(find(ismember(Problem.StateNames,'w_inv'),1),:)
        ];
    
    end    
    
    % Controls
    
    if strcmp(Setup.ModelOptions.Invader.Type, 'Quad1')       
        
        Results.Invader.Controls.vI_x = ...
            Problem.ControlValues(find(ismember(Problem.ControlNames,'vI_x'),1),:);
        Results.Invader.Controls.vI_y = ...
            Problem.ControlValues(find(ismember(Problem.ControlNames,'vI_y'),1),:);
        Results.Invader.Controls.vI_z = ...
            Problem.ControlValues(find(ismember(Problem.ControlNames,'vI_z'),1),:);
        
    elseif strcmp(Setup.ModelOptions.Invader.Type, 'Quad2')
        
        Results.Invader.Controls.T_x_inv = ...
            Problem.ControlValues(find(ismember(Problem.ControlNames,'T_x_inv'),1),:);
        Results.Invader.Controls.T_y_inv = ...
            Problem.ControlValues(find(ismember(Problem.ControlNames,'T_y_inv'),1),:);
        Results.Invader.Controls.T_z_inv = ...
            Problem.ControlValues(find(ismember(Problem.ControlNames,'T_z_inv'),1),:);
        
    end    
    
    %% LOS
    pDIO = Results.Invader.States.Pos - Results.Defender.States.Pos;
    Results.LOS.Elevation = atan2(-pDIO(3,:),sqrt(pDIO(1,:).^2 + pDIO(2,:).^2));
    Results.LOS.Azimuth = atan2(pDIO(2,:),pDIO(1,:));
    
    %% Seeker
    if Setup.ModelOptions.Defender.SixDoF
        Results.Seeker.Elevation = Results.LOS.Elevation - Results.Defender.States.Att(2,:);
        Results.Seeker.Azimuth   = Results.LOS.Azimuth   - Results.Defender.States.Att(3,:);    
    end
    
    %% Target Data

    

end
